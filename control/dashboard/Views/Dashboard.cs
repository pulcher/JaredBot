using System;
using Terminal.Gui.App;
using Terminal.Gui.ViewBase;
using Terminal.Gui.Views;
using Terminal.Gui.Drawing;
using dashboard.Models;
using dashboard.Services;
using Terminal.Gui.Drivers;

namespace dashboard.Views;

public sealed class Dashboard : Window
{
    public MenuBar MenuBarV2 { get; private set; }
    public LogView LogViewer { get; private set; }

    private readonly TelemetryState _telemetry;
    private readonly SignalRListenerService _signalR;

    // Per-property views
    private readonly Label _rollValueLabel;
    private readonly Label _pitchValueLabel;
    private readonly Label _yawValueLabel;
    private readonly Label _c1ValueLabel;
    private readonly Label _c2ValueLabel;

    // Config frames: setpoint + PID + Kalman
    private readonly Label _setpointLabel;
    private readonly TextField _setpointInput;

    private readonly Label _pidPLabel;
    private readonly TextField _pidPInput;

    private readonly Label _pidILabel;
    private readonly TextField _pidIInput;

    private readonly Label _pidDLabel;
    private readonly TextField _pidDInput;

    private readonly Label _kalmanQLabel;
    private readonly TextField _kalmanQInput;

    private readonly Label _kalmanRLabel;
    private readonly TextField _kalmanRInput;

    private readonly Label _kalmanNLabel;
    private readonly TextField _kalmanNInput;

    public Dashboard()
    {
        Title = "Dashboard";

        InitializeMenu();

        var telemetryHeight = Dim.Absolute(3);

        // We will place 5 boxes side-by-side, each taking 20% width
        int boxCount = 5;
        Dim boxWidth = Dim.Absolute(12);

        // Roll
        var rollFrame = new FrameView
        {
            Title = "Roll",
            X = 0,
            Y = 1,
            Width = boxWidth,
            Height = telemetryHeight,
            TextAlignment = Alignment.End
        };
        _rollValueLabel = new Label
        {
            Text = "0.000",
            // Fill the frame horizontally so right-align works nicely
            X = 0,
            Y = Pos.Center(),
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };
        rollFrame.Add(_rollValueLabel);
        Add(rollFrame);

        // Pitch
        var pitchFrame = new FrameView
        {
            Title = "Pitch",
            X = Pos.Right(rollFrame),
            Y = 1,
            Width = boxWidth,
            Height = telemetryHeight
        };
        _pitchValueLabel = new Label
        {
            Text = "0.000",
            X = 0,
            Y = Pos.Center(),
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };
        pitchFrame.Add(_pitchValueLabel);
        Add(pitchFrame);

        // Yaw
        var yawFrame = new FrameView
        {
            Title = "Yaw",
            X = Pos.Right(pitchFrame),
            Y = 1,
            Width = boxWidth,
            Height = telemetryHeight
        };
        _yawValueLabel = new Label
        {
            Text = "0.000",
            X = 0,
            Y = Pos.Center(),
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };
        yawFrame.Add(_yawValueLabel);
        Add(yawFrame);

        // Custom1
        var c1Frame = new FrameView
        {
            Title = "C1",
            X = Pos.Right(yawFrame),
            Y = 1,
            Width = boxWidth,
            Height = telemetryHeight
        };
        _c1ValueLabel = new Label
        {
            Text = "0.000",
            X = 0,
            Y = Pos.Center(),
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };
        c1Frame.Add(_c1ValueLabel);
        Add(c1Frame);

        // Custom2
        var c2Frame = new FrameView
        {
            Title = "C2",
            X = Pos.Right(c1Frame),
            Y = 1,
            Width = boxWidth,
            Height = telemetryHeight
        };
        _c2ValueLabel = new Label
        {
            Text = "0.000",
            X = 0,
            Y = Pos.Center(),
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };
        c2Frame.Add(_c2ValueLabel);
        Add(c2Frame);

        //
        // --- CONFIG FRAMES (FAKE DATA FOR NOW) ---
        //

        // Start these config frames one row below the telemetry row.
        // Telemetry row: Y = 1, Height = 3 → next row is 4.
        int configY = 4;

        // Setpoint
        var setpointFrame = new FrameView
        {
            Title = "Setpoint",
            X = 0,
            Y = configY,
            Width = Dim.Absolute(20),
            Height = Dim.Absolute(4)
        };

        _setpointLabel = new Label
        {
            Text = "0.000",     // fake initial value
            X = 1,
            Y = 0,
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };

        _setpointInput = new TextField()
        {
            Text = "0.12345",
            X = Pos.AnchorEnd(10 + 1),
            Y = 1,
            Width = 10,                  // fixed width
            TextAlignment = Alignment.End,
            TextDirection = Terminal.Gui.Text.TextDirection.RightLeft_BottomTop
        };

        _setpointInput.KeyDown += (source, args) =>
        {
            if (args.KeyCode == KeyCode.Enter)
            {
                var text = _setpointInput.Text.ToString() ?? string.Empty;
                if (double.TryParse(text, out var value))
                {
                    _setpointLabel.Text = value.ToString("0.000");
                    // later: send to SignalR hub
                    LogViewer.AddMessage($"[CFG] Setpoint updated to {value:0.000}");
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid setpoint: '{text}'");
                }
            }
        };

        setpointFrame.Add(_setpointLabel);
        setpointFrame.Add(_setpointInput);
        Add(setpointFrame);

        // PID P
        var pidPFrame = new FrameView
        {
            Title = "PID P",
            X = Pos.Right(setpointFrame),
            Y = configY,
            Width = Dim.Absolute(18),
            Height = Dim.Absolute(4)
        };

        _pidPLabel = new Label
        {
            Text = "1.000",
            X = 1,
            Y = 0,
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };

        _pidPInput = new TextField()
        {
            Text = "1.000",
            X = 1,
            Y = 1,
            Width = 10,
            TextAlignment = Alignment.End
        };

        _pidPInput.KeyDown += (source, args) =>
        {
            if (args.KeyCode == KeyCode.Enter)
            {
                var text = _pidPInput.Text.ToString() ?? string.Empty;
                if (double.TryParse(text, out var value))
                {
                    _pidPLabel.Text = value.ToString("0.000");
                    LogViewer.AddMessage($"[CFG] PID P updated to {value:0.000}");
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid PID P: '{text}'");
                }
            }
        };

        pidPFrame.Add(_pidPLabel);
        pidPFrame.Add(_pidPInput);
        Add(pidPFrame);

        // PID I
        var pidIFrame = new FrameView
        {
            Title = "PID I",
            X = Pos.Right(pidPFrame),
            Y = configY,
            Width = Dim.Absolute(18),
            Height = Dim.Absolute(4)
        };

        _pidILabel = new Label
        {
            Text = "0.000",
            X = 1,
            Y = 0,
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };

        _pidIInput = new TextField()
        {
            Text = "0.000",
            X = 1,
            Y = 1,
            Width = 10,
            TextAlignment = Alignment.End
        };

        _pidIInput.KeyDown += (source, args) =>
        {
            if (args.KeyCode == KeyCode.Enter)
            {
                var text = _pidIInput.Text.ToString() ?? string.Empty;
                if (double.TryParse(text, out var value))
                {
                    _pidILabel.Text = value.ToString("0.000");
                    LogViewer.AddMessage($"[CFG] PID I updated to {value:0.000}");
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid PID I: '{text}'");
                }
            }
        };

        pidIFrame.Add(_pidILabel);
        pidIFrame.Add(_pidIInput);
        Add(pidIFrame);

        // PID D
        var pidDFrame = new FrameView
        {
            Title = "PID D",
            X = Pos.Right(pidIFrame),
            Y = configY,
            Width = Dim.Absolute(18),
            Height = Dim.Absolute(4)
        };

        _pidDLabel = new Label
        {
            Text = "0.000",
            X = 1,
            Y = 0,
            Width = Dim.Fill(),
            TextAlignment = Alignment.End
        };

        _pidDInput = new TextField()
        {
            Text = "0.000",
            X = 1,
            Y = 1,
            Width = 10,
            TextAlignment = Alignment.End
        };

        _pidDInput.KeyDown += (source, args) =>
        {
            if (args.KeyCode == KeyCode.Enter)
            {
                var text = _pidDInput.Text.ToString() ?? string.Empty;
                if (double.TryParse(text, out var value))
                {
                    _pidDLabel.Text = value.ToString("0.000");
                    LogViewer.AddMessage($"[CFG] PID D updated to {value:0.000}");
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid PID D: '{text}'");
                }
            }
        };

        pidDFrame.Add(_pidDLabel);
        pidDFrame.Add(_pidDInput);
        Add(pidDFrame);

        // Log view in the bottom portion
        InitializeLogView();

        _telemetry = new TelemetryState();

        var hubUrl = "ws://192.168.1.88:81/";

        _signalR = new SignalRListenerService(
            hubUrl,
            message => LogViewer.AddMessage(message),
            _telemetry,
            snapshot => Application.Invoke(() => UpdateTelemetryView(snapshot)));

        _ = _signalR.StartAsync();
    }

    private void UpdateTelemetryView(TelemetrySnapshot snapshot)
    {
        // Format values; null-safe with "—" fallback
        static string F(double? v) => v.HasValue ? v.Value.ToString("0.000") : "—";

        _rollValueLabel.Text = F(snapshot.Roll);
        _pitchValueLabel.Text = F(snapshot.Pitch);
        _yawValueLabel.Text = F(snapshot.Yaw);
        _c1ValueLabel.Text = F(snapshot.Custom1);
        _c2ValueLabel.Text = F(snapshot.Custom2);
    }

    private void InitializeMenu()
    {
        MenuBarV2 = new MenuBar(
        [
            new MenuBarItem("_File",
            [
                new MenuItem("_New", "Create a new file", () => { }),
                new MenuItem("_Open", "Open a file", () => { }),
                new MenuItem("_Save", "Save the file", () => { }),
                new MenuItem("_Quit", "Quit the application", () => Application.RequestStop())
            ]),
            new MenuBarItem("_Edit",
            [
                new MenuItem("_Cut", "Cut selection", () => { }),
                new MenuItem("_Copy", "Copy selection", () => { }),
                new MenuItem("_Paste", "Paste clipboard", () => { })
            ]),
            new MenuBarItem("_Help",
            [
                new MenuItem("_About", "About this app", () =>
                {
                    MessageBox.Query(App, "Terminal.Gui v2 Dashboard Example", "OK");
                })
            ])
        ]);

        Add(MenuBarV2);
    }

    private void InitializeLogView()
    {
        LogViewer = new LogView
        {
            X = 0,
            Y = Pos.AnchorEnd(10),
            Width = Dim.Fill(),
            Height = 10,
            CanFocus = false,
            BorderStyle = LineStyle.Single,
            Title = "Log"
        };

        Add(LogViewer);
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            if (_signalR is not null)
            {
                _ = _signalR.DisposeAsync();
            }
        }

        base.Dispose(disposing);
    }
}