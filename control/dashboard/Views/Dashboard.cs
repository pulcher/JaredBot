using System;
using Terminal.Gui.App;
using Terminal.Gui.ViewBase;
using Terminal.Gui.Views;
using Terminal.Gui.Drawing;
using System.Globalization;
using dashboard;
using dashboard.Models;
using dashboard.Services;
using Terminal.Gui.Drivers;
using Terminal.Gui.Input;

namespace dashboard.Views;

public sealed class Dashboard : Window
{
    public MenuBar MenuBarV2 { get; private set; }
    public LogView LogViewer { get; private set; }

    private readonly TelemetryState _telemetry;
    private readonly SignalRListenerService _signalR;

    private bool _didSyncInputsFromStatus;
    private ControlConfigSnapshot? _lastStatus;

    private Label _logStatusLabel;

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

    private readonly Label _lastStatusValueLabel;
    private readonly Label _lastCommandValueLabel;

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
                    _ = SendConfigCommandFromInputsAsync();
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid setpoint: '{text}'");
                }
            }
        };

        _setpointInput.Accepting += (s, e) =>
        {
            _ = SendConfigCommandFromInputsAsync();
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
            TextAlignment = Alignment.End,
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
                    _ = SendConfigCommandFromInputsAsync();
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid PID P: '{text}'");
                }
            }
        };

        _pidPInput.Accepting += (s, e) =>
        {
            _ = SendConfigCommandFromInputsAsync();
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
            TextAlignment = Alignment.End,
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
                    _ = SendConfigCommandFromInputsAsync();
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid PID I: '{text}'");
                }
            }
        };

        _pidIInput.Accepting += (s, e) =>
        {
            _ = SendConfigCommandFromInputsAsync();
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
            TextAlignment = Alignment.End,
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
                    _ = SendConfigCommandFromInputsAsync();
                }
                else
                {
                    LogViewer.AddMessage($"[CFG] Invalid PID D: '{text}'");
                }
            }
        };

        _pidDInput.Accepting += (s, e) =>
        {
            _ = SendConfigCommandFromInputsAsync();
        };

        pidDFrame.Add(_pidDLabel);
        pidDFrame.Add(_pidDInput);
        Add(pidDFrame);

        var statusFrame = new FrameView
        {
            Title = "Last Status (S:)",
            X = 0,
            Y = configY + 4,
            Width = Dim.Percent(50),
            Height = Dim.Absolute(3)
        };

        _lastStatusValueLabel = new Label
        {
            Text = "—",
            X = 0,
            Y = 0,
            Width = Dim.Fill(),
            Height = Dim.Fill(),
            TextAlignment = Alignment.Start
        };

        statusFrame.Add(_lastStatusValueLabel);
        Add(statusFrame);

        var commandFrame = new FrameView
        {
            Title = "Last Command (C:)",
            X = Pos.Right(statusFrame),
            Y = statusFrame.Y,
            Width = Dim.Fill(),
            Height = statusFrame.Height
        };

        _lastCommandValueLabel = new Label
        {
            Text = "—",
            X = 0,
            Y = 0,
            Width = Dim.Fill(),
            Height = Dim.Fill(),
            TextAlignment = Alignment.Start
        };

        commandFrame.Add(_lastCommandValueLabel);
        Add(commandFrame);

        // Log view in the bottom portion
        InitializeLogView();

        _telemetry = new TelemetryState();

        var hubUrl = "ws://192.168.1.73:81/";

        _signalR = new SignalRListenerService(
            hubUrl,
            message => Application.Invoke(() =>
            {
                LogViewer.AddMessage(message);
                UpdateLogStatusLabel();
            }),
            _telemetry,
            snapshot => Application.Invoke(() => UpdateTelemetryView(snapshot)),
            cfg => Application.Invoke(() => UpdateConfigView(cfg)));

        var inputs = new View[] { _setpointInput, _pidPInput, _pidIInput, _pidDInput };
        foreach (var v in inputs)
        {
            v.KeyDown += (_, e) =>
            {
                if (e.KeyCode == KeyCode.Tab)
                {
                    FocusNextInput(inputs, v, forward: true);
                    e.Handled = true;
                }
                else if (e.KeyCode == (KeyCode.Tab | KeyCode.ShiftMask))
                {
                    FocusNextInput(inputs, v, forward: false);
                    e.Handled = true;
                }
            };
        }

        _ = Task.Run(() => _signalR.StartAsync());
    }

    private void FocusNextInput(View[] inputs, View current, bool forward)
    {
        if (inputs.Length == 0)
            return;

        var idx = Array.IndexOf(inputs, current);
        if (idx < 0)
            idx = 0;

        var next = forward ? (idx + 1) % inputs.Length : (idx - 1 + inputs.Length) % inputs.Length;
        inputs[next].SetFocus();
    }

    private async Task SendConfigCommandFromInputsAsync()
    {
        static bool TryParseInvariant(string s, out float v)
        {
            return float.TryParse(
                s,
                System.Globalization.NumberStyles.Any,
                CultureInfo.InvariantCulture,
                out v);
        }

        var spText = _setpointInput.Text.ToString() ?? string.Empty;
        var kpText = _pidPInput.Text.ToString() ?? string.Empty;
        var kiText = _pidIInput.Text.ToString() ?? string.Empty;
        var kdText = _pidDInput.Text.ToString() ?? string.Empty;

        if (!TryParseInvariant(spText, out var sp) ||
            !TryParseInvariant(kpText, out var kp) ||
            !TryParseInvariant(kiText, out var ki) ||
            !TryParseInvariant(kdText, out var kd))
        {
            return;
        }

        // Prefer full SET if we've observed the extra KF values from the last S: status.
        if (_lastStatus?.QAngle is double qa &&
            _lastStatus?.QGyro is double qg &&
            _lastStatus?.RAngle is double ra &&
            _lastStatus?.K1 is double k1)
        {
            var cmd = JaredBotBalanceCommands.Set(
                sp,
                kp,
                ki,
                kd,
                (float)qa,
                (float)qg,
                (float)ra,
                (float)k1);

            LogViewer.AddMessage($"[CFG] -> {cmd.TrimEnd('\n')}");
            _lastCommandValueLabel.Text = cmd.TrimEnd('\n');
            await _signalR.SendCommandAsync(cmd);
            return;
        }

        // Fallback: send SP + PID as two commands.
        var spCmd = JaredBotBalanceCommands.Sp(sp);
        await _signalR.SendCommandAsync(spCmd);
        var pidCmd = JaredBotBalanceCommands.Pid(kp, ki, kd);
        LogViewer.AddMessage($"[CFG] -> {pidCmd.TrimEnd('\n')}");
        _lastCommandValueLabel.Text = pidCmd.TrimEnd('\n');
        await _signalR.SendCommandAsync(pidCmd);
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

    private void UpdateConfigView(ControlConfigSnapshot cfg)
    {
        static string F4(double? v) => v.HasValue ? v.Value.ToString("0.0000") : "—";
        static string F3(double? v) => v.HasValue ? v.Value.ToString("0.000") : "—";

        _setpointLabel.Text = F4(cfg.Setpoint);
        _pidPLabel.Text = F3(cfg.Kp);
        _pidILabel.Text = F3(cfg.Ki);
        _pidDLabel.Text = F3(cfg.Kd);

        _lastStatusValueLabel.Text = cfg.RawLine ?? "S:<missing>";
        _lastStatus = cfg;

        if (!_didSyncInputsFromStatus)
        {
            if (cfg.Setpoint.HasValue) _setpointInput.Text = F4(cfg.Setpoint);
            if (cfg.Kp.HasValue) _pidPInput.Text = F3(cfg.Kp);
            if (cfg.Ki.HasValue) _pidIInput.Text = F3(cfg.Ki);
            if (cfg.Kd.HasValue) _pidDInput.Text = F3(cfg.Kd);

            _didSyncInputsFromStatus = true;
        }
    }

    private void RequestReconnect()
    {
        ResetPanelsForReconnect();
        LogViewer.AddMessage("[UI] Reconnect requested.");
        _ = _signalR.ReconnectAsync();
    }

    private void ResetPanelsForReconnect()
    {
        _didSyncInputsFromStatus = false;
        _lastStatus = null;

        _rollValueLabel.Text = "—";
        _pitchValueLabel.Text = "—";
        _yawValueLabel.Text = "—";
        _c1ValueLabel.Text = "—";
        _c2ValueLabel.Text = "—";

        _setpointLabel.Text = "—";
        _pidPLabel.Text = "—";
        _pidILabel.Text = "—";
        _pidDLabel.Text = "—";

        _lastStatusValueLabel.Text = "—";
        _lastCommandValueLabel.Text = "—";
    }

    private void InitializeMenu()
    {
        MenuBarV2 = new MenuBar(
        [
            new MenuBarItem("_File",
            [
                new MenuItem("_Quit", "Quit the application", () => Application.RequestStop(), new Key(KeyCode.Q).WithCtrl)
            ]),
            new MenuBarItem("_Reconnect",
            [
                new MenuItem("_Reconnect", "Reconnect websocket", () => RequestReconnect(), new Key(KeyCode.E).WithCtrl)
            ]),
            new MenuBarItem("_Log",
            [
                new MenuItem("_Toggle Log", "Show/hide log panel", () => ToggleLogPanel(), new Key(KeyCode.L).WithCtrl)
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
            Title = "Log",
            Visible = false
        };

        LogViewer.VisibilityChanged += UpdateLogStatusLabel;

        _logStatusLabel = new Label
        {
            Text = "Log: 0 msgs (Ctrl+L to show)",
            X = 0,
            Y = Pos.AnchorEnd(1),
            Width = Dim.Fill(),
            Height = 1,
            TextAlignment = Alignment.Start
        };

        Add(LogViewer);
        Add(_logStatusLabel);
    }

    private void UpdateLogStatusLabel()
    {
        var count = LogViewer.MessageCount;
        if (LogViewer.LogVisible)
        {
            _logStatusLabel.Text = $"Log: {count} msgs (Ctrl+L to hide)";
        }
        else
        {
            _logStatusLabel.Text = $"Log: {count} msgs (Ctrl+L to show)";
        }
    }

    private void ToggleLogPanel()
    {
        LogViewer.ToggleVisibility();
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            if (_signalR is not null)
            {
                _signalR.DisposeAsync().AsTask().GetAwaiter().GetResult();
            }

            LogViewer.DisposeAsync().AsTask().GetAwaiter().GetResult();
        }

        base.Dispose(disposing);
    }
}