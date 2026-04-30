using System;
using System.Collections.Generic;
using System.Drawing;
using System.Threading.Tasks;
using dashboard.Services;
using Terminal.Gui.ViewBase;

namespace dashboard.Views;

public class LogView : View
{
    private readonly object _lock = new();
    private readonly List<string> _logMessages = new();
    private readonly BufferedSessionLogWriter _sessionLogWriter = new();
    private int _messageCountAtLastDraw;

    /// <summary>When false, messages are buffered but no redraw is requested.</summary>
    public bool LogVisible { get; private set; }

    public int MessageCount
    {
        get { lock (_lock) { return _logMessages.Count; } }
    }

    /// <summary>Raised after visibility changes so the host can adjust layout.</summary>
    public event Action? VisibilityChanged;

    public void AddMessage(string message)
    {
        _sessionLogWriter.Enqueue(message);

        lock (_lock)
        {
            _logMessages.Add($"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}: {message}");
        }

        if (LogVisible)
        {
            SetNeedsLayout();
            SetNeedsDraw();
        }
    }

    public void ToggleVisibility()
    {
        LogVisible = !LogVisible;
        Visible = LogVisible;
        if (LogVisible)
        {
            SetNeedsLayout();
            SetNeedsDraw();
        }
        VisibilityChanged?.Invoke();
    }

    public void Show()
    {
        if (!LogVisible)
            ToggleVisibility();
    }

    public void Hide()
    {
        if (LogVisible)
            ToggleVisibility();
    }

    public ValueTask DisposeAsync()
    {
        return _sessionLogWriter.DisposeAsync();
    }

    protected override bool OnDrawingSubViews()
    {
        base.OnDrawingSubViews();

        var bounds = Viewport;
        int width = bounds.Width;
        int height = bounds.Height;

        string[] snapshot;
        lock (_lock)
        {
            int startIndex = Math.Max(0, _logMessages.Count - height);
            int count = Math.Min(height, _logMessages.Count - startIndex);
            snapshot = new string[count];
            _logMessages.CopyTo(startIndex, snapshot, 0, count);
            _messageCountAtLastDraw = _logMessages.Count;
        }

        for (int line = 0; line < snapshot.Length; line++)
        {
            var text = snapshot[line];

            if (text.Length > width)
            {
                text = text[..width];
            }

            Move(0, line);
            AddStr(text);
        }

        return true;
    }
}
