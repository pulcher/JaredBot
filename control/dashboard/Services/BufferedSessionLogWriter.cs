using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace dashboard.Services;

public sealed class BufferedSessionLogWriter : IAsyncDisposable
{
    private readonly string _filePath;
    private readonly object _lock = new();
    private readonly List<string> _pendingLines = new();
    private readonly CancellationTokenSource _cts = new();
    private readonly Task _flushLoopTask;
    private bool _disposed;

    public BufferedSessionLogWriter(string? logsDirectory = null, TimeSpan? flushInterval = null)
    {
        logsDirectory ??= Path.Combine(AppContext.BaseDirectory, "logs");
        Directory.CreateDirectory(logsDirectory);

        _filePath = Path.Combine(logsDirectory, $"dashboard-{DateTime.Now:yyyyMMdd-HHmmss}.log");
        var interval = flushInterval ?? TimeSpan.FromSeconds(5);

        _flushLoopTask = Task.Run(() => FlushLoopAsync(interval));
    }

    public void Enqueue(string message)
    {
        if (_disposed)
            return;

        var line = $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}: {message}";

        lock (_lock)
        {
            _pendingLines.Add(line);
        }
    }

    private async Task FlushLoopAsync(TimeSpan interval)
    {
        try
        {
            using var timer = new PeriodicTimer(interval);

            while (await timer.WaitForNextTickAsync(_cts.Token).ConfigureAwait(false))
            {
                await FlushPendingAsync(_cts.Token).ConfigureAwait(false);
            }
        }
        catch (OperationCanceledException) when (_cts.IsCancellationRequested)
        {
        }
    }

    private async Task FlushPendingAsync(CancellationToken cancellationToken = default)
    {
        string[] lines;

        lock (_lock)
        {
            if (_pendingLines.Count == 0)
                return;

            lines = [.. _pendingLines];
            _pendingLines.Clear();
        }

        await File.AppendAllLinesAsync(_filePath, lines, cancellationToken).ConfigureAwait(false);
    }

    public async ValueTask DisposeAsync()
    {
        if (_disposed)
            return;

        _disposed = true;

        try
        {
            _cts.Cancel();
            await _flushLoopTask.ConfigureAwait(false);
        }
        catch (OperationCanceledException) when (_cts.IsCancellationRequested)
        {
        }
        finally
        {
            _cts.Dispose();
        }

        await FlushPendingAsync().ConfigureAwait(false);
    }
}
