using System;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using dashboard.Models;

namespace dashboard.Services;

public sealed class SignalRListenerService : IAsyncDisposable
{
    private readonly Uri _uri;
    private readonly Action<string> _onMessage;
    private readonly TelemetryState? _telemetry;
    private readonly Action<TelemetrySnapshot>? _onTelemetry;
    private readonly Action<ControlConfigSnapshot>? _onConfig;
    private readonly CancellationTokenSource _cts = new();

    private readonly object _wsLock = new();
    private ClientWebSocket? _ws;

    private readonly object _reconnectLock = new();
    private CancellationTokenSource _reconnectCts = new();

    private const int ReconnectDelayMs = 500;

    public SignalRListenerService(
        string hubUrl,
        Action<string> onMessage,
        TelemetryState? telemetry = null,
        Action<TelemetrySnapshot>? onTelemetry = null,
        Action<ControlConfigSnapshot>? onConfig = null)
    {
        if (!Uri.TryCreate(hubUrl, UriKind.Absolute, out var uri) ||
            (uri.Scheme != "ws" && uri.Scheme != "wss"))
        {
            throw new ArgumentException($"Invalid websocket URI: {hubUrl}", nameof(hubUrl));
        }

        _uri = uri;
        _onMessage = onMessage ?? throw new ArgumentNullException(nameof(onMessage));
        _telemetry = telemetry;
        _onTelemetry = onTelemetry;
        _onConfig = onConfig;
    }

    public async Task StartAsync()
    {
        _onMessage($"[WS] Connecting to {_uri} (press 'Q' in dashboard to quit).");

        while (!_cts.IsCancellationRequested)
        {
            CancellationTokenSource iterationCts;
            lock (_reconnectLock)
            {
                iterationCts = CancellationTokenSource.CreateLinkedTokenSource(_cts.Token, _reconnectCts.Token);
            }

            var connected = false;

            try
            {
                connected = await ConnectAndReceiveLoopAsync(_uri, iterationCts.Token).ConfigureAwait(false);
                if (connected)
                {
                    // We had a successful session (connect + at least one receive or clean close)
                }
            }
            catch (OperationCanceledException) when (_cts.IsCancellationRequested)
            {
                iterationCts.Dispose();
                break;
            }
            catch (OperationCanceledException)
            {
                // Reconnect was requested — skip the delay and retry immediately.
                _onMessage("[WS] Reconnect signal received, retrying now...");
                iterationCts.Dispose();
                continue;
            }
            catch (WebSocketException wex) when (wex.WebSocketErrorCode == WebSocketError.ConnectionClosedPrematurely)
            {
                _onMessage("[WS] Connection closed prematurely by remote. Will retry.");
                _onMessage($"[WS] WebSocketException: ErrorCode={wex.ErrorCode}, WebSocketErrorCode={wex.WebSocketErrorCode}");
                await WaitBeforeReconnectAsync(iterationCts.Token).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                _onMessage($"[WS] Loop failed: {ex.Message}");

                if (ex is WebSocketException wex)
                    _onMessage($"[WS] WebSocketException: ErrorCode={wex.ErrorCode}, WebSocketErrorCode={wex.WebSocketErrorCode}");

                if (ex.InnerException is SocketException sex)
                    _onMessage($"[WS] SocketException: Code={sex.SocketErrorCode}, Message={sex.Message}");

                await WaitBeforeReconnectAsync(iterationCts.Token).ConfigureAwait(false);
            }
            finally
            {
                iterationCts.Dispose();
            }
        }

        _onMessage("[WS] Listener stopped.");
    }

    private async Task WaitBeforeReconnectAsync(CancellationToken cancellation)
    {
        _onMessage($"[WS] Reconnecting in {ReconnectDelayMs}ms...");

        try
        {
            await Task.Delay(ReconnectDelayMs, cancellation).ConfigureAwait(false);
        }
        catch (OperationCanceledException)
        {
            return;
        }
    }

    private async Task<bool> ConnectAndReceiveLoopAsync(Uri uri, CancellationToken cancellation)
    {
        using var ws = new ClientWebSocket();
        ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(0);

        lock (_wsLock)
        {
            _ws = ws;
        }

        using var connectTimeout = CancellationTokenSource.CreateLinkedTokenSource(cancellation);
        connectTimeout.CancelAfter(TimeSpan.FromSeconds(30));

        await ws.ConnectAsync(uri, connectTimeout.Token).ConfigureAwait(false);
        _onMessage("[WS] Connected. Receiving frames until shutdown or remote close...");

        var buffer = new byte[16 * 1024];
        var gotAnyData = false;

        try
        {
            while (!cancellation.IsCancellationRequested && ws.State == WebSocketState.Open)
            {
                using var ms = new System.IO.MemoryStream();
                WebSocketReceiveResult result;

                try
                {
                    do
                    {
                        result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), cancellation).ConfigureAwait(false);

                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            _onMessage($"[WS] Server sent close: {result.CloseStatus} - {result.CloseStatusDescription}");
                            using var closeAckTimeout = new CancellationTokenSource(TimeSpan.FromSeconds(5));
                            await ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "Client ack", closeAckTimeout.Token).ConfigureAwait(false);
                            return gotAnyData;
                        }

                        if (result.Count > 0)
                            gotAnyData = true;

                        ms.Write(buffer, 0, result.Count);
                    }
                    while (!result.EndOfMessage);
                }
                catch (WebSocketException wsex)
                {
                    _onMessage($"[WS] Receive failed: {wsex.Message}");
                    _onMessage($"[WS] WebSocketState={ws.State}, ErrorCode={wsex.ErrorCode}, WebSocketErrorCode={wsex.WebSocketErrorCode}");
                    break;
                }
                catch (OperationCanceledException) when (cancellation.IsCancellationRequested)
                {
                    break;
                }

                await ProcessFrameAsync(result.MessageType, ms.ToArray()).ConfigureAwait(false);
            }
        }
        finally
        {
            lock (_wsLock)
            {
                if (ReferenceEquals(_ws, ws))
                    _ws = null;
            }

            if (ws.State == WebSocketState.Open)
            {
                try
                {
                    using var closeTimeout = new CancellationTokenSource(TimeSpan.FromSeconds(5));
                    await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client done", closeTimeout.Token).ConfigureAwait(false);
                }
                catch { }
            }
        }

        return gotAnyData;
    }

    public Task SendCommandAsync(string commandLine)
    {
        if (string.IsNullOrWhiteSpace(commandLine))
            return Task.CompletedTask;

        ClientWebSocket? ws;
        lock (_wsLock)
        {
            ws = _ws;
        }

        if (ws is null || ws.State != WebSocketState.Open)
        {
            _onMessage("[WS] Cannot send command: not connected.");
            return Task.CompletedTask;
        }

        var bytes = Encoding.UTF8.GetBytes(commandLine);
        return ws.SendAsync(bytes, WebSocketMessageType.Text, true, _cts.Token);
    }

    public void SendCommand(string commandLine)
    {
        _ = SendCommandAsync(commandLine);
    }

    public async Task ReconnectAsync()
    {
     _onMessage("[WS] Reconnect requested.");

        // Signal the current iteration to cancel (interrupts delay or ConnectAsync).
        lock (_reconnectLock)
        {
            _reconnectCts.Cancel();
            _reconnectCts.Dispose();
            _reconnectCts = new CancellationTokenSource();
        }

        // Also abort the current websocket if one exists.
        ClientWebSocket? ws;
        lock (_wsLock)
        {
            ws = _ws;
        }

        if (ws is null)
            return;

        try
        {
            ws.Abort();
        }
        catch
        {
        }
    }

    public void Reconnect()
    {
        _ = ReconnectAsync();
    }

    private Task ProcessFrameAsync(WebSocketMessageType type, byte[] payload)
    {
        if (type == WebSocketMessageType.Text)
        {
            var text = SafeUtf8(payload);
            _onMessage(text);

            if (text.Equals("A:OK", StringComparison.OrdinalIgnoreCase) ||
                text.Equals("A:ERR", StringComparison.OrdinalIgnoreCase))
            {
                return Task.CompletedTask;
            }

            if (text.StartsWith("T:", StringComparison.OrdinalIgnoreCase))
            {
                var snapshot = ParseTelemetryCsv(text);
                if (snapshot is not null)
                {
                    _telemetry?.Update(snapshot);
                    _onTelemetry?.Invoke(snapshot);
                }

                return Task.CompletedTask;
            }

            if (text.StartsWith("S:", StringComparison.OrdinalIgnoreCase))
            {
                var cfg = ParseConfigCsv(text);
                if (cfg is not null)
                    _onConfig?.Invoke(cfg);

                return Task.CompletedTask;
            }

            var pairs = ParseKeyValuePairs(text);
            if (pairs.Count > 0)
            {
                foreach (var kv in pairs)
                    _onMessage($"{kv.Key} = {kv.Value}");
            }

            return Task.CompletedTask;
        }

        var previewLen = Math.Min(payload.Length, 128);
        var hex = Convert.ToHexString(payload, 0, previewLen);
        var preview = payload.Length <= previewLen ? hex : hex + "...";
        _onMessage($"Binary preview (hex): {preview}");

        return Task.CompletedTask;
    }

    private static TelemetrySnapshot? ParseTelemetryCsv(string line)
    {
        var span = line.AsSpan().Trim();
        if (!span.StartsWith("T:", StringComparison.OrdinalIgnoreCase))
            return null;

        span = span[2..].TrimStart();
        var parts = span.ToString().Split(',', StringSplitOptions.TrimEntries);
        if (parts.Length < 3)
            return null;

        static double? TryParseAt(string[] p, int index)
        {
            if (index < 0 || index >= p.Length)
                return null;

            if (double.TryParse(p[index], System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var v))
                return v;

            return null;
        }

        return new TelemetrySnapshot
        {
            UtcTimestamp = DateTime.UtcNow,
            RawLine = line,
            Roll = TryParseAt(parts, 0),
            Pitch = TryParseAt(parts, 1),
            Yaw = TryParseAt(parts, 2),
            Custom1 = TryParseAt(parts, 3),
            Custom2 = TryParseAt(parts, 4)
        };
    }

    private static ControlConfigSnapshot? ParseConfigCsv(string line)
    {
        var span = line.AsSpan().Trim();
        if (!span.StartsWith("S:", StringComparison.OrdinalIgnoreCase))
            return null;

        span = span[2..].TrimStart();
        var parts = span.ToString().Split(',', StringSplitOptions.TrimEntries);
        if (parts.Length < 4)
            return null;

        static double? TryParse(string s)
        {
            if (double.TryParse(s, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out var v))
                return v;

            return null;
        }

        return new ControlConfigSnapshot
        {
            UtcTimestamp = DateTime.UtcNow,
            RawLine = line,
            Setpoint = TryParse(parts[0]),
            Kp = TryParse(parts[1]),
            Ki = TryParse(parts[2]),
            Kd = TryParse(parts[3]),
            QAngle = parts.Length > 4 ? TryParse(parts[4]) : null,
            QGyro = parts.Length > 5 ? TryParse(parts[5]) : null,
            RAngle = parts.Length > 6 ? TryParse(parts[6]) : null,
            K1 = parts.Length > 7 ? TryParse(parts[7]) : null
        };
    }

    private static string SafeUtf8(byte[] bytes)
    {
        try { return Encoding.UTF8.GetString(bytes); }
        catch { return "<invalid-utf8>"; }
    }

    private static System.Collections.Generic.Dictionary<string, string> ParseKeyValuePairs(string input)
    {
        var dict = new System.Collections.Generic.Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        var pattern = new Regex(@"(?<k>[A-Za-z0-9_]+)\s*=\s*(?<v>[^\s]+)", RegexOptions.Compiled);

        foreach (Match m in pattern.Matches(input))
        {
            var k = m.Groups["k"].Value;
            var v = m.Groups["v"].Value;

            if (!dict.ContainsKey(k))
                dict[k] = v;
            else
                dict[k] = dict[k] + "," + v;
        }

        return dict;
    }

    public async ValueTask DisposeAsync()
    {
        try { _cts.Cancel(); } catch { }
        lock (_reconnectLock)
        {
            try { _reconnectCts.Cancel(); } catch { }
            _reconnectCts.Dispose();
        }
        _cts.Dispose();
        await Task.CompletedTask;
    }
}
