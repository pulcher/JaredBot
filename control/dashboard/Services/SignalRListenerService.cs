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

    // Backoff settings (same as playground)
    private const int MinBackoffMs = 500;
    private const int MaxBackoffMs = 30_000;
    private const double BackoffFactor = 2.0;

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

        var backoffMs = MinBackoffMs;

        while (!_cts.IsCancellationRequested)
        {
            var connected = false;

            try
            {
                connected = await ConnectAndReceiveLoopAsync(_uri, _cts.Token);
                if (connected)
                {
                    // We had a successful session (connect + at least one receive or clean close)
                    backoffMs = MinBackoffMs;
                }
            }
            catch (OperationCanceledException) when (_cts.IsCancellationRequested)
            {
                break;
            }
            catch (WebSocketException wex) when (wex.WebSocketErrorCode == WebSocketError.ConnectionClosedPrematurely)
            {
                _onMessage("[WS] Connection closed prematurely by remote. Will retry.");
                _onMessage($"[WS] WebSocketException: ErrorCode={wex.ErrorCode}, WebSocketErrorCode={wex.WebSocketErrorCode}");
                backoffMs = await WaitWithBackoffAsync(backoffMs);
            }
            catch (Exception ex)
            {
                _onMessage($"[WS] Loop failed: {ex.Message}");

                if (ex is WebSocketException wex)
                    _onMessage($"[WS] WebSocketException: ErrorCode={wex.ErrorCode}, WebSocketErrorCode={wex.WebSocketErrorCode}");

                if (ex.InnerException is SocketException sex)
                    _onMessage($"[WS] SocketException: Code={sex.SocketErrorCode}, Message={sex.Message}");

                backoffMs = await WaitWithBackoffAsync(backoffMs);
            }
        }

        _onMessage("[WS] Listener stopped.");
    }

    private async Task<int> WaitWithBackoffAsync(int backoffMs)
    {
        _onMessage($"[WS] Reconnecting in {backoffMs}ms...");

        try
        {
            await Task.Delay(backoffMs, _cts.Token);
        }
        catch (OperationCanceledException)
        {
            return backoffMs;
        }

        return Math.Min(MaxBackoffMs, (int)(backoffMs * BackoffFactor));
    }

    private async Task<bool> ConnectAndReceiveLoopAsync(Uri uri, CancellationToken cancellation)
    {
        using var ws = new ClientWebSocket();
        ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(0);

        using var connectTimeout = CancellationTokenSource.CreateLinkedTokenSource(cancellation);
        connectTimeout.CancelAfter(TimeSpan.FromSeconds(30));

        await ws.ConnectAsync(uri, connectTimeout.Token);
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
                        result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), cancellation);

                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            _onMessage($"[WS] Server sent close: {result.CloseStatus} - {result.CloseStatusDescription}");
                            await ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "Client ack", CancellationToken.None);
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

                await ProcessFrameAsync(result.MessageType, ms.ToArray());
            }
        }
        finally
        {
            if (ws.State == WebSocketState.Open)
            {
                try { await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client done", CancellationToken.None); }
                catch { }
            }
        }

        return gotAnyData;
    }

    private Task ProcessFrameAsync(WebSocketMessageType type, byte[] payload)
    {
        if (type == WebSocketMessageType.Text)
        {
            var text = SafeUtf8(payload);
            _onMessage(text);

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
            Kd = TryParse(parts[3])
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
        _cts.Dispose();
        await Task.CompletedTask;
    }
}
