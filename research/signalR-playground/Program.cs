using System;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace signalR_playground
{
    internal static class Program
    {
        private const int MinBackoffMs = 500;
        private const int MaxBackoffMs = 30_000;
        private const double BackoffFactor = 2.0;

        public static async Task Main(string[] args)
        {
            var uriString = args.Length > 0 ? args[0] : "ws://192.168.1.88:81/";

            if (!Uri.TryCreate(uriString, UriKind.Absolute, out var uri) ||
                (uri.Scheme != "ws" && uri.Scheme != "wss"))
            {
                Console.WriteLine($"Invalid websocket URI: {uriString}");
                return;
            }

            Console.WriteLine($"Raw WebSocket client connecting to: {uriString}");
            Console.WriteLine("Press Ctrl+C to stop.");

            using var shutdownCts = new CancellationTokenSource();
            Console.CancelKeyPress += (_, e) =>
            {
                e.Cancel = true;
                Console.WriteLine("Shutdown requested...");
                shutdownCts.Cancel();
            };

            var backoffMs = MinBackoffMs;

            while (!shutdownCts.IsCancellationRequested)
            {
                var connectedAtLeastOnce = false;

                try
                {
                    await ConnectAndReceiveLoopAsync(uri, shutdownCts.Token);
                    // If we get here without exception and we had a successful connection,
                    // treat it as a healthy cycle and reset backoff.
                    if (connectedAtLeastOnce)
                    {
                        backoffMs = MinBackoffMs;
                    }
                }
                catch (OperationCanceledException) when (shutdownCts.IsCancellationRequested)
                {
                    break;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"WebSocket loop failed: {ex.Message}");

                    if (ex is WebSocketException wex)
                    {
                        Console.WriteLine(
                            $"WebSocketException: ErrorCode={wex.ErrorCode}, WebSocketErrorCode={wex.WebSocketErrorCode}");
                    }

                    if (ex.InnerException is SocketException sex)
                    {
                        Console.WriteLine(
                            $"SocketException: Code={sex.SocketErrorCode}, Message={sex.Message}");
                    }

                    Console.WriteLine($"Reconnecting in {backoffMs}ms...");
                    try
                    {
                        await Task.Delay(backoffMs, shutdownCts.Token);
                    }
                    catch (OperationCanceledException)
                    {
                        break;
                    }

                    backoffMs = Math.Min(MaxBackoffMs, (int)(backoffMs * BackoffFactor));
                }

                // Local function to capture whether we actually connected
                async Task ConnectAndReceiveLoopAsync(Uri innerUri, CancellationToken token)
                {
                    using var ws = new ClientWebSocket();
                    ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(20);

                    using var connectTimeout = CancellationTokenSource.CreateLinkedTokenSource(token);
                    connectTimeout.CancelAfter(TimeSpan.FromSeconds(15));

                    await ws.ConnectAsync(innerUri, connectTimeout.Token);
                    connectedAtLeastOnce = true;
                    if (connectedAtLeastOnce)
                    {
                        backoffMs = MinBackoffMs;
                    }

                    Console.WriteLine("Connected. Receiving frames until shutdown or remote close...");

                    var buffer = new byte[16 * 1024];

                    try
                    {
                        while (!token.IsCancellationRequested && ws.State == WebSocketState.Open)
                        {
                            using var ms = new System.IO.MemoryStream();
                            WebSocketReceiveResult result;

                            try
                            {
                                do
                                {
                                    result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), token);

                                    if (result.MessageType == WebSocketMessageType.Close)
                                    {
                                        Console.WriteLine(
                                            $"Server sent close: {result.CloseStatus} - {result.CloseStatusDescription}");
                                        await ws.CloseOutputAsync(
                                            WebSocketCloseStatus.NormalClosure,
                                            "Client ack",
                                            CancellationToken.None);
                                        return;
                                    }

                                    ms.Write(buffer, 0, result.Count);
                                } while (!result.EndOfMessage);
                            }
                            catch (WebSocketException wsex)
                            {
                                Console.WriteLine($"Receive failed: {wsex.Message}");
                                Console.WriteLine(
                                    $"WebSocketState={ws.State}, ErrorCode={wsex.ErrorCode}, WebSocketErrorCode={wsex.WebSocketErrorCode}");
                                throw;
                            }
                            catch (OperationCanceledException) when (token.IsCancellationRequested)
                            {
                                break;
                            }

                            var messageBytes = ms.ToArray();
                            await ProcessFrameAsync(result.MessageType, messageBytes);
                        }
                    }
                    finally
                    {
                        if (ws.State == WebSocketState.Open)
                        {
                            try
                            {
                                await ws.CloseAsync(
                                    WebSocketCloseStatus.NormalClosure,
                                    "Client done",
                                    CancellationToken.None);
                            }
                            catch
                            {
                                // ignore close errors
                            }
                        }
                    }
                }
            }

            Console.WriteLine("Client shutdown complete.");
        }

        private static Task ProcessFrameAsync(WebSocketMessageType type, byte[] payload)
        {
            var utc = DateTime.UtcNow;
            Console.WriteLine($"{utc:O} Frame -> type={type} length={payload.Length}");

            if (type == WebSocketMessageType.Text)
            {
                var text = SafeUtf8(payload);
                Console.WriteLine(text);

                var pairs = ParseKeyValuePairs(text);
                if (pairs.Count > 0)
                {
                    foreach (var kv in pairs)
                    {
                        Console.WriteLine($"{kv.Key} = {kv.Value}");
                    }
                }
            }
            else
            {
                var previewLen = Math.Min(payload.Length, 128);
                var hex = Convert.ToHexString(payload, 0, previewLen);
                var preview = payload.Length <= previewLen ? hex : hex + "...";
                Console.WriteLine($"Binary preview (hex): {preview}");
            }

            return Task.CompletedTask;
        }

        private static string SafeUtf8(byte[] bytes)
        {
            try
            {
                return Encoding.UTF8.GetString(bytes);
            }
            catch
            {
                return "<invalid-utf8>";
            }
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
                {
                    dict[k] = v;
                }
                else
                {
                    dict[k] = dict[k] + "," + v;
                }
            }

            return dict;
        }
    }
}
