using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace signalR_sender;

internal static class Program
{
    public static async Task Main(string[] args)
    {
        var uriString = args.Length > 0 ? args[0] : "ws://192.168.1.88:81/";

        if (!Uri.TryCreate(uriString, UriKind.Absolute, out var uri) ||
            (uri.Scheme != "ws" && uri.Scheme != "wss"))
        {
            Console.Error.WriteLine($"Invalid websocket URI: {uriString}");
            return;
        }

        using var shutdownCts = new CancellationTokenSource();
        Console.CancelKeyPress += (_, e) =>
        {
            e.Cancel = true;
            shutdownCts.Cancel();
        };

        using var ws = new ClientWebSocket();
        ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(20);

        using var connectTimeout = CancellationTokenSource.CreateLinkedTokenSource(shutdownCts.Token);
        connectTimeout.CancelAfter(TimeSpan.FromSeconds(15));

        Console.WriteLine($"Connecting: {uri}");
        await ws.ConnectAsync(uri, connectTimeout.Token);
        Console.WriteLine("Connected. Type a line and press Enter to send. Ctrl+C to quit.");

        while (!shutdownCts.IsCancellationRequested && ws.State == WebSocketState.Open)
        {
            var line = Console.ReadLine();
            if (line is null)
                break;

            var bytes = Encoding.UTF8.GetBytes(line);
            await ws.SendAsync(bytes, WebSocketMessageType.Text, true, shutdownCts.Token);
        }

        if (ws.State == WebSocketState.Open)
        {
            try
            {
                await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "Client done", CancellationToken.None);
            }
            catch
            {
            }
        }
    }
}
