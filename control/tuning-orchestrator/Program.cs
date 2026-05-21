using System.Globalization;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;

const string DefaultWsUrl = "ws://192.168.1.73:81";
const int DefaultTestSeconds = 15;

var wsUrl = args.Length > 0 ? args[0] : DefaultWsUrl;
if (!Uri.TryCreate(wsUrl, UriKind.Absolute, out var wsUri) ||
    (wsUri.Scheme != "ws" && wsUri.Scheme != "wss"))
{
    Console.WriteLine($"Invalid websocket URI: {wsUrl}");
    return;
}

var runUtc = DateTime.UtcNow;
var runId = runUtc.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture);
var runRoot = Path.Combine(AppContext.BaseDirectory, "runs", runId);
Directory.CreateDirectory(runRoot);

Console.WriteLine("JaredBot Tuning Orchestrator v1");
Console.WriteLine($"WS source: {wsUrl}");
Console.WriteLine($"Run folder: {runRoot}");
Console.WriteLine("Mode: suggest patch only; no automatic firmware edit/upload.");
Console.WriteLine();

var tests = TestCatalog.CreateDefault(DefaultTestSeconds);
var listener = new TelemetryListener(wsUri);

using var appCts = new CancellationTokenSource();
var listenTask = listener.RunAsync(appCts.Token);

await WaitForConnectionWarmupAsync(listener, TimeSpan.FromSeconds(3), appCts.Token);

for (var i = 0; i < tests.Count; i++)
{
    var test = tests[i];
    Console.WriteLine($"Upcoming test [{i + 1}/{tests.Count}]: {test.Id} - {test.Title}");
    Console.WriteLine($"Goal: {test.Goal}");
    Console.WriteLine($"Checklist: {test.OperatorChecklist}");
    Console.WriteLine($"Expected duration: {test.Duration.TotalSeconds:0}s");
    Console.Write("Type 'r' to run, 's' to skip, or 'q' to end session: ");

    var response = (Console.ReadLine() ?? string.Empty).Trim();
    if (response.Equals("q", StringComparison.OrdinalIgnoreCase) ||
        response.Equals("quit", StringComparison.OrdinalIgnoreCase))
    {
        Console.WriteLine("Session ended by operator.");
        break;
    }

    if (response.Equals("s", StringComparison.OrdinalIgnoreCase) ||
        response.Equals("skip", StringComparison.OrdinalIgnoreCase))
    {
        Console.WriteLine("Skipped.");
        Console.WriteLine();
        continue;
    }

    if (!(response.Equals("r", StringComparison.OrdinalIgnoreCase) ||
          response.Equals("ready", StringComparison.OrdinalIgnoreCase)))
    {
        Console.WriteLine("Input not recognized as run command. Skipping this test.");
        Console.WriteLine();
        continue;
    }

    Console.WriteLine("Starting test. Keep area safe. Press Ctrl+C to abort app.");
    var testRun = await ExecuteTestAsync(test, i + 1, runRoot, listener, appCts.Token);

    Console.WriteLine($"Completed: {test.Id}");
    Console.WriteLine($"Captured events: {testRun.RawEvents.Count}");
    Console.WriteLine($"Summary: {testRun.SummaryPath}");
    Console.WriteLine($"Copilot handoff: {testRun.HandoffPath}");
    Console.WriteLine();
}

appCts.Cancel();
try
{
    await listenTask;
}
catch (OperationCanceledException)
{
}

Console.WriteLine("Done.");

static async Task<TestRunResult> ExecuteTestAsync(
    TestDefinition test,
    int sequence,
    string runRoot,
    TelemetryListener listener,
    CancellationToken cancellationToken)
{
    var startedUtc = DateTime.UtcNow;
    var testFolder = Path.Combine(runRoot, $"{sequence:00}-{test.Id}");
    Directory.CreateDirectory(testFolder);

    var markerStart = TelemetryEvent.Marker("TEST_START", test.Id, $"{test.Title}|{test.Goal}", startedUtc);
    listener.AddMarker(markerStart);

    var sample = listener.BeginSample();
    var stalled = false;
    TelemetryEvent? stallWarn = null;
    var endAtUtc = startedUtc + test.Duration;
    var lastFrameCount = listener.TotalFrames;
    var lastFrameProgressUtc = DateTime.UtcNow;
    var stopCueShown = false;

    try
    {
        while (DateTime.UtcNow < endAtUtc)
        {
            await Task.Delay(TimeSpan.FromMilliseconds(250), cancellationToken);

            if (!stopCueShown &&
                test.Id.Equals("controlled_stop", StringComparison.OrdinalIgnoreCase) &&
                DateTime.UtcNow - startedUtc >= TimeSpan.FromSeconds(8))
            {
                stopCueShown = true;
                Console.WriteLine("[cue] Send STOP now (dashboard stop/disarm or serial command C:STOP / C:ARM,0).");
            }

            var currentFrameCount = listener.TotalFrames;
            if (currentFrameCount > lastFrameCount)
            {
                lastFrameCount = currentFrameCount;
                lastFrameProgressUtc = DateTime.UtcNow;
                continue;
            }

            if (DateTime.UtcNow - lastFrameProgressUtc >= TimeSpan.FromSeconds(2))
            {
                stalled = true;
                stallWarn = TelemetryEvent.Marker("TEST_WARN", test.Id, "telemetry_stall", DateTime.UtcNow);
                listener.AddMarker(stallWarn);
                Console.WriteLine($"[warn] telemetry stalled during {test.Id}; ending test early.");
                break;
            }
        }
    }
    finally
    {
        sample.Dispose();
    }

    var endedUtc = DateTime.UtcNow;
    var markerEnd = TelemetryEvent.Marker("TEST_END", test.Id, stalled ? "telemetry_stall" : "complete", endedUtc);
    listener.AddMarker(markerEnd);

    var events = sample.Events;
    events.Insert(0, markerStart);
    if (stalled && stallWarn is not null)
    {
        events.Add(stallWarn);
    }
    events.Add(markerEnd);

    var summary = TestSummary.FromEvents(test, startedUtc, endedUtc, events, listener.LastConfigSnapshot);

    var rawPath = Path.Combine(testFolder, "events.ndjson");
    await WriteNdjsonAsync(rawPath, events, cancellationToken);

    var summaryPath = Path.Combine(testFolder, "summary.json");
    await WriteJsonAsync(summaryPath, summary, cancellationToken);

    var handoffPath = Path.Combine(testFolder, "copilot-handoff.md");
    await File.WriteAllTextAsync(handoffPath, HandoffPromptBuilder.Build(test, summary, rawPath, summaryPath), cancellationToken);

    return new TestRunResult(test.Id, rawPath, summaryPath, handoffPath, events);
}

static async Task WriteNdjsonAsync(string path, IReadOnlyList<TelemetryEvent> events, CancellationToken cancellationToken)
{
    await using var stream = File.Create(path);
    await using var writer = new StreamWriter(stream, new UTF8Encoding(false));

    foreach (var evt in events)
    {
        var json = JsonSerializer.Serialize(evt);
        await writer.WriteLineAsync(json.AsMemory(), cancellationToken);
    }
}

static async Task WriteJsonAsync<T>(string path, T value, CancellationToken cancellationToken)
{
    await using var stream = File.Create(path);
    await JsonSerializer.SerializeAsync(stream, value, new JsonSerializerOptions
    {
        WriteIndented = true
    }, cancellationToken);
}

static async Task WaitForConnectionWarmupAsync(TelemetryListener listener, TimeSpan timeout, CancellationToken cancellationToken)
{
    var started = DateTime.UtcNow;
    while (DateTime.UtcNow - started < timeout && !cancellationToken.IsCancellationRequested)
    {
        if (listener.HasSeenAnyFrame)
        {
            Console.WriteLine("Telemetry stream detected.");
            Console.WriteLine();
            return;
        }

        await Task.Delay(200, cancellationToken);
    }

    Console.WriteLine("No telemetry frames seen yet. You can still run tests and capture markers.");
    Console.WriteLine();
}

internal sealed class TelemetryListener
{
    private readonly Uri _uri;
    private readonly object _sync = new();
    private readonly List<ActiveSample> _samples = new();

    private volatile bool _hasSeenAnyFrame;
    private ControlConfigSnapshot? _lastConfig;
    private long _totalFrames;
    private DateTime _lastFrameUtc = DateTime.MinValue;

    public TelemetryListener(Uri uri)
    {
        _uri = uri;
    }

    public bool HasSeenAnyFrame => _hasSeenAnyFrame;
    public long TotalFrames
    {
        get
        {
            lock (_sync)
            {
                return _totalFrames;
            }
        }
    }

    public DateTime LastFrameUtc
    {
        get
        {
            lock (_sync)
            {
                return _lastFrameUtc;
            }
        }
    }

    public ControlConfigSnapshot? LastConfigSnapshot
    {
        get
        {
            lock (_sync)
            {
                return _lastConfig;
            }
        }
    }

    public ActiveSample BeginSample()
    {
        lock (_sync)
        {
            var sample = new ActiveSample(this);
            _samples.Add(sample);
            return sample;
        }
    }

    public void AddMarker(TelemetryEvent marker)
    {
        lock (_sync)
        {
            foreach (var sample in _samples)
            {
                sample.Events.Add(marker);
            }
        }
    }

    public async Task RunAsync(CancellationToken cancellationToken)
    {
        var backoff = TimeSpan.FromMilliseconds(500);
        while (!cancellationToken.IsCancellationRequested)
        {
            try
            {
                await ConnectAndReceiveAsync(cancellationToken);
                backoff = TimeSpan.FromMilliseconds(500);
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[ws] receive loop failed: {ex.Message}");
                await Task.Delay(backoff, cancellationToken);
                var nextMs = Math.Min(backoff.TotalMilliseconds * 2.0, 30_000);
                backoff = TimeSpan.FromMilliseconds(nextMs);
            }
        }
    }

    private async Task ConnectAndReceiveAsync(CancellationToken cancellationToken)
    {
        using var ws = new ClientWebSocket();
        ws.Options.KeepAliveInterval = TimeSpan.FromSeconds(20);

        using var connectCts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        connectCts.CancelAfter(TimeSpan.FromSeconds(15));

        await ws.ConnectAsync(_uri, connectCts.Token);
        Console.WriteLine($"[ws] connected: {_uri}");

        var buffer = new byte[16 * 1024];
        while (!cancellationToken.IsCancellationRequested && ws.State == WebSocketState.Open)
        {
            using var ms = new MemoryStream();
            WebSocketReceiveResult result;

            do
            {
                result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), cancellationToken);
                if (result.MessageType == WebSocketMessageType.Close)
                {
                    await ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "ack", CancellationToken.None);
                    return;
                }

                ms.Write(buffer, 0, result.Count);
            }
            while (!result.EndOfMessage);

            if (result.MessageType != WebSocketMessageType.Text)
            {
                continue;
            }

            var line = Encoding.UTF8.GetString(ms.ToArray());
            _hasSeenAnyFrame = true;

            var evt = TelemetryEvent.FromRaw(line, DateTime.UtcNow);
            var config = ControlConfigSnapshot.TryParse(line);

            lock (_sync)
            {
                _totalFrames++;
                _lastFrameUtc = evt.UtcTimestamp;

                if (config is not null)
                {
                    _lastConfig = config;
                }

                foreach (var sample in _samples)
                {
                    sample.Events.Add(evt);
                }
            }
        }
    }

    private void StopSample(ActiveSample sample)
    {
        lock (_sync)
        {
            _samples.Remove(sample);
        }
    }

    internal sealed class ActiveSample : IDisposable
    {
        private readonly TelemetryListener _owner;
        private bool _disposed;

        internal ActiveSample(TelemetryListener owner)
        {
            _owner = owner;
        }

        public List<TelemetryEvent> Events { get; } = new();

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }

            _disposed = true;
            _owner.StopSample(this);
        }
    }
}

internal sealed record TestDefinition(
    string Id,
    string Title,
    string Goal,
    TimeSpan Duration,
    string OperatorChecklist);

internal static class TestCatalog
{
    public static List<TestDefinition> CreateDefault(int secondsPerTest)
    {
        var duration = TimeSpan.FromSeconds(secondsPerTest);
        return new List<TestDefinition>
        {
            new(
                "imu_settle",
                "IMU settle",
                "Observe sensor baseline noise and stability while bot remains untouched.",
                duration,
                "Hold bot still on stand; no motor arming."),
            new(
                "upright_hold",
                "Static upright hold",
                "Measure tilt settling and hold stability with minimal disturbance.",
                duration,
                "Arm in safe area; hands off during capture."),
            new(
                "push_recovery",
                "Small push recovery",
                "Measure response and oscillation after a small manual disturbance.",
                duration,
                "Apply one light push at ~5s mark."),
            new(
                "disturbance_recovery",
                "Disturbance recovery",
                "Evaluate larger disturbance handling and post-event settling.",
                duration,
                "Apply one moderate push at ~5s mark; be ready to catch."),
            new(
                "controlled_stop",
                "Controlled stop",
                "Validate safe stop behavior and post-stop telemetry consistency.",
                duration,
                "At ~8s, issue a stop/disarm (dashboard button or serial C:STOP / C:ARM,0), then let test finish.")
        };
    }
}

internal sealed record TelemetryEvent(
    DateTime UtcTimestamp,
    string Kind,
    string Raw,
    double[] NumericValues,
    string? Name,
    string? Detail)
{
    public static TelemetryEvent FromRaw(string raw, DateTime utc)
    {
        var line = (raw ?? string.Empty).Trim();
        var kind = "RAW";
        var data = line;
        var name = default(string);
        var detail = default(string);

        var idx = line.IndexOf(':');
        if (idx > 0)
        {
            kind = line[..idx].Trim().ToUpperInvariant();
            data = line[(idx + 1)..].Trim();
        }

        var numbers = ParseNumbers(data);

        if (kind == "F")
        {
            var parts = data.Split(',', 2, StringSplitOptions.TrimEntries);
            if (parts.Length > 0)
            {
                name = parts[0];
            }

            if (parts.Length > 1)
            {
                detail = parts[1];
            }
        }

        return new TelemetryEvent(utc, kind, line, numbers, name, detail);
    }

    public static TelemetryEvent Marker(string marker, string testId, string detail, DateTime utc)
    {
        var raw = $"M:{marker},{testId},{detail}";
        return new TelemetryEvent(utc, "M", raw, Array.Empty<double>(), marker, detail);
    }

    private static double[] ParseNumbers(string csv)
    {
        if (string.IsNullOrWhiteSpace(csv))
        {
            return Array.Empty<double>();
        }

        var parts = csv.Split(',', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);
        var values = new List<double>(parts.Length);
        foreach (var part in parts)
        {
            if (double.TryParse(part, NumberStyles.Float, CultureInfo.InvariantCulture, out var value))
            {
                values.Add(value);
            }
        }

        return values.ToArray();
    }
}

internal sealed record ControlConfigSnapshot(
    DateTime UtcTimestamp,
    string RawLine,
    double? TrimDeg,
    double? Kp,
    double? Ki,
    double? Kd,
    double? HoldKp,
    double? HoldKi,
    double? VelocityKp,
    double? QAngle,
    double? QGyro,
    double? RAngle,
    double? K1,
    double? MinPwm)
{
    public static ControlConfigSnapshot? TryParse(string line)
    {
        if (line is null)
        {
            return null;
        }

        var trimmed = line.Trim();
        if (!trimmed.StartsWith("S:", StringComparison.OrdinalIgnoreCase))
        {
            return null;
        }

        var payload = trimmed[2..];
        var values = payload.Split(',', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries)
            .Select(static value =>
            {
                if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed))
                {
                    return (double?)parsed;
                }

                return null;
            })
            .ToList();

        double? At(int index) => index >= 0 && index < values.Count ? values[index] : null;

        return new ControlConfigSnapshot(
            DateTime.UtcNow,
            trimmed,
            At(0),
            At(1),
            At(2),
            At(3),
            At(4),
            At(5),
            At(6),
            At(7),
            At(8),
            At(9),
            At(10),
            At(11));
    }
}

internal sealed record TestSummary(
    string TestId,
    string Title,
    DateTime StartedUtc,
    DateTime EndedUtc,
    double DurationSec,
    int EventCount,
    int RuntimeSampleCount,
    int FaultCount,
    IReadOnlyList<string> Faults,
    double? PeakAbsAngle,
    double? PeakAbsTargetAngle,
    double? PeakAbsLeftPwm,
    double? PeakAbsRightPwm,
    double? MeanAbsAngle,
    ControlConfigSnapshot? ConfigSnapshot)
{
    public static TestSummary FromEvents(
        TestDefinition test,
        DateTime startedUtc,
        DateTime endedUtc,
        IReadOnlyList<TelemetryEvent> events,
        ControlConfigSnapshot? latestConfig)
    {
        var runtimeFrames = events.Where(static evt => evt.Kind == "T" && evt.NumericValues.Length > 0).ToList();
        var faults = events
            .Where(static evt => evt.Kind == "F")
            .Select(static evt => string.IsNullOrWhiteSpace(evt.Raw) ? "F:<empty>" : evt.Raw)
            .Distinct(StringComparer.Ordinal)
            .ToList();

        double? peakAngle = null;
        double? peakTarget = null;
        double? peakLeftPwm = null;
        double? peakRightPwm = null;

        var absAngles = new List<double>(runtimeFrames.Count);

        foreach (var frame in runtimeFrames)
        {
            if (TryGet(frame.NumericValues, 0, out var angle))
            {
                var abs = Math.Abs(angle);
                absAngles.Add(abs);
                peakAngle = MaxNullable(peakAngle, abs);
            }

            if (TryGet(frame.NumericValues, 3, out var targetAngle))
            {
                peakTarget = MaxNullable(peakTarget, Math.Abs(targetAngle));
            }

            if (TryGet(frame.NumericValues, 7, out var leftPwm))
            {
                peakLeftPwm = MaxNullable(peakLeftPwm, Math.Abs(leftPwm));
            }

            if (TryGet(frame.NumericValues, 8, out var rightPwm))
            {
                peakRightPwm = MaxNullable(peakRightPwm, Math.Abs(rightPwm));
            }
        }

        double? meanAbsAngle = absAngles.Count > 0 ? absAngles.Average() : null;

        return new TestSummary(
            test.Id,
            test.Title,
            startedUtc,
            endedUtc,
            (endedUtc - startedUtc).TotalSeconds,
            events.Count,
            runtimeFrames.Count,
            faults.Count,
            faults,
            peakAngle,
            peakTarget,
            peakLeftPwm,
            peakRightPwm,
            meanAbsAngle,
            latestConfig);
    }

    private static bool TryGet(IReadOnlyList<double> values, int index, out double value)
    {
        if (index >= 0 && index < values.Count)
        {
            value = values[index];
            return true;
        }

        value = default;
        return false;
    }

    private static double? MaxNullable(double? current, double candidate)
    {
        return current is null ? candidate : Math.Max(current.Value, candidate);
    }
}

internal static class HandoffPromptBuilder
{
    public static string Build(TestDefinition test, TestSummary summary, string rawPath, string summaryPath)
    {
        var sb = new StringBuilder();
        sb.AppendLine("# Firmware Tuning Handoff");
        sb.AppendLine();
        sb.AppendLine("You are assisting with balance-bot firmware tuning.");
        sb.AppendLine("Safety mode: suggest patch only. Do not assume auto-upload.");
        sb.AppendLine();
        sb.AppendLine("## Objective");
        sb.AppendLine(test.Goal);
        sb.AppendLine();
        sb.AppendLine("## Test Metadata");
        sb.AppendLine($"- Test Id: {summary.TestId}");
        sb.AppendLine($"- Title: {summary.Title}");
        sb.AppendLine($"- Duration (sec): {summary.DurationSec:0.0}");
        sb.AppendLine($"- Runtime samples: {summary.RuntimeSampleCount}");
        sb.AppendLine($"- Fault count: {summary.FaultCount}");
        sb.AppendLine();
        sb.AppendLine("## Key Metrics");
        sb.AppendLine($"- Peak |angle| deg: {Fmt(summary.PeakAbsAngle)}");
        sb.AppendLine($"- Mean |angle| deg: {Fmt(summary.MeanAbsAngle)}");
        sb.AppendLine($"- Peak |target angle| deg: {Fmt(summary.PeakAbsTargetAngle)}");
        sb.AppendLine($"- Peak |left pwm|: {Fmt(summary.PeakAbsLeftPwm)}");
        sb.AppendLine($"- Peak |right pwm|: {Fmt(summary.PeakAbsRightPwm)}");
        sb.AppendLine();
        sb.AppendLine("## Faults");
        if (summary.Faults.Count == 0)
        {
            sb.AppendLine("- none");
        }
        else
        {
            foreach (var fault in summary.Faults)
            {
                sb.AppendLine($"- {fault}");
            }
        }

        sb.AppendLine();
        sb.AppendLine("## Current Config Snapshot");
        if (summary.ConfigSnapshot is null)
        {
            sb.AppendLine("- No S: frame observed during this session.");
        }
        else
        {
            var cfg = summary.ConfigSnapshot;
            sb.AppendLine($"- trimDeg: {Fmt(cfg.TrimDeg)}");
            sb.AppendLine($"- kp/ki/kd: {Fmt(cfg.Kp)} / {Fmt(cfg.Ki)} / {Fmt(cfg.Kd)}");
            sb.AppendLine($"- holdKp/holdKi/velocityKp: {Fmt(cfg.HoldKp)} / {Fmt(cfg.HoldKi)} / {Fmt(cfg.VelocityKp)}");
            sb.AppendLine($"- qAngle/qGyro/rAngle/k1: {Fmt(cfg.QAngle)} / {Fmt(cfg.QGyro)} / {Fmt(cfg.RAngle)} / {Fmt(cfg.K1)}");
            sb.AppendLine($"- minPwm: {Fmt(cfg.MinPwm)}");
        }

        sb.AppendLine();
        sb.AppendLine("## Artifacts");
        sb.AppendLine($"- Raw events NDJSON: {rawPath}");
        sb.AppendLine($"- Summary JSON: {summaryPath}");
        sb.AppendLine();
        sb.AppendLine("## Requested Output");
        sb.AppendLine("1. Diagnose likely control issues based on metrics and fault lines.");
        sb.AppendLine("2. Propose minimal parameter/code adjustments with safety justification.");
        sb.AppendLine("3. Provide a bounded patch suggestion for firmware files only.");
        sb.AppendLine("4. Include a rollback plan and next test recommendation.");

        return sb.ToString();
    }

    private static string Fmt(double? value)
    {
        return value is null ? "n/a" : value.Value.ToString("0.###", CultureInfo.InvariantCulture);
    }
}

internal sealed record TestRunResult(
    string TestId,
    string RawPath,
    string SummaryPath,
    string HandoffPath,
    IReadOnlyList<TelemetryEvent> RawEvents);
