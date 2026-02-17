namespace dashboard.Models;

public sealed class ControlConfigSnapshot
{
    public DateTime UtcTimestamp { get; init; }

    public double? Setpoint { get; init; }
    public double? Kp { get; init; }
    public double? Ki { get; init; }
    public double? Kd { get; init; }

    public string? RawLine { get; init; }
}
