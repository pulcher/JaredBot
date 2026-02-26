using System.Globalization;

namespace dashboard;

public static class JaredBotBalanceCommands
{
    private const char NewLine = '\n';

    public static string Sp(float setpointDeg) => Line($"C:SP,{F4(setpointDeg)}");

    public static string Kp(float kp) => Line($"C:KP,{F4(kp)}");
    public static string Ki(float ki) => Line($"C:KI,{F4(ki)}");
    public static string Kd(float kd) => Line($"C:KD,{F4(kd)}");

    public static string Pid(float kp, float ki, float kd) => Line($"C:PID,{F4(kp)},{F4(ki)},{F4(kd)}");

    public static string Qa(float qAngle) => Line($"C:QA,{F6(qAngle)}");
    public static string Qg(float qGyro) => Line($"C:QG,{F6(qGyro)}");
    public static string Ra(float rAngle) => Line($"C:RA,{F6(rAngle)}");
    public static string K1(float k1) => Line($"C:K1,{F4(k1)}");

    public static string Kf(float qAngle, float qGyro, float rAngle) => Line($"C:KF,{F6(qAngle)},{F6(qGyro)},{F6(rAngle)}");

    public static string Set(
        float setpointDeg,
        float kp,
        float ki,
        float kd,
        float qAngle,
        float qGyro,
        float rAngle,
        float k1)
        => Line($"C:SET,{F4(setpointDeg)},{F4(kp)},{F4(ki)},{F4(kd)},{F6(qAngle)},{F6(qGyro)},{F6(rAngle)},{F4(k1)}");

    public static string Reset() => Line("C:RESET");
    public static string KfReset() => Line("C:KFRESET");

    public static (string kp, string set) ExampleLines()
        => (Kp(40.0f), Set(0.0f, 40.0f, 0.0f, 0.0f, 0.0010f, 0.0030f, 0.0300f, 0.0000f));

    private static string Line(string body)
    {
        if (body.Length > 0 && body[^1] == NewLine)
            return body;

        return body + NewLine;
    }

    private static string F4(float v) => v.ToString("0.0000", CultureInfo.InvariantCulture);
    private static string F6(float v) => v.ToString("0.000000", CultureInfo.InvariantCulture);
}
