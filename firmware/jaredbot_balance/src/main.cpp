#include <Arduino.h>
#include <MsTimer2.h>
#include <MPU6050.h>
#include <Wire.h>
#include <NeoSWSerial.h>

// Use analog pins as digital: A3 = 17 (RX), A2 = 16 (TX)
NeoSWSerial extSerial(17, 16); // RX, TX

MPU6050 mpu6050;
int16_t ax, ay, az, gx, gy, gz;

// TB6612 pins
const int right_R1 = 8;
const int right_R2 = 12;
const int PWM_R = 10;
const int left_L1 = 7;
const int left_L2 = 6;
const int PWM_L = 9;

// Angle parameters
float Angle;
float angle0 = 1.8;
float Gyro_x, Gyro_y, Gyro_z;

// Kalman filter variables
float Q_angle = 0.001;
float Q_gyro = 0.003;
float R_angle = 0.5;
char C_0 = 1;
float dt = 0.005;
float K1 = 0.05;

float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias;

float angle = 0;
float angle_speed = 0;
float angleY_one = 0;

float Pdot[4] = {0, 0, 0, 0};
float P[2][2] = {{1, 0}, {0, 1}};
float PCt_0, PCt_1, E;

// PD parameters
double kp = 34, ki = 0, kd = 0.62;
int PD_pwm;
float pwm1 = 0, pwm2 = 0;

// Telemetry timers
unsigned long lastDebug = 0;     // 10 Hz Serial
unsigned long lastCSV = 0;       // 50 Hz extSerial

// Command buffer for ESP32
String cmdBuffer = "";

// Forward declarations
void DSzhongduan();
void angle_calculate(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz,
                     float dt, float Q_angle, float Q_gyro,
                     float R_angle, float C_0, float K1);
void Kalman_Filter(double angle_m, double gyro_m);
void Yiorderfilter(float angle_m, float gyro_m);
void PD();
void anglePWM();

// ---------------- SETUP ----------------
void setup()
{
    pinMode(right_R1, OUTPUT);
    pinMode(right_R2, OUTPUT);
    pinMode(left_L1, OUTPUT);
    pinMode(left_L2, OUTPUT);
    pinMode(PWM_R, OUTPUT);
    pinMode(PWM_L, OUTPUT);

    digitalWrite(right_R1, 1);
    digitalWrite(right_R2, 0);
    digitalWrite(left_L1, 0);
    digitalWrite(left_L2, 1);
    analogWrite(PWM_R, 0);
    analogWrite(PWM_L, 0);

    Wire.begin();
    Serial.begin(9600);
    delay(1500);
    extSerial.begin(9600);
    delay(1500);

    mpu6050.initialize();
    delay(2);

    // Auto-zero calibration
    float sum = 0;
    for (int i = 0; i < 400; i++) {
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float A = -atan2(ay, az) * (180 / PI);
        sum += A;
        delay(5);
    }
    angle0 = sum / 400.0;

    // Timer2 ISR every 5ms
    MsTimer2::set(5, DSzhongduan);
    MsTimer2::start();
}

// ---------------- MAIN LOOP ----------------
void loop()
{
    unsigned long now = millis();

    // ----------- Receive commands from ESP32 -----------
    while (extSerial.available()) {
        char c = extSerial.read();
        if (c == '\n') {
            if (cmdBuffer.length() > 0) {
                if (cmdBuffer.startsWith("C:")) {
                    String command = cmdBuffer.substring(2);
                    command.trim();
                    Serial.print("[ESP32 CMD] ");
                    Serial.println(command);
                } else {
                    Serial.print("[ESP32 INVALID] ");
                    Serial.println(cmdBuffer);
                }
            }
            cmdBuffer = "";
        } else if (c != '\r') {
            cmdBuffer += c;
        }
    }

    // ----------- 10 Hz USB Serial Debug -----------
    if (now - lastDebug >= 100) {
        lastDebug = now;

        Serial.print("angle=");
        Serial.print(angle);
        Serial.print(" speed=");
        Serial.print(angle_speed);
        Serial.print(" pwm=");
        Serial.print(PD_pwm);
        Serial.println();
    }

    // ----------- 50 Hz CSV Telemetry on extSerial -----------
    if (now - lastCSV >= 20) {
        lastCSV = now;

        // CSV: angle,angle_speed,gyro_x,pd_pwm,pwm1,pwm2
        extSerial.print("T:");
        extSerial.print(angle);
        extSerial.print(",");
        extSerial.print(angle_speed);
        extSerial.print(",");
        extSerial.print(Gyro_x);
        extSerial.print(",");
        extSerial.print(PD_pwm);
        extSerial.print(",");
        extSerial.print(pwm1);
        extSerial.print(",");
        extSerial.print(pwm2);
        extSerial.println();
    }
}

// ---------------- ISR ----------------
void DSzhongduan()
{
    sei();
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
    PD();
    anglePWM();
}

// ---------------- ANGLE CALC ----------------
void angle_calculate(int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz,
                     float dt, float Q_angle, float Q_gyro,
                     float R_angle, float C_0, float K1)
{
    Angle = -atan2(ay, az) * (180 / PI);
    Gyro_x = -gx / 131.0;
    Kalman_Filter(Angle, Gyro_x);

    Gyro_z = -gz / 131.0;

    float angleAx = -atan2(ax, az) * (180 / PI);
    Gyro_y = -gy / 131.0;
    Yiorderfilter(angleAx, Gyro_y);
}

// ---------------- KALMAN FILTER ----------------
void Kalman_Filter(double angle_m, double gyro_m)
{
    angle += (gyro_m - q_bias) * dt;
    angle_err = angle_m - angle;

    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    q_bias += K_1 * angle_err;
    angle_speed = gyro_m - q_bias;
    angle += K_0 * angle_err;
}

// ---------------- FIRST ORDER FILTER ----------------
void Yiorderfilter(float angle_m, float gyro_m)
{
    angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

// ---------------- PD CONTROL ----------------
void PD()
{
    PD_pwm = kp * (angle + angle0) + kd * angle_speed;
}

// ---------------- MOTOR PWM ----------------
void anglePWM()
{
    pwm1 = -PD_pwm;
    pwm2 = -PD_pwm;

    if (pwm1 > 255) pwm1 = 255;
    if (pwm1 < -255) pwm1 = -255;
    if (pwm2 > 255) pwm2 = 255;
    if (pwm2 < -255) pwm2 = -255;

    if (angle > 80 || angle < -80) {
        pwm1 = pwm2 = 0;
    }

    if (pwm2 >= 0) {
        digitalWrite(left_L1, LOW);
        digitalWrite(left_L2, HIGH);
        analogWrite(PWM_L, pwm2);
    } else {
        digitalWrite(left_L1, HIGH);
        digitalWrite(left_L2, LOW);
        analogWrite(PWM_L, -pwm2);
    }

    if (pwm1 >= 0) {
        digitalWrite(right_R1, LOW);
        digitalWrite(right_R2, HIGH);
        analogWrite(PWM_R, pwm1);
    } else {
        digitalWrite(right_R1, HIGH);
        digitalWrite(right_R2, LOW);
        analogWrite(PWM_R, -pwm1);
    }
}
