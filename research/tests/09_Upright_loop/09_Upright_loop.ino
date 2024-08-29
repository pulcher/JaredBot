#include <MsTimer2.h>        //internal timer 2
#include <PinChangeInt.h>    //this library can make all pins of arduino UNO as external interrupt
#include <MPU6050.h>      //MPU6050 library 
#include <Wire.h>        //IIC communication library 

MPU6050 mpu6050;     //Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz;     //Instantiate an MPU6050 object; name mpu6050

//TB6612 pins
const int right_R1=8;    
const int right_R2=12;
const int PWM_R=10;
const int left_L1=7;
const int left_L2=6;
const int PWM_L=9;

///////////////////////angle parameters//////////////////////////////
float Angle;
float angle_X; //calculate the inclined angle variable of X-axis by accelerometer
float angle_Y; //calculate the inclined angle variable of Y-axis by accelerometer
float angle0 = 1.8; //Actual measured angle (ideally 0 degrees)   
// this 1.8 ain't right, but neither is 1.0.  I am not sure what it should be.  More testing is needed.
float Gyro_x,Gyro_y,Gyro_z;  //Angular angular velocity for gyroscope calculation
///////////////////////angle parameters//////////////////////////////

///////////////////////Kalman_Filter////////////////////////////
float Q_angle = 0.001;  //Covariance of gyroscope noise
float Q_gyro = 0.003;    //Covariance of gyroscope drift noise
float R_angle = 0.5;    //Covariance of accelerometer
char C_0 = 1;
float dt = 0.005; // The value of dt is the filter sampling time.
float K1 = 0.05; // a function containing the Kalman gain is used to calculate the deviation of the optimal estimate
float K_0,K_1,t_0,t_1;
float angle_err;
float q_bias;    //gyroscope drift

float accelz = 0;
float angle;
float angleY_one;
float angle_speed;

float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
//////////////////////Kalman_Filter/////////////////////////

//////////////////////PD parameters///////////////////////////////
double kp = 34, ki = 0, kd = 0.62;                   //Angle loop parameter
double setp0 = 0; //Angle balance point
int PD_pwm;  //angle output
float pwm1=0,pwm2=0;

//void anglePWM();

void setup() 
{
  //set the control motor’s pin to OUTPUT
  pinMode(right_R1,OUTPUT);       
  pinMode(right_R2,OUTPUT);
  pinMode(left_L1,OUTPUT);
  pinMode(left_L2,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(PWM_L,OUTPUT);

  //Initial state value
  digitalWrite(right_R1,1);
  digitalWrite(right_R2,0);
  digitalWrite(left_L1,0);
  digitalWrite(left_L2,1);
  analogWrite(PWM_R,0);
  analogWrite(PWM_L,0);

  // Join I2C bus
  Wire.begin();                            //Join the I2C bus sequence
  Serial.begin(9600);                       //open serial monitor, set the baud rate to 9600
  delay(1500);
  mpu6050.initialize();                       //initialize MPU6050
  delay(2);

  //5ms  use timer2 to set the timer interrupt (Note: using timer2 will affect the PWM output of pin3 pin11.)
  MsTimer2::set(5, DSzhongduan);    //5ms execute the function DSzhongduan once
  MsTimer2::start();    // start the interrupt
}

void loop() 
{
  Serial.print("angle = ");
  Serial.print(angle);
  Serial.print("\tAngle = ");
  Serial.print(Angle);
  Serial.print("\tangle_err = ");
  Serial.print(angle_err);
  Serial.print("\tangle0 = ");
  Serial.print(angle0);

  Serial.print("\tGyro_x = ");
  Serial.print(Gyro_x);
  Serial.print("\tK_Gyro_x = ");
  Serial.print(angle_speed);
  
  Serial.print("\t\tPD_pwm = ");
  Serial.print(PD_pwm);
  Serial.print("\t\tpwm1 = ");
  Serial.print(pwm1);
  Serial.print("\t\tpwm2 = ");
  Serial.print(pwm2);

  Serial.println();
}

/////////////////////////////////interrupt////////////////////////////
void DSzhongduan()
{
  sei();  //Allow overall interrupt
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data ax ay az gx gy gz
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);      //get angle and Kalman_Filter
  PD();         // angle loop of PD control
  anglePWM();
}
///////////////////////////////////////////////////////////

/////////////////////////////angle calculation///////////////////////
void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  Angle = -atan2(ay , az) * (180/ PI);           //Radial rotation angle calculation formula; negative sign is direction processing
  Gyro_x = -gx / 131;              //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);            //  Kalman Filter
  //Rotation Angle Z axis parameter
  Gyro_z = -gz / 131;                      //Z-axis angular velocity
  //accelz = az / 16.4;

  float angleAx = -atan2(ax, az) * (180 / PI); //Calculate the angle with the x-axis
  Gyro_y = -gy / 131.00; //Y-axis angular velocity
  Yiorderfilter(angleAx, Gyro_y); //first-order filter
}
////////////////////////////////////////////////////////////////

///////////////////////////////KalmanFilter/////////////////////
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;          //Prior estimate
  angle_err = angle_m - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //Differential of azimuth error covariance
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //A priori estimation error covariance differential integral
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //Intermediate variable of matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //Denominator 
  E = R_angle + C_0 * PCt_0;
  //gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  //Intermediate variable of matrix multiplication
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    //Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    //Posterior estimate
  angle_speed = gyro_m - q_bias;   //The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err; ////Posterior estimation to get the optimal angle
}

/////////////////////first-order Filter/////////////////
void Yiorderfilter(float angle_m, float gyro_m)
{
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

//////////////////angle PD////////////////////
void PD()
{
  PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD angle loop control
}


////////////////////////////PWM end value/////////////////////////////
void anglePWM()
{
  pwm2=-PD_pwm;            //The final value assigned to the motor PWM
  pwm1=-PD_pwm;
  
  if(pwm1>255)             //limit PWM value not greater than 255
  {
    pwm1=255;
  }
  if(pwm1<-255) 
  {
    pwm1=-255;
  }
  if(pwm2>255)
  {
    pwm2=255;
  }
  if(pwm2<-255)
  {
    pwm2=-255;
  }

  if(angle>80 || angle<-80)      //When the self-balancing trolley’s tilt angle is greater than 45 degrees, the motor will stop.
  {
    pwm1=pwm2=0;
  }

  if(pwm2>=0)         //determine the motor’s steering and speed by the positive and negative of PWM 
  {
    digitalWrite(left_L1,LOW);
    digitalWrite(left_L2,HIGH);
    analogWrite(PWM_L,pwm2);
  }
  else
  {
    digitalWrite(left_L1,HIGH);
    digitalWrite(left_L2,LOW);
    analogWrite(PWM_L,-pwm2);
  }

  if(pwm1>=0)
  {
    digitalWrite(right_R1,LOW);
    digitalWrite(right_R2,HIGH);
    analogWrite(PWM_R,pwm1);
  }
  else
  {
    digitalWrite(right_R1,HIGH);
    digitalWrite(right_R2,LOW);
    analogWrite(PWM_R,-pwm1);
  }
}
