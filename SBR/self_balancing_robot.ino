#include <Wire.h>
#include <MPU6050.h>
#include "I2Cdev.h"
#include "math.h"

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// TB6612FNG MOTOR DRIVER//////////////////////////
///////////////////////////////////////////////////////////////////////////////

// M1
#define IN1 12
#define IN2 11
#define PWMA 10
#define ENC_IN 4


// M2
#define IN3 7
#define IN4 6
#define PWMB 9
#define ENC_IN2 2

// STANDBY PIN
#define STBY 8


// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 374



// Pulse count from encoder
volatile long encoderValue1 = 0; // M1
volatile long encoderValue2 = 0; // M2

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment
int rpm1 = 0; // M1
int rpm2 = 0; // M2

// Variable for PWM motor speed output
int motorPwm1 = 255;
int motorPwm2 = 250;
///////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// MPU-6050//////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


// Initializing the MPU-6050
MPU6050 mpu;

volatile long accelX, accelY, accelZ, gForceX, gForceY, gForceZ, gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;




volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;
/////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
//////////////////////pulse calculation/////////////////////////
/////////////////////////////////////////////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int pulseright,pulseleft;
////////////////////////////////PI variable parameters//////////////////////////
float speeds_filterold=0;
float positions=0;
int flag1;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;


////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////KALMAN FILTER//////////////////////////
////////////////////////////////////////////////////////////////////////////


//Define three-axis acceleration, three-axis gyroscope variables
int16_t ax, ay, az, gx, gy, gz; 
 
// The tilt angle
float Angle;

 
///////////////////////Kalman_Filter////////////////////////////
// Covariance of gyroscope noise
float Q_angle = 0.001;  
 
// Covariance of gyroscope drift noise
float Q_gyro = 0.003;  
 
// Covariance of accelerometer
float R_angle = 0.5;    
char C_0 = 1;
 
// The filter sampling time.
float dt = 0.005;
 
// a function containing the Kalman gain is used to 
// calculate the deviation of the optimal estimate.
float K1 = 0.05; 
float K_0,K_1,t_0,t_1;
float angle_err;
 
// Gyroscope drift 
float q_bias;    
 
float accelz = 0;
float angle;
float angleY_one;
float angle_speed;
 
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
//////////////////////Kalman_Filter/////////////////////////

///////////////////////angle parameters//////////////////////////////
float angle_X; //Calculate the tilt angle variable about the X axis from the acceleration
float angle_Y; //Calculate the tilt angle variable about the Y axis from the acceleration
float angle0 = 1; //Actual measured angle (ideally 0 degrees)
float Gyro_x,Gyro_y,Gyro_z;  //Angular angular velocity by gyroscope calculation
///////////////////////angle parameters//////////////////////////////

//////////////////////////////////////////////////////////////////
//////////////////////PID parameters///////////////////////////////
/////////////////////////////////////////////////////////////////

double kp = 34, ki = 0, kd = 0.62;                   //angle loop parameters
double kp_speed = 3.6, ki_speed = 0.080, kd_speed = 0;   // speed loop parameters
double setp0 = 0; //angle balance point
int PD_pwm;  //angle output
float pwm1=0,pwm2=0;

////////////////////////////////////////////////////////////////////////


// Setting up the motor
void startMotor(){
  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

   // Set encoder as input with internal pullup  
  pinMode(ENC_IN, INPUT_PULLUP); // M1
  pinMode(ENC_IN2, INPUT_PULLUP); // M2


  
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder1, RISING); // Motor 1 //// Check between change and rising
  attachInterrupt(digitalPinToInterrupt(ENC_IN2), updateEncoder2, RISING); // Motor 2 /// Check between change and rising
  

}

void startmpu(){
    //Start MPU6050 communication
  Wire.beginTransmission(0x68);       //From the datastheet, the address is 0x68, but you can change that above.
  Wire.write(0x6B);                           //Write on 0x6B register
  Wire.write(0x00);                           //Set register to 00000000 and activate gyro
  Wire.endTransmission();                     //End the i2c transmission
  //Change gyro scale to +/-250deg/sec
  Wire.beginTransmission(0x68);       //My MPU6050 address is 0x68, change it at the begginning of the code
  Wire.write(0x1B);                           //Write on 0x1B register
  Wire.write(0x00);                           //Set scale to 250dps, full scale
  Wire.endTransmission();                     //End the i2c transmission
  //Change accelerometer scale to +/-4g.
  Wire.beginTransmission(0x68);       //My MPU6050 address is 0x68, change it at the begginning of the code
  Wire.write(0x1C);                           //Write on 0x1C register
  Wire.write(0b00000000);                           //Set scale to +/-4g
  Wire.endTransmission();                     //End the i2c transmission


  //Enable some filters
  /*When we start, the gyro might have an offset value. We make 520 readdings and get that calibration value
  We use taht later in the code to substract the raw offset. 
  for (int i = 0; i < 520; i++) {                               //Create 520 loops
    
    Wire.beginTransmission(MPU6050_ADDR);                       //Start i2c communication with MPU6050
    Wire.write(0x43);                                           //We read from register 0x43
    Wire.endTransmission();                                     //End the i2c transmission
    Wire.requestFrom(MPU6050_ADDR, 4);                          //Request 2 bytes from the MPU6050
    Gyro_Y_Offset += Wire.read() << 8 | Wire.read();            //Merge high and low byte and get an integer
    Gyro_X_Offset += Wire.read() << 8 | Wire.read();            //Merge high and low byte and get an integer
    delayMicroseconds(3500);                                    //Small delay
  }
  Gyro_X_Offset /= 520;                                         //Divide the total value by 520 to get the avarage gyro offset
  Gyro_Y_Offset /= 520;                                         //Divide the total value by 520 to get the avarage gyro offset

*/

}



void recordAccelRegisters() {
  // REGISTER 0x3B~0x40/REGISTER 59~64
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accelerometer Readings (see datasheet)
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  
  // Use left shift << and bit operations |  Wire.read() read once 1bytes，and automatically read the data of the next address on the next call.
  while(Wire.available() < 6);  // Waiting for all the 6 bytes data to be sent from the slave machine （Must wait for all data to be stored in the buffer before reading） 
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX （Automatically stored as a defined long value）
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}
 
 // divide the raw values by 16384, according to the datasheet
void processAccelData(){
  gForceX = accelX / 16384.0;     //float = long / float
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}
  
void recordGyroRegisters() {
  // REGISTER 0x43~0x48/REGISTER 67~72
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 ~ 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}
 
// The 131.0 comes from the MPU6050 datasheet.
void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

// Print out 3-axis gyroscope data and 3-axis accelerometer data
// Get the values then offset them
void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
   
 
  
 
  // Print the tilt angle in degrees
  Serial.print("Angle = ");
  Serial.print(Angle);
 
  // Print the angular velocity in degrees per second
  Serial.print("   Gyro_x = ");
  Serial.println(Gyro_x);


  
    // Only update display when there is a reading
    if (motorPwm1 > 0 || rpm1 > 0) {
      Serial.print("PWMA VALUE: ");
      Serial.print(motorPwm1);
      Serial.print('\t');
      Serial.print(" PULSES: ");
      Serial.print(encoderValue1);
      Serial.print('\t');
      Serial.print(" SPEEDA: ");
      Serial.print(rpm1);
      Serial.println(" RPM");
      Serial.print("PWMB VALUE: ");
      Serial.print(motorPwm2);
      Serial.print('\t');
      Serial.print(" PULSES: ");
      Serial.print(encoderValue2);
      Serial.print('\t');
      Serial.print(" SPEED: ");
      Serial.print(rpm2);
      Serial.println(" RPMB");
    }
  // After applying Kalman Filter
  Serial.print("Angle = ");
  Serial.print(Angle);
  Serial.print("  K_angle = ");
  Serial.println(angle);
  Serial.print("Gyro_x = ");
  Serial.print(Gyro_x);
  Serial.print("  K_Gyro_x = ");
  Serial.println(angle_speed);
}

/*

*/




void setup()
{
  mpu.initialize();
  
  // Setup Serial Monitor
  Serial.begin(9600); 
  
  // Setup initial values for timer
  previousMillis = millis();

  // Starting the motor
  startMotor();

  Serial.println("Initialize MPU6050");

  startmpu();

}






void loop()
{
  printData();
  recordAccelRegisters();
  recordGyroRegisters();
  anglePWM();
    
    // Write PWM to controller
    analogWrite(PWMA, motorPwm1);
    analogWrite(PWMB, motorPwm2);

    // Motor Movement

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    digitalWrite(STBY, HIGH);
    

  
  // Update RPM value every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;


    // Calculate RPM
    rpm1 = (float)(encoderValue1 * 60 / ENC_COUNT_REV);
    rpm2 = (float)(encoderValue2 * 60 / ENC_COUNT_REV);
    // I2C to get MPU6050 six-axis ax ay az gx gy gz
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     
 
  // Obtain angle and Kalman Filter 
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);     

    
    encoderValue1 = 0;
    encoderValue2 = 0;
  }
  
  


}

/////////////////////////////angle calculate///////////////////////
void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  float Angle = -atan2(ay , az) * (180/ PI);           //Radial rotation angle calculation formula; negative sign is direction processing
  Gyro_x = -gx / 131;              //The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);            //Kalman Filtering
  //Rotation angle Z-axis parameter
  Gyro_z = -gz / 131;                      //Z-axis angular velocity
  //accelz = az / 16.4;
 
  float angleAx = -atan2(ax, az) * (180 / PI); //Calculate the angle with the x-axis
  Gyro_y = -gy / 131.00; //Y-axis angular velocity
  Yiorderfilter(angleAx, Gyro_y); //first-order filtering
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


/////////////////////first-order filtering/////////////////
void Yiorderfilter(float angle_m, float gyro_m)
{
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}
 
//////////////////angle PD////////////////////
void PD()
{
  PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD angle loop control
}


/////////////////speed PI////////////////////
void speedpiout()
{
  float speeds = (pulseleft + pulseright) * 1.0;      //Vehicle speed  pulse value
  pulseright = pulseleft = 0;      //Clear
  speeds_filterold *= 0.7;         //first-order complementary filtering
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -3550,3550);    //Anti-integral saturation
  PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);      //speed loop control PI
}
//////////////////speed PI////////////////////
 
 
////////////////////////////PWM end value/////////////////////////////
void anglePWM()
{
  pwm2=-PD_pwm - PI_pwm ;           //assign the final value of PWM to motor 
  pwm1=-PD_pwm - PI_pwm ;
   
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
 
  // When the self-balancing trolley’s tilt angle gets less than a certain angle, 
  // the motor will stop.
   
  if(angle>25 || angle<-25)      
  {
    pwm1=pwm2=0;
  }
 
  // Determine the motor’s steering and speed by the positive and negative of PWM 
  if(pwm2>=0)         
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(PWMA,pwm2);
  }
  else
  {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
    analogWrite(PWMA,-pwm2);
  }
 
  if(pwm1>=0)
  {
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(PWMB,pwm1);
  }
  else
  {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(PWMB,-pwm1);
  }
}


void updateEncoder1()
{
  // Increment value for each pulse from encoder
  encoderValue1++;
}

void updateEncoder2()
{
  // Increment value for each pulse from encoder
  encoderValue2++;
}

////////////////////pulse calculation///////////////////////
void countpluse()
{
  lz = encoderValue1;     //Assign the value counted by the code wheel to lz
  rz = encoderValue2;
 
  encoderValue1 = 0;     //Clear the code counter count
  encoderValue2 = 0;
 
  lpluse = lz;
  rpluse = rz;
 
  if ((pwm1 < 0) && (pwm2 < 0))                     //judge the moving direction; if backwards（PWM, namely motor voltage is negative）, pulse number is a negative number
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 > 0))                 // if backwards（PWM, namely motor voltage is positive）, pulse number is a positive number
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((pwm1 < 0) && (pwm2 > 0))                 //Judge turning direction of the car;  turn left; Right pulse number is a positive number; Left pulse number is a negative number.
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 < 0))               //Judge turning direction of the car;  turn right; Right pulse number is a negative number; Left pulse number is a positive number.
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }
 
  //enter interrupts per 5ms; pulse number superposes
  pulseright += rpluse;
  pulseleft += lpluse;
}
 
/////////////////////////////////interrupts////////////////////////////
void Interrupt_Service_Routine()
{
  sei();  //Allow global interrupts
  countpluse();        //Pulse superposition subfunction
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC to get MPU6050 six-axis data  ax ay az gx gy gz
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);      //get angle and Kalman filtering
  PD();         //angle loop PD control
  anglePWM();
 
  cc++;
  if(cc>=8)     //5*8=40，40ms entering once speed PI algorithm  
  {
    speedpiout();   
    cc=0;  //Clear
  }
}
///////////////////////////////////////////////////////////