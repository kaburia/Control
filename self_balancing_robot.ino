#include <Wire.h>
#include <MPU6050.h>
#include "I2Cdev.h"
#include "math.h"

// TB6612FNG MOTOR DRIVER
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


// Initializing the MPU-6050
MPU6050 mpu;

int16_t accy, accz, gyroX;
float accAngle;
unsigned long currentTime, previousTime=0, loopTime;
int gyrox, gyroRate;
float gyroAngle = 0;


volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;



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
  attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder1, RISING); // Motor 1
  attachInterrupt(digitalPinToInterrupt(ENC_IN2), updateEncoder2, RISING); // Motor 2
  

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
   // I2C to get MPU6050 six-axis data  ax ay az gx gy gz
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
  // Radial rotation angle calculation formula; 
  // The negative sign indicates the direction.
  // Convert radians to degrees.
  Angle = -atan2(ay , az) * (180/ PI);     
 
  // The x-axis angular velocity calculated by the gyroscope; 
  // The negative sign indicates the direction. The 131 value comes
  // from the MPU6050 datasheet.
  Gyro_x = -gx / 131;  
 
  // Print the tilt angle in degrees
  Serial.print("Angle = ");
  Serial.print(Angle);
 
  // Print the angular velocity in degrees per second
  Serial.print("   Gyro_x = ");
  Serial.println(Gyro_x);
}

/*

*/


// Calculating angle

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
    
    encoderValue1 = 0;
    encoderValue2 = 0;
  }
  
  


}

/////////////////////////////angle calculate///////////////////////
void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  // Radial rotation angle calculation formula; negative sign is direction processing
  Angle = -atan2(ay , az) * (180/ PI); 
 
  // The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Gyro_x = -gx / 131;      
 
  // KalmanFilter 
  Kalman_Filter(Angle, Gyro_x);            
}
////////////////////////////////////////////////////////////////
 
///////////////////////////////KalmanFilter/////////////////////
void Kalman_Filter(double angle_m, double gyro_m)
{
  // Prior estimate
  angle += (gyro_m - q_bias) * dt;          
  angle_err = angle_m - angle;
 
  // Differential of azimuth error covariance
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
 
  // The integral of the covariance differential of the prior estimate error
  P[0][0] += Pdot[0] * dt;    
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
   
  // Intermediate variable of matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
   
  // Denominator
  E = R_angle + C_0 * PCt_0;
   
  // Gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
 
  // Intermediate variable of matrix multiplication
  t_0 = PCt_0;  
  t_1 = C_0 * P[0][1];
 
  // Posterior estimation error covariance
  P[0][0] -= K_0 * t_0;     
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
 
  // Posterior estimation
  q_bias += K_1 * angle_err;    
 
  // The differential value of the output value; work out the optimal angular velocity
  angle_speed = gyro_m - q_bias;   
 
  ////Posterior estimation; work out the optimal angle
  angle += K_0 * angle_err; 
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

ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accy, accz)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}



/*






void setup(){
  mpu.initialize();
  Serial.begin(115200);
}

void loop(){

  // Getting the angle
  accy = mpu.getAccelerationZ();
  accz = mpu.getAccelerationY();
  
  accAngle = atan2(accy, accz); *RAD_TO_DEG

  if (isnan(accAngle));
  else
    Serial.println(accAngle);

  // Getting the rate of change in the angle
  // Time
  currentTime = millis();
  loopTime = currentTime - previousTime;
  previousTime = currentTime;

  gyrox = mpu.getRotationX();
  gyroRate = map(gyrox, -32768,32767, -250,250);
  gyroAngle = gyroAngle + (float)gyroRate *loopTime/1000;

  Serial.println(gyroAngle);


  // Complementary Filters
  // Accelerometer affected by sudden Horizontal Movements
  // Gyroscope drifts away from actual value 
  
  // currentAngle = 0.9934 * (previousAngle + gyroAngle) + 0.0066 * (accAngle)



}

*/

/*

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>

#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(1593);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
  // initialize PID sampling loop
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  // measure distance every 100 milliseconds
  if((count%20) == 0){
    distanceCm = sonar.ping_cm();
  }
  if((distanceCm < 20) && (distanceCm != 0)) {
    setMotors(-motorPower, motorPower);
  }
}
// The ISR will be called every 5 milliseconds

*/




