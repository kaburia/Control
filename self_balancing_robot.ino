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
  Wire.beginTransmission(MPU6050_ADDR);       //From the datastheet, the address is 0x68, but you can change that above.
  Wire.write(0x6B);                           //Write on 0x6B register
  Wire.write(0x00);                           //Set register to 00000000 and activate gyro
  Wire.endTransmission();                     //End the i2c transmission
  //Change gyro scale to +/-250deg/sec
  Wire.beginTransmission(MPU6050_ADDR);       //My MPU6050 address is 0x68, change it at the begginning of the code
  Wire.write(0x1B);                           //Write on 0x1B register
  Wire.write(0x00);                           //Set scale to 250dps, full scale
  Wire.endTransmission();                     //End the i2c transmission
  //Change accelerometer scale to +/-4g.
  Wire.beginTransmission(MPU6050_ADDR);       //My MPU6050 address is 0x68, change it at the begginning of the code
  Wire.write(0x1C);                           //Write on 0x1C register
  Wire.write(0x08);                           //Set scale to +/-4g
  Wire.endTransmission();                     //End the i2c transmission
  //Enable some filters
  Wire.beginTransmission(MPU6050_ADDR);       //My MPU6050 address is 0x68, change it at the begginning of the code
  Wire.write(0x1A);                           //Write on 0x1A register
  Wire.write(0x03);                           //Set Digital Low Pass Filter to ~43Hz
  Wire.endTransmission();                     //End the i2c transmission

}

void setup()
{
  
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
  
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();

  Serial.print(" Xraw = ");
  Serial.print(rawAccel.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawAccel.YAxis);
  Serial.print(" Zraw = ");

  Serial.println(rawAccel.ZAxis);
  Serial.print(" Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normAccel.ZAxis);
  
  delay(1000);


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




