// TB6612FNG MOTOR DRIVER
// M1
#define IN1 12
#define IN2 11
#define PWMA 10
#define ENC_IN 3


// M2
#define IN3 7
#define IN4 6
#define PWMB 9
#define ENC_IN2 2

// STANDBY PIN
#define STBY 8


// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 374


// Analog pin for potentiometer
int speedcontrol = 0;

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

void setup()
{
  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  

  // Setup Serial Monitor
  Serial.begin(9600); 
  
  // Set encoder as input with internal pullup  
  pinMode(ENC_IN, INPUT_PULLUP); // M1
  pinMode(ENC_IN2, INPUT_PULLUP); // M2


  
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENC_IN), updateEncoder1, RISING); // Motor 1
  attachInterrupt(digitalPinToInterrupt(ENC_IN2), updateEncoder2, RISING); // Motor 2
  
  // Setup initial values for timer
  previousMillis = millis();
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



