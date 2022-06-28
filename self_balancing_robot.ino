
// TB6612FNG Pins
// right
#define PWMA 9
#define IN1M 7
#define IN2M 6
// left
#define IN3M 11
#define IN4M 12
#define PWMB 10

#define STBY 8

// Encoder
#define encode_left 2
#define encode_right 4

#define ppr 360

int pwm = 255;
volatile long encodervalue = 0;
float rpm;
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

void setup (){
    Serial.begin(9600);
    pinMode(PWMB, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(IN1M, OUTPUT);
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT);
    pinMode(IN4M, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(encode_left, INPUT_PULLUP);
    pinMode(encode_right, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encode_left), UpdateEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encode_right), UpdateEncoder, RISING);
    previousMillis = millis();

}


void loop(){
  // Both motors moving forward
    analogWrite(PWMA, 220);
    analogWrite(PWMB, 200);
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
    digitalWrite(STBY, HIGH);
    delay(1000);
    rpm = (float)((encodervalue * 60) / ppr)/3;

   //to update milliseconds every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    previousMillis = currentMillis;
  }

  if(pwm > 0 || rpm > 0){
    Serial.print("PWM: ");
    Serial.print(pwm);
    Serial.print('\t');
    Serial.print("PULSE: ");
    Serial.print(encodervalue);
    Serial.print('\t');
    Serial.print("SPEED: ");
    Serial.print(rpm);
    Serial.println(" RPM");
  }



}


void UpdateEncoder(){
    if (digitalRead(encode_left) == digitalRead(encode_right)){
      encodervalue ++;
    }
    else;
     {
      encodervalue --;
     }

}
