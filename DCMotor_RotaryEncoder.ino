# define encoA 3
# define encoB 2

# define ppr 360

int in1 = 12;
int in2 = 4;
int ena = 10;

int pwm = 200;
volatile long encodervalue = 0;
float rpm;
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;


void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(encoA, INPUT_PULLUP);
  pinMode(encoB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoA), UpdateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoB), UpdateEncoder, RISING);
  previousMillis = millis();
}


void loop(){
  // Motor moving forward
  analogWrite(ena, pwm);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
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
    if (digitalRead(encoA) == digitalRead(encoB)){
      encodervalue ++;
    }
    else;
     {
      encodervalue --;
     }

}
