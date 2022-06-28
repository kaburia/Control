#define IN1M 7
#define IN2M 6
#define IN3M 11
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

int pwm = 255;

void setup (){
    Serial.begin(9600);
    pinMode(PWMB, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(IN1M, OUTPUT);
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT);
    pinMode(IN4M, OUTPUT);
    pinMode(STBY, OUTPUT);

}


void loop(){
    analogWrite(PWMA, pwm);
    analogWrite(PWMB, pwm);
    digitalWrite(IN1M, HIGH);
    digitalWrite(IN2M, LOW);
    digitalWrite(IN3M, LOW);
    digitalWrite(IN4M, HIGH);
    digitalWrite(STBY, HIGH);
    delay(1000);
//    digitalWrite(IN3M, LOW);
//    digitalWrite(IN4M, HIGH);
//    digitalWrite(IN3M, LOW);
//    digitalWrite(IN1M, LOW);
//    digitalWrite(IN2M, HIGH);
//    delay(1000);
    

}
