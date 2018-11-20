/*Circuit connections:
  -Motor red   --> OUT1 H-bridge
  -Motor white --> OUT2 H-bridge
  -Motor blue  --> 3.3V (encoder power)
  -Motor black --> GND (encoder power)
  -Motor yellow --> Arduino D2 (encoder phase A)
  -Motor green  --> Arduino D3 (encoder phase B)
  -H-bridge IN1 --> Arduino D10
  -H-bridge IN2 --> Arduino D9*/

const byte R_encoderPinA = 2;
const byte R_encoderPinB = 3;
const byte L_encoderPinA = 18;
const byte L_encoderPinB = 19;

volatile long R_pulse_count;
volatile long L_pulse_count;

long prev_millis;

void R_encoderA_ISR()
{

  if (digitalRead(R_encoderPinA) == HIGH)
    (digitalRead(R_encoderPinB) == LOW) ? R_pulse_count++ : R_pulse_count--;
  else
    (digitalRead(R_encoderPinB) == HIGH) ? R_pulse_count++ : R_pulse_count--;
}

void R_encoderB_ISR()
{
  if (digitalRead(R_encoderPinB) == HIGH)
    (digitalRead(R_encoderPinA) == HIGH) ? R_pulse_count++ : R_pulse_count--;
  else
    (digitalRead(R_encoderPinA) == LOW) ? R_pulse_count++ : R_pulse_count--;
}

void L_encoderA_ISR()
{

  if (digitalRead(L_encoderPinA) == HIGH)
    (digitalRead(L_encoderPinB) == LOW) ? L_pulse_count++ : L_pulse_count--;
  else
    (digitalRead(L_encoderPinB) == HIGH) ? L_pulse_count++ : L_pulse_count--;
}

void L_encoderB_ISR()
{
  if (digitalRead(L_encoderPinB) == HIGH)
    (digitalRead(L_encoderPinA) == HIGH) ? L_pulse_count++ : L_pulse_count--;
  else
    (digitalRead(L_encoderPinA) == LOW) ? L_pulse_count++ : L_pulse_count--;
}

struct motor 
{
  byte flip = 1; //Set the direction
  
  byte signalA = 9;
  byte signalB = 10;
  
  int actuation_signal;
  double Kp = 1.6; //2.1
  double ki = 5; //4
  double kd = 0.01;
  double error;
  double prev_error;
  double integral;
  double derivative;
  double speed_reading;
  long prev_pulse_count;

  void initialize(byte a, byte b, byte c)
  {
    signalA = a;
    signalB = b;
    flip = c;
    
    pinMode(signalA, OUTPUT);
    pinMode(signalB, OUTPUT);

    Serial.println("Motor initialized");
  }

  void pid(int set_point, long pulse_count)
  {
    speed_reading = ((prev_pulse_count - pulse_count) / 0.02) / 34.02; //motor shaft RPM
    Serial.println(speed_reading);
    prev_pulse_count = pulse_count;

    error = set_point - speed_reading;

    integral += (error) * 0.02;
    derivative = (error - prev_error) / 0.02;
    actuation_signal = (Kp * error) + (ki * integral) + (kd * derivative);

    prev_error = error;

    if (actuation_signal > 0)
    {
      analogWrite(signalA, (actuation_signal < 255) ? actuation_signal : 255);
      digitalWrite(signalB, LOW);
    }
    else if (actuation_signal < 0)
    {
      actuation_signal = abs(actuation_signal);
      analogWrite(signalA, (actuation_signal < 255) ? actuation_signal : 255);
      digitalWrite(signalB, LOW);
    }
    else
    {
      digitalWrite(signalA, LOW);
      digitalWrite(signalB, LOW);
    }
  }
};

motor motor_R;
motor motor_L;

void setup()
{
  Serial.begin(115200);

  motor_R.initialize(9, 10, 1);
  motor_L.initialize(11, 12, 1);

  pinMode(R_encoderPinA, INPUT_PULLUP);
  pinMode(R_encoderPinA, INPUT_PULLUP);
  pinMode(L_encoderPinB, INPUT_PULLUP);
  pinMode(L_encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(R_encoderPinA), R_encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_encoderPinB), R_encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoderPinA), L_encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoderPinB), L_encoderB_ISR, CHANGE);
}

void loop()
{
  
  if (millis() - prev_millis > 20)
  {
    prev_millis = millis();
    motor_R.pid(100, R_pulse_count);
    motor_L.pid(100, L_pulse_count);
  }
}
