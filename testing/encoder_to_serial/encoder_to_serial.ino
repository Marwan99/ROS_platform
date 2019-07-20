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

const byte r_motor = 9;
const byte l_motor = 10;

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


void setup()
{
  pinMode(R_encoderPinA, INPUT_PULLUP);
  pinMode(R_encoderPinA, INPUT_PULLUP);
  pinMode(L_encoderPinB, INPUT_PULLUP);
  pinMode(L_encoderPinB, INPUT_PULLUP);

  pinMode(r_motor, OUTPUT);
  pinMode(l_motor, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(R_encoderPinA), R_encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_encoderPinB), R_encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoderPinA), L_encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoderPinB), L_encoderB_ISR, CHANGE);

  Serial.begin(9600);
}

void loop()
{
    Serial.print("Left wheel count: ");
    Serial.println(L_pulse_count);
    
    Serial.print("Right wheel count: ");
    Serial.println(R_pulse_count);
    
    delay(1000);
}
