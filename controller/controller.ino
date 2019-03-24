#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

std_msgs::Int16 left;
std_msgs::Int16 right;

ros::Publisher lwheel("lwheel", &left);
ros::Publisher rwheel("rwheel", &right);

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


void setup()
{
  nh.initNode();
  nh.advertise(left);
  nh.advertise(right);
  
  Serial.begin(115200);

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
  left.publish(&left);
  right.publish(&right);
  nh.spinOnce();
  delay(100);
}
