#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

std_msgs::Int16 left;
std_msgs::Int16 right;

/*Circuit connections:
 * =========Right Motor==========
  -Motor yellow --> Arduino D2 (encoder phase A)
  -Motor green  --> Arduino D3 (encoder phase B)
  -H-bridge IN1 --> Arduino D10
  -H-bridge IN2 --> Arduino D9

  ==========Left Motor===========
  -Motor yellow --> Arduino D19 (encoder phase A)
  -Motor green  --> Arduino D18 (encoder phase B)

  ============Both================
  -Motor red   --> OUT1 H-bridge
  -Motor white --> OUT2 H-bridge
  -Motor blue  --> 3.3V (encoder power)
  -Motor black --> GND (encoder power)
  */

const byte R_encoderPinA = 2;
const byte R_encoderPinB = 3;
const byte L_encoderPinA = 18;
const byte L_encoderPinB = 19;

const byte r_motor_fwd = 9;
const byte r_motor_bck = 10;

const byte l_motor_fwd = 11;
const byte l_motor_bck = 12;

const float wheel_circumference = 3.14*0.065; // in meters

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

class Motor {
  private:
    long prev_pulse_count;
    double previous_time=0, previous_error=0, integral=0, revolutions, dt,
    actuation_signal, current_time, error, derivative, speed_reading, Kp, Ki, Kd;
    const double pulses_per_revolution = 341.2;
    const double millis_in_a_minute = 6000;
    const int deadband = 0;
    int out;

  public:
    Motor(double set_Kp, double set_Ki, double set_Kd) {
      Kp = set_Kp;
      Ki = set_Ki;
      Kd = set_Kd;
    }

    int pid(int set_point, long pulse_count)
    {
      current_time = millis();
      revolutions = ((pulse_count - prev_pulse_count) / pulses_per_revolution);
      dt = (current_time - previous_time);

      previous_time = current_time;
      prev_pulse_count = pulse_count;

      if(dt==0)
        speed_reading=0;
      else
        speed_reading = revolutions/(dt/millis_in_a_minute); // Speed in RPM

      error = set_point - speed_reading;

      integral += ((error) * dt);
      derivative = (error - previous_error) / dt;
      actuation_signal = (Kp * error) + (Ki * integral) + (Kd * derivative);

      previous_error = error;

      if (actuation_signal > deadband) {
        out = (actuation_signal < 255) ? actuation_signal : 255;
        return out;
      } else if (actuation_signal < -(deadband)) {
        out = (actuation_signal > -255) ? actuation_signal : -255;
        return out;
      } else {
        return 0;
      } 
    }
};

Motor left_motor(4, 0.1, 0);
Motor right_motor(4, 0.1, 0);

float left_target, right_target;

//======================ROS===========================
void l_messageCb(const std_msgs::Float32& l_cmd){

  /* The incoming message should be in m/s. However the PID controller controls
    RPM, so a conversion is needed.
  */

  left_target = (l_cmd.data/wheel_circumference)*60;
}

void r_messageCb(const std_msgs::Float32& r_cmd){

  /* The incoming message should be in m/s. However the PID controller controls
    RPM, so a conversion is needed.
  */

  right_target = (r_cmd.data/wheel_circumference)*60;
}

ros::Subscriber<std_msgs::Float32> l_sub("lwheel_vtarget", &l_messageCb );
ros::Subscriber<std_msgs::Float32> r_sub("rwheel_vtarget", &r_messageCb );

ros::Publisher lwheel("lwheel", &left);
ros::Publisher rwheel("rwheel", &right);


void setup()
{
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
  nh.subscribe(l_sub);
  nh.subscribe(r_sub);

  pinMode(R_encoderPinA, INPUT_PULLUP);
  pinMode(R_encoderPinA, INPUT_PULLUP);
  pinMode(L_encoderPinB, INPUT_PULLUP);
  pinMode(L_encoderPinB, INPUT_PULLUP);

  pinMode(r_motor_fwd, OUTPUT);
  pinMode(r_motor_bck, OUTPUT);
  pinMode(l_motor_fwd, OUTPUT);
  pinMode(l_motor_bck, OUTPUT);


  attachInterrupt(digitalPinToInterrupt(R_encoderPinA), R_encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_encoderPinB), R_encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoderPinA), L_encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoderPinB), L_encoderB_ISR, CHANGE);
}

void loop()
{
    left.data = L_pulse_count;
    right.data = R_pulse_count;
    lwheel.publish(&left);
    rwheel.publish(&right);

    if(left_target > 0){
      analogWrite(l_motor_fwd, left_motor.pid(left_target, L_pulse_count));
      analogWrite(l_motor_bck, 0);
    } else {
      analogWrite(l_motor_bck, abs(left_motor.pid(left_target, L_pulse_count)));
      analogWrite(l_motor_fwd, 0);
    }

    if(right_target > 0){
      analogWrite(r_motor_fwd, right_motor.pid(right_target, R_pulse_count));
      analogWrite(r_motor_bck, 0);
    } else {
      analogWrite(r_motor_bck, abs(right_motor.pid(right_target, R_pulse_count)));
      analogWrite(r_motor_fwd, 0);
    }
  
    nh.spinOnce();
    delay(20);
}
