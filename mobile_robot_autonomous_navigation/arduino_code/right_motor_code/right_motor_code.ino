// PID motor position control.
// Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <PinChangeInt.h>
#include <Wire.h>
#include <PID_v1.h>

#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              10

double kp =1, ki =20 , kd =0;             // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd,DIRECT);  
void setup() {

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  
  Wire.begin(9);                // join i2c bus with address #9 for Right Motor
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
  //Serial.begin (9600); 
  
}

void loop() {
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H-Bridge module
  delay(10);
}

void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, 0);                             // drive motor CW
    analogWrite(M2, out);
  }
  else {
    analogWrite(M1, abs(out));
    analogWrite(M2, 0);                        // drive motor CCW
  }
}


void requestEvent() {
  int8_t s = (int8_t)(setpoint * 255 / 360);  // Simulating the encoder change proportionally to setpoint
  Wire.write(s);  // respond with simulated encoder change
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  uint8_t a,b;
  a = Wire.read();
  b = Wire.read();
  setpoint= (double)((b<<8)|a);
  //Serial.println((uint8_t)a);
  //Serial.println((uint8_t)b);
  
  //Serial.println(setpoint);
}
