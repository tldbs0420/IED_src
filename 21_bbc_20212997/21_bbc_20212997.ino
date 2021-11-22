#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
#define INTERVAL 25

Servo myservo;
int a, b; // unit: mm
unsigned long last_sampling_time; // unit: ms
float dist_ema; // unit: mm, for ema filter
float alpha=0.5; // range: 0 to 1

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);
  
// initialize serial port
  Serial.begin(57600);

  a = 83;
  b = 350;
  last_sampling_time = 0;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,target:255,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema);
  if(dist_ema<255) myservo.writeMicroseconds(1900);
  else myservo.writeMicroseconds(1200);
  last_sampling_time += INTERVAL;
  dist_ema=(alpha*dist_cali)+((1-alpha)*dist_ema); // Using ema filter
}
