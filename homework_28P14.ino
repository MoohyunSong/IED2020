#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 430

// Distance sensor
#define _DIST_ALPHA 0.5
#define DELAY_MICROS 1500
#define EMA_ALPHA 0.35

float ema_dist = 0;
float filtered_dist;
float samples_num = 3;

// Servo range
#define _DUTY_MIN 1150 
#define _DUTY_NEU 1470
#define _DUTY_MAX 1850 

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 150

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 0.8
#define _KD 50.0
#define _KI 0.01

//unit: mm
#define _CALI_MIN 69
#define _CALI_MAX 290

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 
  duty_target = duty_curr = 1300;
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(duty_curr);
  
  // move servo to neutral position
  
  pinMode(PIN_LED, OUTPUT);
  
// initialize global variables
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  error_prev = 0;
  pterm = iterm = dterm = 0;
  

  event_dist = false;
  event_servo = false;
  event_serial = false;
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
}
  
void loop() {
/////////////////////
// Event generator //
/////////////////////
  // [1615] 거리 측정 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
      event_dist = true;
    
  // [1615] 서보 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
      event_servo= true;
    
  // [1615] Serial 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
      event_serial= true;

////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    float x = ir_distance();
    dist_raw = filtered_ir_distance();

  // PID control logic
    error_curr = _DIST_TARGET - dist_raw;

    pterm = error_curr * _KP;
    dterm = (error_curr - error_prev) * _KD;
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    else if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    error_prev = error_curr;
    
    last_sampling_time_dist += _INTERVAL_DIST;
  }
  
  if(event_servo) {
    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);

    last_sampling_time_servo += _INTERVAL_SERVO;
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");

    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}


float ir_distance(void){ // return value unit: mm
  float val, result;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  result = 100 + 300.0 / (_CALI_MAX - _CALI_MIN) * (val - _CALI_MIN);
  return result;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
