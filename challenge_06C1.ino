int ledPin = 7;
int period_val;
unsigned long time;

void setup() {
  pinMode(7, OUTPUT);
  Serial.begin(115200);
  set_period(100); // set the period value at here.
}

void loop() {
  for (int fadeValue = 0; fadeValue <= 100; fadeValue +=1) {
    set_duty(fadeValue);
  }
  for (int fadeValue = 100; fadeValue >= 0; fadeValue -=1) {
    set_duty(fadeValue);
  }
  time = millis();
  Serial.println(time);
}

int set_period(int period) {
  period_val = period;
}

int set_duty(int duty) {
  int count = 0;
  int count_limit = 0;
  int newduty = 0;
  
  if (period_val == 100) {
    newduty = duty;
    count_limit = 50;
  }
  else if (period_val == 1000) {
    newduty = duty * 10;
    count_limit = 5;
  }
  else {
    newduty = duty * 100;
    count_limit = 1;
  }
  
  while (count < count_limit) {
    digitalWrite(7, HIGH);
    delayMicroseconds(duty);

    digitalWrite(7, LOW);
    delayMicroseconds(period_val-duty);

    count++;
  }
}
