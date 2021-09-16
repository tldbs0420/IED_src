#define PIN_LED 7
unsigned int toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  toggle = 0;
}

void loop(){
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  for(int i=0;i<5;i++){
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }
  digitalWrite(PIN_LED, 1);
  while (1){} // infinite loop
}

int toggle_state(int toggle){
  if (toggle == 0)
    return 1;
  else if (toggle == 1)
    return 0;
}
