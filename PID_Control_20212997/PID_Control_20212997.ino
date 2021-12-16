#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9 // [1234] LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10 // [1352] 서보모터를 아두이노의 10번 핀에 연결
#define PIN_IR A0 // [1352] IR센서를 아두이노 A0 핀에 연결

// Framework setting
#define _DIST_TARGET 255 // [1352]목표값을 탁구공 중심 위치까지 거리 255mm로 Fix
#define _DIST_MIN 100 // [1352] 최소 측정 거리를 100mm로 설정
#define _DIST_MAX 410 // [1352] 측정 거리의 최댓값을 410mm로 설정

// Distance sensor
#define _DIST_ALPHA 0.2 // [2979] 알파값 설정


// Servo range
#define _DUTY_MIN 1000 //[2998] Servo Minimum microseconds
#define _DUTY_NEU 1515 //[2983] Servo 중간값 설정
#define _DUTY_MAX 2000 // [2979] Servo Max값 설정

// Servo speed control
#define _SERVO_ANGLE 30 // [2992] 서보 각 설정
#define _SERVO_SPEED 35 // [2976] 서보 스피드 설정

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20 // [2980] 서보 INTERVAL값 설정
#define _INTERVAL_SERIAL 100 //[2989] 시리얼 출력 INTERVAL 설정

// PID parameters
#define _KP 3.7 // [3000] K🇵값 초기화
#define _KD 165
#define _KI 0.03


//calibrate sensor base value
#define DIST_10C 130 //[2998] sample value for sensor 10cm dist value for calibrate
#define DIST_40C 340 //[2998] sample value for sensor 40cm dist value for calibrate
#define DIST_20C 235
#define SIZE 25
#define mSIZE 7



//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo; //[2991] create servo object to control a servo

// Distance sensor
float dist_target; // location to send the ball [2976] 공 위치
float dist_min, dist_max, dist_raw, dist_ema, dist_prev, dist_cali;// [2981] 거리 변수 설정(현재, ema 필터 적용, 직전값) 
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; //[2989] 논리형 변수 설정
// Servo speed control
int duty_chg_per_interval; //[1352] 주기동안 duty 변화량 변수 
int duty_target, duty_curr;//[1352] 서보모터의 목표위치, 서보에 실제 입력할 위치
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
//[2998] initialize variables for Low Pass Filter
float alpha = _DIST_ALPHA; //[2999] alpha 선언 및 값 적용
int a=80, b=350; //350
int msize=SIZE-mSIZE;

int test[SIZE];
int ttest[SIZE];

float sum=0;

//[2998] initialize variables for Low Pass Filter
unsigned long oldmil; //[2998] old milliseconds var
static long apt = 0;  //[2998] filter sampling time saver
int fc = 15; //[2998] cut off frequency (5~ 15 hz recommended)
float dt = 20/1000.0; //[2998] interval setup
float lambda = 2*PI*fc*dt; //[2998] lambda setup
float filter = 0.0, prev = 0.0; //[2998] setup for filter and prev var’s
int result;


void setup() {
   // initialize GPIO pins for LED and attach servo
   pinMode(PIN_LED, OUTPUT); //[1352] LED 핀 설정
   // myservo.attach(PIN_SERVO); //[1352] Servo 핀 설정 
   // [2998] servo attach duplicated
   // move servo to neutral position
   myservo.attach(PIN_SERVO); //[2998] Servo Attaching 
   myservo.writeMicroseconds(_DUTY_NEU); //[2998] Servo Set to neutral position

   // initialize global variables
   dist_min = _DIST_MIN; //[2999] dist_min 값 적용
   dist_max = _DIST_MAX; //[2999] dist_max 값 적용
   dist_ema = 0.0; //[2999] dist_ema 값 초기화
   dist_target=_DIST_TARGET;
   error_prev=0.0;
   duty_curr = _DUTY_NEU;

   // convert angle speed into duty change per interval.
   duty_chg_per_interval = _SERVO_SPEED; //[2985] duty_chg_per_interval 설정

   
   // initialize serial port
   Serial.begin(57600); //[2999] serial port 57600



}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
    unsigned long time_curr = millis();
    if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
      last_sampling_time_dist += _INTERVAL_DIST;
      event_dist = true;
    }
    if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
    }
    if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
      last_sampling_time_serial += _INTERVAL_SERIAL;
      event_serial = true;
    }//[2981] 인터벌 값과 샘플링 차이의 합과 비교해서 각각의 이벤트 실행

//[2998] Fixed some indentation, tabs and more
////////////////////
// Event handlers //
////////////////////

   if(event_dist) {
     event_dist = false;//[2981] 이벤트가 실행되었으므로 재실행을 위한 값 초기화
     // get a distance reading from the distance sensor
     dist_raw = ir_distance_filtered(); //[3000]raw_dist 거리값 설정

  // PID control logic
     error_curr = dist_target - dist_raw; //[3000] 에러는 목표거리에서 탁구공의 거리를 뺀값
     pterm = _KP*error_curr; //[3000] 제어식에서의 비례항
     //control = pterm; //+ iterm + dterm; [3000] PID결과값

     dterm=_KD*(error_curr-error_prev);

     iterm += _KI * error_curr;
     
     control = pterm + dterm + iterm;
     duty_target = _DUTY_NEU + control;
     // duty_target = f(duty_neutral, control)
     //duty_target = _DUTY_NEU+pterm;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
      // [1352] 서보의 가동범위를 고정
     if (duty_target < _DUTY_MIN) {  //[1352] 조건일때 duty_target값을 _DUTY_MIN으로
        duty_target = _DUTY_MIN; 
      }
     else if (duty_target > _DUTY_MAX) { //[1352] 조건일때 duty_target값을 _DUTY_MAX으로
        duty_target = _DUTY_MAX;
      }
      
     error_prev = error_curr; //[3000] 이전의 에러값을 저장
   }
  
   if(event_servo) {
      event_servo = false; //[1352]server Event handler가 false
      // adjust duty_curr toward duty_target by duty_chg_per_interval
      // update servo position       
      if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr>duty_target) duty_curr=duty_target;
      }
      else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr<duty_target) duty_curr=duty_target;
      }
      myservo.writeMicroseconds(duty_curr); 
      //[3000] 서보위치 업데이트 error_curr 뒤의 숫자는 수정가능 
  }
  
  if(event_serial) {
     event_serial = false; //[2989]  이벤트 실행 후 초기화
     /*Serial.print("Min:0,Low:200,dist:"); //[2983] “Min:0,Low:200,dist:” 문구 출력
     Serial.print(dist_raw); //[2979] dist_raw 값(적외선 센서에서 돌려받은 값) 출력
     Serial.print(",pterm:"); 
     Serial.print(pterm);
     Serial.print(",duty_target:"); //[2994] duty_target 값 출력
     Serial.print(duty_target); //[2994] duty_target 값 출력
     Serial.print(",duty_curr:"); // [2980] “duty_curr” 출력
     Serial.print(duty_curr); // [2980] duty_curr 값 출력
     Serial.println(",High:310,Max:2000");*/

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
     
     
  }
  //[2998] if you are using the Low Pass Filter delay(20) could be needed
}

float ir_distance(void){ // return value unit: mm
    float val; //[2998] Announce val 
    float volt = float(analogRead(PIN_IR)); //[2998] Read analog data from PIN_IR
     
    val = ((6762.0/(volt-9.0))-4.0) * 10.0; 
    //[2998] Calculate analog data to usable format
    return val; //[2998] Return val 
    //[2998] Fixed some indentation
}

int sort_asc(const void *cmp1, const void *cmp2)
{
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);

  return a-b;
}

float ir_distance_filtered(void){ // return value unit: mm
   //[2997] EMA 필터 대신 다른 필터를 사용한 것 같은데 맞나요? 
   //[2998] Low Pass Filter 를 사용했는 소스이며 밑에 EMA 필터도 추가해서 둘중
   // 하나를 선택하게 해도 될듯 합니다.
   //[2997] return (alpha*ir_distance())+((1-alpha)*dist_ema);
   //[2997] *모든 학부생들 분께* dist_ema 사용시 코드입니다. 
   //[2997] EMA필터 사용시 이용해주시기 바랍니다.
   //dist_cali= 100 + 300.0 / (b - a) * (ir_distance() - a);
   //dist_ema=(alpha*dist_cali)+((1-alpha)*dist_ema);
   //return dist_ema;

   //dist_cali= 100 + 300.0 / (b - a) * (ir_distance() - a);
   //dist_ema=(alpha*dist_cali)+((1-alpha)*dist_ema);
   

   //dist_cali= 100 + 300.0 / (b - a) * (ir_distance() - a);
   
   sum=0;
   for(int i=0; i<SIZE-1;i++){
    test[i]=test[i+1];
    //Serial.print(i);
    //Serial.print(":");
    //Serial.println(test[i]);
   }
   test[SIZE-1]=ir_distance();
   //Serial.print("15:");
   //Serial.println(test[SIZE-1]);
   memcpy(ttest, test, sizeof(ttest));
   qsort(ttest, sizeof(ttest)/sizeof(ttest[0]), sizeof(ttest[0]), sort_asc);
   for (int i=mSIZE-1; i<SIZE-mSIZE;i++){
    sum+=ttest[i];
   }
   
   
   result=sum/(SIZE-(2*(mSIZE)));
   dist_cali= 100 + 300.0 / (b - a) * (result - a);
   dist_ema=(alpha*dist_cali)+((1-alpha)*dist_ema);
   
   return dist_ema;
   

}
