#include <PID_v1.h>

#include <SoftwareSerial.h>

#include <ArduinoSTL.h>

//#include <ArduinoSTL.h>
//#include <PID_v1.h>
using namespace std;
// encoder 6000 rpm
vector<uint8_t> data_send;
vector<uint8_t> data_recevice;
// hardware robot
float encoder_relution=600; //pulse/cycle
float R=0.072,L=0.455;
float gear=6;
//pid for motor 1
double Pk1 = 2.0;
double Ik1 = 10;
double Dk1 = 0.05;

double Setpoint1, Input1, Output1, Output1a;  // PID variables
PID PID1(&Input1 ,&Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);  //PID setup

//pid for motor 2
double Pk2 = 2.0;
double Ik2 = 10;
double Dk2 = 0.05;

double Setpoint2, Input2, Output2, Output2a;  // PID variables
PID PID2(&Input2 ,&Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT);  //PID setup

// linear and rotary robot m/s vs rad/s
float ax=0.5,aw=1.0;
float v_set1,v_set2;
float w_set1,w_set2;
float vr_set;
float vl_set;
float v_read,w_read,vr_read,vl_read;
float time_out_uart;
//
unsigned long currentMillis;
unsigned long previousMillis;
int looptime = 100;

float encoder1_count;
float encoder2_count;


// wheel encoder interrupt
#define encoder_0_PinA 2           // encoder 1
#define encoder_0_PinB 4

#define encoder_1_PinA 3          // encoder 2
#define encoder_1_PinB 7

#define motor_0_in1 5 //motor 1 input 1(pwm pin)
#define motor_0_in2 6 //motor 1 input 2(pwm pin)

#define motor_1_in1 10 //motor 2 input 1(pwm pin)
#define motor_1_in2 11 //motor 2 input 2(pwm pin)

void setup() {
  //
  pinMode(motor_0_in1,OUTPUT);
  pinMode(motor_0_in2,OUTPUT);
  pinMode(motor_1_in1,OUTPUT);
  pinMode(motor_1_in2,OUTPUT);

  pinMode(encoder_0_PinA,INPUT_PULLUP);
  pinMode(encoder_0_PinB,INPUT_PULLUP);
  
  pinMode(encoder_1_PinA,INPUT_PULLUP);
  pinMode(encoder_1_PinB,INPUT_PULLUP);
  
  attachInterrupt(0,doencoderA, RISING); //left
 
  attachInterrupt(1,doencoderC, RISING); //right

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255,255);
  PID1.SetSampleTime(looptime);

  PID2.SetMode(AUTOMATIC);
  PID2.SetOutputLimits(-255,255);
  PID2.SetSampleTime(looptime);
  //
  Serial.begin(115200);
}
void serialEvent() {
  //statements
  static uint8_t data;
  data=(uint8_t)Serial.read();
  data_recevice.resize(data_recevice.size()+1);
  data_recevice[data_recevice.size()-1]=data;
  //
  if(data_recevice.size()>=16){
    if(data_recevice[data_recevice.size()-1]==0xff & data_recevice[data_recevice.size()-16]==0xff){
      // get data from uart
      v_set1=(float)data_recevice[1]/100;
      if(data_recevice[2]==0) v_set1=-v_set1;
      w_set1=(float)data_recevice[3]/100;
      if(data_recevice[4]==0) w_set1=-w_set1;
      data_recevice.resize(0);
      // sendata to pi
      static uint8_t data_send[16];
      data_send[0]=0xff;
            //
            data_send[1]=(uint8_t)(fabs(vl_read)*10);
            data_send[2]=(uint8_t)(fabs(vr_read)*10);
            data_send[3]=(uint8_t)(fabs(v_read)*100);
            data_send[4]=(uint8_t)(fabs(w_read)*100);
            //
            if(vl_read>0) data_send[5]=1;
            else data_send[5]=0;
            if(vr_read>0) data_send[6]=1;
            else data_send[6]=0;
            if(v_read>0) data_send[7]=1;
            else data_send[7]=0;
            if(w_read>0) data_send[8]=1;
            else data_send[8]=0;
      data_send[15]=0xff;
      //
      Serial.write(data_send,16);
      time_out_uart=0;
    }
  }
}
void loop(){
  //
  currentMillis = millis();
  if(currentMillis - previousMillis >= looptime) {
    previousMillis = currentMillis;
    time_out_uart+=(float)(looptime)/1000;
    if(time_out_uart>=2.0){
      v_set2=0;
      w_set2=0;
      v_set1=0;
      w_set1=0;
    }
    // acceleration for robot
    if(v_set1>v_set2) {
        v_set2+=ax*(float)(looptime)/1000;
        if(v_set2>v_set1) v_set2=v_set1;
    }
    if(v_set1<v_set2){
        v_set2-=ax*(float)(looptime)/1000;
        if(v_set2<v_set1) v_set2=v_set1;
    }
    if(w_set1>w_set2) {
            w_set2+=aw*(float)(looptime)/1000;
            if(w_set2>w_set1) w_set2=w_set1;
    }
    if(w_set1<w_set2){
        w_set2-=aw*(float)(looptime)/1000;
        if(w_set2<w_set1) w_set2=w_set1;
    }
     //v_set2=100;
    // w_set2=0.314;
    //
    vl_set=(2*v_set2-w_set2*L)/(2*R);
    vl_read=encoder1_count/encoder_relution*2*3.1415*1000/looptime;
    encoder1_count=0;
    Setpoint1 =vl_set;
    Input1 =vl_read;
    PID1.Compute();
    //
    vr_set=(2*v_set2+w_set2*L)/(2*R);
    vr_read=encoder2_count/encoder_relution*2*3.1415*1000/looptime;
    encoder2_count=0;
    Setpoint2 =vr_set;
    Input2 =vr_read;
    PID2.Compute();
    //drive motor
    //Motor 1
    //Output1=100;
    //Output2=100;
    if (Output1 > 0){
      Output1a = abs (Output1);
      analogWrite(motor_0_in1,(int)Output1a);
      analogWrite(motor_0_in2,0);
    }
    if (Output1 < 0){
      Output1a = abs (Output1);
      analogWrite(motor_0_in2,(int)Output1a);
      analogWrite(motor_0_in1,0);
    }
    //Motor 2
    if (Output2 > 0){
      Output2a = abs (Output2);
      analogWrite(motor_1_in1,(int)Output2a);
      analogWrite(motor_1_in2,0);
    }
    if (Output2 < 0){
      Output2a = abs (Output2);
      analogWrite(motor_1_in2,(int)Output2a);
      analogWrite(motor_1_in1,0);
    }
    // read v_read,w_read;
    v_read=R/2*(vr_read+vl_read);
    w_read=R/L*(vr_read-vl_read);
    //
    //string data_string=String(vl_read)+"|"+String(vr_read);
    // Serial.write(String(vr_read).c_str());
    // Serial.write("|");
    // Serial.write(String(vr_set).c_str());
    // Serial.write("|");
    // Serial.write(String(vl_set).c_str());
    // Serial.write("|");
    // Serial.write(String(vl_read).c_str());
    // Serial.write("|");
    // Serial.write(String((int)Output1a).c_str());
    // Serial.write("|");
    // Serial.write(String((int)Output2a).c_str());
    // Serial.write("\n");

  }
}
//encoderInterrupt
//encoder 1
void doencoderA(){
  // look for a low-to-high on channel A
  if(digitalRead(encoder_0_PinA) == HIGH){
    // check channel B to see which way encoder is turning
    if(digitalRead(encoder_0_PinB) == LOW){
      encoder1_count = encoder1_count + 1; //CW
    }
    else {
      encoder1_count = encoder1_count - 1; //CCW
    }
  }
}
// encoder 2
void doencoderC(){
  // look for a low-to-high on channel B
  if(digitalRead(encoder_1_PinA) == HIGH){
    // check channel A to see which way encoder is turning
    if(digitalRead(encoder_1_PinB) == LOW){
      encoder2_count = encoder2_count + 1; //CW
    }
    else {
      encoder2_count = encoder2_count - 1; //CCW
    }
  }
}
