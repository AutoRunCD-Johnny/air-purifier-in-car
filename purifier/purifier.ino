/**********************************/
/*Bluetooth controlled smart air purifier in-car
 * 
 * 1. detect pm2.5 and HCHO concentration in air
 * 2. fan speed control via either pm2.5 or HCHO concentration in automatic
 * 3. monitored and controlled via bluetooth
 * 
 * Auhor: Jingting(Johnny) Jiang
 * start date: Aug 8th 2017
 */


//A0 read the HCHO gas, A1 read PM2.5
//D2 detect button to switch mode
//D3 output PWM
//D4 control relay
//D5 driving LED of PM25
//D7,D8,D9 control RGB indicator LED
//D6 and D10 are software Tx and Rx port
//D11 detect state of BT module
//PWM OC2B control the motor rpm
//relay control the fan power

/*there are 4 modes
 * M0: manual mode, use potential meter to change rpm, red light
 * M1: PM2.5 auto mode, rpm vs PM2.5 concentration, green light
 * M2: HCHO auto mode, rpm vs HCHO concentration, blue light
 * M4: OFF mode, no light
 */
 
#include <SoftwareSerial.h>
#include <math.h>
#define mode_pin 2
#define PWM_pin 3 
#define relay_pin 4
#define LED_drive 5
#define R 7
#define G 8
#define B 9
#define STx 6
#define SRx 10
#define bt_stat 11 
#define baud 38400


SoftwareSerial BTSerial(SRx, STx);
String input;
boolean flag=true;
byte mode=3,temp_mode,fan_speed=4,temp_speed,light=1;
float PM25=0,HCHO=0;
unsigned long T0,T1;
//unsigned short t0,t1;

void setup(){
  T0=millis();//start time counting
  pwm_init();
  
  pinMode(LED_drive,OUTPUT); //PM2.5 LED driving pin
  digitalWrite(LED_drive,HIGH);

  pinMode(R,OUTPUT);
  pinMode(G,OUTPUT);
  pinMode(B,OUTPUT);
  digitalWrite(R,LOW);
  digitalWrite(G,LOW);
  digitalWrite(B,LOW);
  
  pinMode(relay_pin,OUTPUT); 
  digitalWrite(relay_pin,HIGH);//use pnp to drive relay,low is ON
  
  attachInterrupt(digitalPinToInterrupt(mode_pin),mode_swap,LOW); 
  attachInterrupt(digitalPinToInterrupt(bt_stat),bt_switch,CHANGE);
  
  ADCSRA&=0b11111100;//change default ADC clock prescale from 128 to 16
  /*Serial.begin(9600);
  while(!Serial){
  Serial.println("connnecting");
  }
  Serial.println("connected");*/
  BTSerial.begin(baud);
}

//when OCR2B=2, rpm is lowest, when OCR2B>=6 rpm is highest..

void loop(){
  PM25=PM25_read();
  HCHO=HCHO_read();
  if(BTSerial.available()){
    input=BTSerial.readString();
    temp_mode=input[0];
    temp_speed=fan_speed;
    fan_speed=input[1]-48;
    light=input[2]-48;
    //BTSerial.println(fan_speed);
    if(temp_mode==48){
      mode=0;
      if(fan_speed<0||fan_speed>7){
        fan_speed=temp_speed;
        BTSerial.println("invalid input,try again.(type-in from 0 to 6)");
      } 
    }
    else if(temp_mode==49)
    mode=1;
    else if(temp_mode==50)
    mode=2;
    else if(temp_mode==51)
    mode=3;
    else{
    BTSerial.println("invalid input!(type-in 0,1,2,3 to switch mode)");
    fan_speed=temp_speed;
    }
  }
  BTSerial.print("mode: ");
  BTSerial.println(mode);
  
//manual
  if(mode==0){
    digitalWrite(relay_pin,LOW);
    if(!flag){
      pwm_init();
      flag=true;
    }
    //OCR2B=analogRead(A2)/110;
    //Serial.println(analogRead(A2));
    OCR2B=fan_speed+2;
    BTSerial.print("fan_speed: ");
    BTSerial.println(fan_speed);
    if(light==1){
      digitalWrite(R,HIGH);
      digitalWrite(G,LOW);
      digitalWrite(B,LOW);
    }
    else{
      digitalWrite(R,LOW);
      digitalWrite(G,LOW);
      digitalWrite(B,LOW);
    }
  }
  
//PM2.5
  else if(mode==1){
    digitalWrite(relay_pin,LOW);
    //mode_1();
    //OCR2B=6;
    if(!flag){
      pwm_init();
      flag=true;
    }    
    if(PM25<=30)
    OCR2B=3;
    else if(PM25>30&&PM25<=40)
    OCR2B=4;
    else if(PM25>40&&PM25<=50)
    OCR2B=8;
    else if(PM25>50&&PM25<=70)
    OCR2B=6;
    else if(PM25>70&&PM25<=90)
    OCR2B=7;
    else if(PM25>90)
    OCR2B=8;
    if(light==1){
      digitalWrite(R,LOW);
      digitalWrite(G,HIGH);
      digitalWrite(B,LOW);
    }
    else{
      digitalWrite(R,LOW);
      digitalWrite(G,LOW);
      digitalWrite(B,LOW);
    }
    BTSerial.print("fan_speed: ");
    BTSerial.println(OCR2B-2);
    //BTSerial.println(OCR2B);
  }
  
//HCHO  
  else if(mode==2){
    digitalWrite(relay_pin,LOW);
    //mode_2();
    //OCR2B=f(HCHO);
    //OCR2B=4;
    //test_pwm();
    //Serial.println(OCR2B+1);
    if(!flag){
      pwm_init();
      flag=true;
    }
    if(HCHO<=0.04)
    OCR2B=3;
    else if(HCHO>0.04&&HCHO<=0.10)
    OCR2B=4;
    else if(HCHO>0.10&&HCHO<=0.18)
    OCR2B=5;
    else if(HCHO>0.18&&HCHO<=0.25)
    OCR2B=6;
    else if(HCHO>0.25&&HCHO<=0.4)
    OCR2B=7;
    else if(HCHO>0.4)
    OCR2B=8;
    if(light==1){
      digitalWrite(R,LOW);
      digitalWrite(G,LOW);
      digitalWrite(B,HIGH);
    }
    else{
      digitalWrite(R,LOW);
      digitalWrite(G,LOW);
      digitalWrite(B,LOW);
    }
    BTSerial.print("fan_speed: ");
    BTSerial.println(OCR2B-2);
  }
  
//off  
  else if(mode==3){
    digitalWrite(relay_pin,HIGH);
    digitalWrite(PWM_pin,LOW);//turn off pwm output
    flag=false;
    digitalWrite(R,LOW);
    digitalWrite(G,LOW);
    digitalWrite(B,LOW);
  }
  /*Serial.print("PM2.5: ");
  Serial.println(PM25);
  Serial.print("HCHO concentration: ");
  Serial.println(HCHO);*/
  if(millis()-T0<600000){
    BTSerial.print("HCHO reading currently inaccurate please wait for ");
    BTSerial.print(600-(millis()-T0)/1000.0);
    BTSerial.println(" seconds.");
  }
  BTSerial.print("PM2.5(ug/m^3): ");
  BTSerial.println(PM25);
  BTSerial.print("HCHO(mg/m^3): ");
  BTSerial.println(HCHO);
  //delay(5000);
}

void mode_swap(){
  detachInterrupt(digitalPinToInterrupt(mode_pin));
  BTSerial.println("interrupt");
  if(mode<3)
  mode++;
  else
  mode=0;
  //BTSerial.print("mode: ");
  //BTSerial.println(mode);
//debounce
  delay(20);
  while(digitalRead(mode_pin)!=HIGH){
    delay(5);
  }
  attachInterrupt(digitalPinToInterrupt(mode_pin),mode_swap,LOW);
}

void pwm_init(){
  // Fast PWM use tiumer/counter2
  pinMode(PWM_pin,OUTPUT); //OC2B PWM output,PD5
  //pinMode(11,OUTPUT); //OC2A PWM output,PD6
  byte COM2A=1; //Toggle OC2A on compare match 
  byte COM2B=2; //OC2B in non-inverting mode
  byte WGM0=3; //WGM21:20
  byte WGM1=1; //WGM22
  byte CS2=3;//16MHz 64 prescale(N=64), f=7813Hz,  default CS2 is 3
//Timer register setting
  TCCR2A=(COM2A<<6)+(COM2B<<4)+WGM0;
  TCCR2B=(WGM1<<3)+CS2;
  OCR2A=9; //f=16MHz/(64*9+1)=25kHz 
  OCR2B=0; //duty cycle= (OCR0B+1)/(1+OCR0A)
}

void bt_switch(){
  if(digitalRead(bt_stat)==LOW){
    BTSerial.end();
  }
  else{
    BTSerial.begin(baud);
  }
}

float PM25_read(){
  unsigned long val=0,temp_val=0;
  float output=0.0;
  byte N=200; //calculate the avg in 200 samples, take 2 seconds
  digitalWrite(LED_drive,HIGH);
  delay(1);
  for(byte i=0;i<N;i++){
    //t0=micros();
    digitalWrite(LED_drive,LOW);//if LED pin drived by pnp then LOW. npn use HIGH
    delayMicroseconds(280);
    //t0=micros();
    temp_val=analogRead(A1);// cost 112us default, 18us when prescaler os 16
    //t1=micros();
    //Serial.println(t1-t0);
    delayMicroseconds(22); //40-18
    digitalWrite(LED_drive,HIGH);
    delayMicroseconds(9680);
    //t1=micros();
    val+=temp_val;
    //Serial.println(temp_val);
  }
  output=((((((float)val*5.0/1024.0)/float(N))-0.6)/0.5)*100); //check datasheet for each parameter
  //BTSerial.println((val*5.0/1024.0)/N);
  return output;
}

float HCHO_read(){
  float output=0;
  float volt=0;
  short N=200;
  for(byte i=0;i<N;i++){
    volt+=analogRead(A0)*5.0/1024.0;
    //BTSerial.println(volt/(1+i));
  }
  volt/=N;
  output=pow(10.0,-3.665+3.009*volt-0.362*pow(volt,2));
  return output*30.013/24.45;
}

/*
void test_pwm(){
  if(flag){
    if(OCR2B<9)
      ++OCR2B;  
    else
      flag=false;
  }
  else if(!flag){
    if(OCR2B!=0)
      --OCR2B;
    else
      flag=true;
  }
}

*/

