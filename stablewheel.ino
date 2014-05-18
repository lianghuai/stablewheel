#include <PID_v1.h>
#include "Psx_analog.h"  
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu(0x68);

#define  sonic_in 12
#define sonic_out 13
#define attPin 4
#define clockPin 5

#define center  0x7F
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float angle;
float angle_offset = 19.5;
double Setpoint, Input, Output,setpoint; 

//double kp = 4.8,ki = 20,kd = 0.08,gof=-1.7,gob=1.5,dl=75 // GOOD
//double kp = 4,ki = 25,kd = 0.08,gof=-1.7,gob=1.5,dl=80; //BETTER
double kp = 4,ki = 40,kd = 0.05,gof=-1.7,gob=1.5,dl=80; //BETTER

unsigned char count,up=1,nt=0;
char flag[10]={1,1,1,1,1,1,1,1,1,1};
signed char tl,tr,en=0,led=0,pressl,pressr;
union{
	signed int all;
	unsigned char s[2];
}data;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
PID myPID(&Input, &Output, &Setpoint,kp,ki,kd, DIRECT); 

void motor(int v)
{
  v=v;
  if(v>0)
  {
    v+=dl;
    digitalWrite(5,0);
    digitalWrite(6,1); 
    digitalWrite(7,1);
    digitalWrite(8,0);   
    analogWrite(9,v+tl);
    analogWrite(10,v+tr); 
  }
  else if(v<0)
  {
    v=-v;
    v+=dl;
    digitalWrite(5,1);
    digitalWrite(6,0); 
    digitalWrite(7,0);
    digitalWrite(8,1);   
    analogWrite(9,v+tl);
    analogWrite(10,v+tr); 
  }
  else
  {
    analogWrite(9,0);
    analogWrite(10,0);  
  }
}

 void setup() 
{  
  pinMode(sonic_in,INPUT);
  pinMode(sonic_out,OUTPUT);
  
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  digitalWrite(3,0);
  digitalWrite(4,1);
  digitalWrite(8,1);
  digitalWrite(9,0);
  analogWrite(9,0);
  analogWrite(10,0); 
  digitalWrite(3,1);

  delay(100); 
  Wire.begin();
    Serial.begin(38400);
  while (!Serial);
  
  mpu.initialize();
  delay(2);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) 
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
  
  Setpoint = -0.1;
  setpoint = -0.1;
  myPID.SetTunings(kp,ki,kd);
  myPID.SetOutputLimits(-255+dl,255-dl);
  myPID.SetSampleTime(5);
  myPID.SetMode(AUTOMATIC);
}
 
void loop()
{
  //Serial.println("loop");
  if (!dmpReady) 
    return;
  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) 
    return;
    //Serial.println("get interrupt");
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
   mpu.resetFIFO();
  } 
  else if (mpuIntStatus & 0x02) 
  {
     while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
     mpu.getFIFOBytes(fifoBuffer, packetSize);
     fifoCount -= packetSize;
     mpu.dmpGetQuaternion(&q, fifoBuffer);
     mpu.dmpGetGravity(&gravity, &q);
     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  //从DMP中取出Yaw、Pitch、Roll三个轴的角度，放入数组ypr。单位：弧度
     angle=-ypr[2] * 180/M_PI  + angle_offset;
  }
  Input = angle;
  myPID.Compute();
  myPID.Compute();

  /*
  Serial.print(angle);
  Serial.print(",");
  Serial.print(Output);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.println(0);
  */
  if(angle>50||angle<-50)
  Output=0;
  motor(Output);
  count++;
  
  //askDistance();
  /*
  if(count==5)
  {
    count=0;
    Psx.poll();
    Serial.write(10);
  }
  if (Psx.digital_buttons & psxLeft)                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
  {
    tl=5;
    tr=-5;
    pressl=1;
  }
  else{if(pressl)
  {tl=0,tr=0;pressl=0;}
}
  if (Psx.digital_buttons & psxRight)                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
  {
    tr=5;
    tl=-5;
    pressr=1;
  }
  else{if(pressr)
  {tl=0,tr=0;pressr=0;}}
 
  if(Psx.digital_buttons & psxUp)
  {
    Setpoint=gof;
    kd=0.2;
    nt=0;
  }
  if(Psx.digital_buttons & psxDown)
  {
    Setpoint=gob;
    kd=0.2;
    nt=0;
  }
  if (Psx.digital_buttons & psxL2)                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
  {
    if(flag[0])
    setpoint+=0.1;  
    flag[0]=0;    // If button is pressed, turn on the LED
  }
  else flag[0]=1;
  if (Psx.digital_buttons & psxR2)                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
  {
    if(flag[1])
    setpoint-=0.1;  
    flag[1]=0;    // If button is pressed, turn on the LED
  }
  else flag[1]=1;
  
  
   if (Psx.digital_buttons & psxX)                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
  {
    Setpoint=gob;
    kd=0.2;
    nt=0;
  }
  if (Psx.digital_buttons & psxSqu)                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
  {
    Setpoint=gof;
    kd=0.2;
    nt=0;
  }
    if (!((Psx.digital_buttons & psxSqu)||(Psx.digital_buttons & psxX)||(Psx.digital_buttons & psxUp)||(Psx.digital_buttons & psxDown)))                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
    {Setpoint=setpoint;

    }
  
  if (Psx.digital_buttons & psxSlct)                                       // If the data anded with a button's hex value is true,                                                          // it signifies the button is pressed. Hex values for each                                                        // button can be found in Psx.h
  {
    motor(0);
    digitalWrite(3,1);
    while(1)
    {
      Serial.print(kp);
  Serial.print(',');
  Serial.print(ki);
  Serial.print(',');
  Serial.print(kd);
  Serial.print(',');
  Serial.print(dl);
  Serial.print(',');
  Serial.println(Setpoint);
    }
  }
  if (Psx.digital_buttons & psxO)  
  {
    tl=20;
    tr=20;
    en=1;
  }
  else {if(en){tr=0;tl=0;en=0;}}

if (Psx.digital_buttons & psxTri)  
  {
    ki=0;
    kd=20;
    Setpoint=-10.0;
  }
  if (Psx.digital_buttons & psxL1) 
  {led=0;digitalWrite(3,1);}
  if (Psx.digital_buttons & psxR1) 
  {led=1;}
  if (Psx.digital_buttons & psxStrt)
{motor(0);
digitalWrite(3,1);
while(1){}}  
  myPID.SetTunings(kp,ki,kd);  
 if(led)
 {
analogWrite(3,nt);  
if(up)
{nt++;
if(nt==255)
up=0;}
else{nt--;
if(nt==0)
up=1;}
 }
  */
}

int start_move = 0;
int last_detect = 0;

void askDistance() // 量出前方距離
{
  
  int now = millis();
  if(start_move == 0 && (now - last_detect)>1000) {
    
        last_detect = now;
  	digitalWrite(sonic_out, LOW); // 讓超聲波發射低電壓2μs
    	delayMicroseconds(2);
    	digitalWrite(sonic_out, HIGH); // 讓超聲波發射高電壓10μs，這裡至少是10μs
    	delayMicroseconds(10);
    	digitalWrite(sonic_out, LOW); // 維持超聲波發射低電壓
    	float distance = pulseIn(sonic_in, HIGH); // 讀差相差時間
    	distance = distance/5.8/10; // 將時間轉為距離距离（單位：公分）

        Serial.print("distance\t");
        Serial.println(distance);
        if(distance < 10) {
          start_move = now;
          tl = 10;
          tr = -10;
          Setpoint = 0.2;
          Serial.println("SET MOVE");
        } 
        
        
  } else if(start_move > 0) {
     if((now - start_move) > 10000) {
       Serial.println("STOP MOVE");
       start_move = 0;
       tl = 0;
       tr = 0;
       Setpoint = -0.1;
     }
  }

}

