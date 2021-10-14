#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include <Servo.h>

Servo brakeL;
Servo brakeR;
int pos = 0;
int retracted = 70; //value for brake retracted 
int engagedR = 10; //value for right brake engaged
int engagedL = 10; //value for left brake while engaged

unsigned long ifTime = 0;

int canPin = 3;//???
int rServoPin = 7;
int lServoPin = 6;
int hornPin = 8;

int wiggles = 30;
int wiggle = 0;

bool brake = false;

const int SPI_CS_PIN=10;

MCP_CAN CAN(SPI_CS_PIN);

unsigned long recID = 0x720;
unsigned long sndID = 0x666; //dvv dvv
unsigned long timeHold = 0;

void setup() {
  brakeL.attach(lServoPin);
  brakeR.attach(rServoPin);
  Serial.begin(115200);
  while(CAN_OK != CAN.begin(CAN_500KBPS)){
    delay(100);
  }
  CAN.init_Mask(0,0,0x7ff);
  CAN.init_Filt(0,0,0x720);
}

void canRead(){
  unsigned char len = 0;
  unsigned char buf[3];
  if(CAN_MSGAVAIL==CAN.checkReceive()){
    CAN.readMsgBuf(&len, buf);
    unsigned char id= CAN.getCanId();
    //Serial.println(id);
    if(id==recID){
      Serial.println(buf[0]);
      digitalWrite(hornPin, (buf[0]>0));
      if(buf[2]>0){
        wiggle=0;
      }else{
        wiggle=1;
      }
    }
  }
}

void loop() {
  if(digitalRead(canPin)==LOW){
    canRead();
  }
  bool elapsed = (millis()-ifTime)>200;
  if(elapsed && wiggle == 0){
    ifTime=millis();
    brakeL.write(retracted);
    brakeR.write(retracted);
  }else if(elapsed && wiggle <= wiggles){
    ifTime=millis();
    if(wiggle%2==0){
      brakeL.write(engagedR-5);
      brakeR.write(engagedL-5);
      wiggle++;
    }else{
      brakeL.write(engagedR);
      brakeR.write(engagedL);
      wiggle++;
    }
  }
}
