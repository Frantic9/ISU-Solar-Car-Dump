#include <mcp_can.h>
#include <mcp_can_dfs.h>

unsigned char stmp[3]={7, 55};

const int SPI_CS_PIN = 10;
const int out1 = 4;  //Right turn
const int out2 = 5;  //Right turn
const int out3 = 6;  //drl
const int out4 = 7;  //drl
const int out5 = 8;  //left turn
const int out6 = 9;  //left turn

const int canPin = A0;

const int cycleTime = 20;
const int flashPeriod = 500;
const int flashCycles = flashPeriod/cycleTime;
int cycleCounter = 0;

unsigned long timeHold = 0;
unsigned long timRec = 0;
unsigned long checkWait = 90000;
unsigned long sendTime = 0;
unsigned long sendInterval = 250;

const unsigned long recID = 0x70a;
const unsigned long sendID = 0x70d;

bool drl = 0;
bool rt = 0;
bool lt = 0;

unsigned char buf[8];
unsigned char error[8]={0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00};
unsigned char error2[2]={0xff,0x55};
unsigned char len = 0;
unsigned char lastState = 0;

MCP_CAN CAN(SPI_CS_PIN);


void setup() {
  Serial.begin(115200);
  pinMode(out1, OUTPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  pinMode(out4, OUTPUT);
  pinMode(out5, OUTPUT);
  pinMode(out6, OUTPUT);
  pinMode(canPin, INPUT_PULLUP);
    
  for(int i = 0; i<10 || CAN_OK != CAN.begin(CAN_500KBPS); i++)
  {
     delay(100);
  }
  //Serial.println("CAN BUS Shield init OK!");
  
  CAN.init_Mask(0,0,0x7ff);

  CAN.init_Filt(0, 0, recID); //main recieving id
}

void checkCom()
{
  if((millis()-timRec)>550){
    if(!digitalRead(canPin)){
      canRead();
      if(digitalRead(canPin)){
        CAN.sendMsgBuf(sendID,0,2,error2);
        timRec = millis();
        return;
      }else{
        //noBueno
      }
    }
  }
  while((millis()-timRec) > 550){ //shut down if we don't get a message every 550 ms
    digitalWrite(out2, LOW);
    //=-digitalWrite(out3, HIGH);
    //digitalWrite(out4, HIGH);
    digitalWrite(out5, LOW);
    digitalWrite(out6, LOW);
    delay(100);
    digitalWrite(out2, LOW);
    //digitalWrite(out3, LOW);
    //digitalWrite(out4, LOW);
    digitalWrite(out5, HIGH);
    digitalWrite(out6, HIGH);
    delay(100);
    CAN.sendMsgBuf(sendID, 0, 8, error);
    if(digitalRead(canPin)==LOW){
      CAN.readMsgBuf(&len, buf);
      if(CAN.getCanId() == recID && buf[1] == 0x44){
        timRec = millis();
        break;
      }
    }
  }
  //Serial.println("chk com");
}

void canRead(){
  while (CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&len, buf);
    if (CAN.getCanId() == recID && buf[1] == 0x55){
      if(lastState != buf[0]) cycleCounter = 0;
      lastState = buf[0];
      drl = (buf[0] & 0b00000100) >> 2;
      rt = (buf[0] & 0b00001000) >> 3;
      lt = (buf[0] & 0b00010000) >> 4;
      timRec = millis();
      digitalWrite(out3, drl);
      digitalWrite(out4, drl);
    }
  }
}

void canSend(){ 
  stmp[0] = 0x42;
  CAN.sendMsgBuf(sendID, 0, 2, stmp);
  sendTime = millis();
}

void loop() {
  if(!digitalRead(canPin)) canRead();
  if(millis()-sendTime > sendInterval) canSend();
  if(!digitalRead(canPin)) canRead();
  if(millis() > checkWait) checkCom();
  if(!digitalRead(canPin)) canRead();
  if(cycleCounter <= flashCycles){
    digitalWrite(out1, LOW); //right front
    digitalWrite(out2, LOW); //right front
    digitalWrite(out5, LOW); //left front
    digitalWrite(out6, LOW); //left front
    cycleCounter++;
  }else if(cycleCounter > flashCycles && cycleCounter < flashCycles*2){
    digitalWrite(out1, rt); //right front
    digitalWrite(out2, rt); //right front
    digitalWrite(out5, lt); //left front
    digitalWrite(out6, lt); //left front
    cycleCounter++;
  }else{
    cycleCounter = 0;
  }
  delay(cycleTime);
}
