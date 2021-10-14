#include <mcp_can.h>
#include <mcp_can_dfs.h>

unsigned char stmp[3]={7, 55};

const int SPI_CS_PIN = 10;
const int rtTail = 3;
const int ltTail = 4;
const int trackers = A7;
const int canPin = 2;

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
const unsigned long sendID = 0x70c;

bool brk = 0;
bool lt = 0;
bool rt = 0;
bool pt = 0;

unsigned char buf[8];
unsigned char error[8]={0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00};
unsigned char error2[2]={0xff,0x55};
unsigned char len = 0;
unsigned char lastState = 0;

MCP_CAN CAN(SPI_CS_PIN);


void setup() {
  pinMode(rtTail, OUTPUT);
  pinMode(ltTail, OUTPUT);
  pinMode(trackers, OUTPUT);
  pinMode(canPin, INPUT_PULLUP);
    
  while(CAN_OK != CAN.begin(CAN_500KBPS)) delay(100);
    
  CAN.init_Mask(0,0,0x7ff);

  CAN.init_Filt(0, 0, 0x70a); //main recieving id
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
    digitalWrite(trackers, LOW);
    digitalWrite(rtTail, HIGH);
    digitalWrite(ltTail, LOW);
    delay(100);
    digitalWrite(rtTail, LOW);
    digitalWrite(ltTail, HIGH);
    delay(100);
    CAN.sendMsgBuf(sendID, 0, 8, error);
    if(!digitalRead(canPin)){
      CAN.readMsgBuf(&len, buf);
      if(CAN.getCanId() == recID && buf[1] == 0x44){
        timRec = millis();
        return;
      }
    }
  }
  Serial.println("chk com");
}

void canRead(){
  while (CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&len, buf);
    if (CAN.getCanId() == recID && buf[1] == 0x55){
      if(lastState != buf[0]) cycleCounter = 0;
      lastState = buf[0];
      pt = (buf[0] & 0b10000000) >> 7;
      brk = (buf[0] & 0b00100000) >> 5;
      lt = (buf[0] & 0b00010000) >> 4;
      rt = (buf[0] & 0b00001000) >> 3;
      timRec = millis();
      digitalWrite(trackers, pt);
    }
  }
}

void canSend(){ 
  stmp[0] = map(analogRead(A5), 0, 1023, 0, 255);
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
    digitalWrite(rtTail, (brk && (!rt))); //right back
    digitalWrite(ltTail, (brk && (!lt))); //left back
    cycleCounter++;
  }else if(cycleCounter > flashCycles && cycleCounter < flashCycles*2){
    digitalWrite(rtTail, (brk || rt)); //right back
    digitalWrite(ltTail, (brk || lt)); //left back
    cycleCounter++;
  }else{
    cycleCounter = 0;
  }
  delay(cycleTime);
}
