//#include <Adafruit_GPS.h>
#include <mcp_can_dfs.h>
#include <mcp_can.h>
//#include <SoftwareSerial.h>

unsigned char stmp[3]={7, 55};

//Hardware Serial - Telem
//D2/D3 - Tx/Rx GPS

const int SPI_CS_PIN = 10;
const int in1 = 4;  //StopButton
const int out2 = 5;  //Cam Power
const int out3 = 6;  //BMS
const int out4 = 7;  //BMS
const int out5 = 8;  //Third Brake
const int out6 = 9;  //Third Brake

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
const unsigned long sendID = 0x70e;

bool brk = 0;
bool bms = 0;
bool sent=false;

unsigned char buf[8];
unsigned char error[8]={0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00};
unsigned char error2[2]={0xff,0x55};
unsigned char len = 0;
unsigned char lastState = 0;

MCP_CAN CAN(SPI_CS_PIN);


void setup() {
  Serial.begin(57600);
  pinMode(in1, INPUT);
  pinMode(out2, OUTPUT);
  pinMode(out3, OUTPUT);
  pinMode(out4, OUTPUT);
  pinMode(out5, OUTPUT);
  pinMode(out6, OUTPUT);
  pinMode(canPin, INPUT_PULLUP);
  
  CAN.begin(CAN_500KBPS);

  delay(2000);
    
  for(int i = 0; i<10 || CAN_OK != CAN.begin(CAN_500KBPS); i++)
  {
     delay(100);
  }
  //Serial.println("CAN BUS Shield init OK!");
  
  CAN.init_Mask(0,0,0x000);
  CAN.init_Filt(0, 0, recID); //main recieving id

  digitalWrite(out2, HIGH);
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
    digitalWrite(out3, HIGH);
    digitalWrite(out4, HIGH);
    digitalWrite(out5, LOW);
    digitalWrite(out6, LOW);
    delay(100);
    digitalWrite(out2, LOW);
    digitalWrite(out3, LOW);
    digitalWrite(out4, LOW);
    digitalWrite(out5, HIGH);
    digitalWrite(out6, HIGH);
    delay(100);
    CAN.sendMsgBuf(sendID, 0, 8, error);
    if(digitalRead(canPin)==LOW){
      CAN.readMsgBuf(&len, buf);
      if(CAN.getCanId() == recID && buf[1] == 0x44){
        timRec = millis();
        return;
      }
    }
  }
  //Serial.println("chk com");
}

void canRead(){
  int id=0;
  while (CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&len, buf);
    id=CAN.getCanId();
    if (id == recID && buf[1] == 0x55){
      if(lastState != buf[0]) cycleCounter = 0;
      lastState = buf[0];
      bms = (buf[0] & 0b01000000) >> 6;
      brk = (buf[0] & 0b00100000) >> 5;
      timRec = millis();
      digitalWrite(out5, brk);
      digitalWrite(out6, brk);
    }else{
      Serial.print((id&0b11111111111), HEX);
      Serial.print(" ");
      Serial.println((char*)buf);
    }
  }
}

void canSend(){ 
  stmp[0] = (digitalRead(in1)<<7);
  CAN.sendMsgBuf(sendID, 0, 2, stmp);
  sendTime = millis();
}

void loop() {
  if(!digitalRead(canPin)) canRead();
  if(millis()-sendTime > sendInterval) canSend();
  if(!digitalRead(canPin)) canRead();
  //if(millis() > checkWait) checkCom();
  if(!digitalRead(canPin)) canRead();
  if(cycleCounter <= flashCycles){
    digitalWrite(out3, LOW);
    digitalWrite(out4, LOW);
    cycleCounter++;
  }else if(cycleCounter > flashCycles && cycleCounter < flashCycles*2){
    digitalWrite(out3, bms);
    digitalWrite(out4, bms);
    cycleCounter++;
  }else{
    cycleCounter = 0;
  }
  delay(cycleTime/4);
  if(!digitalRead(canPin)) canRead();
  delay(cycleTime/4);
  if(!digitalRead(canPin)) canRead();
  delay(cycleTime/4);
  if(!digitalRead(canPin)) canRead();
  delay(cycleTime/4);
}
