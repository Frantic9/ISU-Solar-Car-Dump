#include <mcp_can.h>
#include <mcp_can_dfs.h>

unsigned char stmp[4]={55, 55, 55,55};

const int SPI_CS_PIN = 10;
const int in1 = A0;  //oh shit
const int in2 = A2;  //brake
const int in3 = A1;  //throttle or A7
const int in4 = A3;  //regen
const int out1 = 4; //horn

const int canPin = 2;

const unsigned long recID = 0x720;
const unsigned long sendID = 0x700;

unsigned char len = 0;
unsigned char buf[8];

MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  //Serial.begin(115200);
  pinMode(in1, INPUT_PULLUP);
  pinMode(in2, INPUT_PULLUP);
  //pinMode(in3, OUTPUT);
  //pinMode(in4, OUTPUT); Analog doesn't need a pinMode
  pinMode(out1, OUTPUT);
  pinMode(canPin, INPUT_PULLUP);
  
  CAN.begin(CAN_500KBPS);
    
  for(int i = 0; i<10 || CAN_OK != CAN.begin(CAN_500KBPS); i++)
  {
     delay(100);
  }
  //Serial.println("CAN BUS Shield init OK!");
  
  CAN.init_Mask(0,0,0x7ff);

  CAN.init_Filt(0, 0, recID); //main recieving id
}

void canRead(){
  while (CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&len, buf);
    if (CAN.getCanId() == recID && buf[1] == 0x55){
      digitalWrite(out1,buf[0]);
    }
  }
}

void canSend(){ 
  stmp[0] = (map(analogRead(in3),0,1024,0,255));
  stmp[1] = 0x55;
  stmp[2] = digitalRead(in1);
  stmp[3] = digitalRead(in2);
  CAN.sendMsgBuf(sendID, 0, 4, stmp);
}

void loop() {
  if(!digitalRead(canPin)) canRead();
  canSend();
  if(!digitalRead(canPin)) canRead();
  delay(10);
}
