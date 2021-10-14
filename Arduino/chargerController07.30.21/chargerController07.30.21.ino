#include <mcp_can_dfs.h>
#include <mcp_can.h>


#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);

bool charge = false;
bool power = false;

int sendAddr0 = 0x714; //simulate wheel
int sendAddr1 = 0x700;  //simulate throttle
int sendAddr2 = 0x70e; //mid lights
unsigned char sendBuf0[5] = {0, 0x55, 0x00, 0, 0};
unsigned char sendBuf1[4] = {10, 0x55, 0x00, 10};
unsigned char sendBuf2[2] = {0x80, 0x55};

int relay = 3;
int canInt = A6;



void setup(){
  while (CAN_OK != CAN.begin(CAN_500KBPS)) delay(100);
  
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);

}

void canSend(){//Send button status
  sendBuf0[0] = 00; //regen value
  sendBuf0[1] = 0x55;
  //Assembeling GPIO in the form of ((boolValOfInterest&0b00000001)<<(8-desiredPosition))| [cont]
  sendBuf0[2] = 0;
  sendBuf0[3] = ((0b1)<<7);
  sendBuf0[4] = 0;
  //CAN.sendMsgBuf(sendAddr0, 0, 5, sendBuf0);
  CAN.sendMsgBuf(sendAddr1, 0, 4, sendBuf1);
  CAN.sendMsgBuf(sendAddr2, 0, 2, sendBuf2);
  delay(100);
}

void canRead(){
  unsigned char len = 0;
  unsigned char buf[8];

  if(CAN_MSGAVAIL == CAN.checkReceive()){ // check if data coming
    
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    int id = CAN.getCanId();
    switch (id){
      case 0x6b0:
        charge = buf[5]>2;
      case 0x716:
        if(id!=0x716) break;
        power = (buf[3]&0b10000000)>>7;//Aknowledge GPIO from computer, used to switch neopixels mainly.
    }
    
  }
  
}

void switchRelay(bool i){
  digitalWrite(relay, i);
}

void loop(){

  //CHECK CAN BUS
  if(analogRead(canInt)<512) canRead();

  //sWITCH RELAY
  switchRelay(power);
  
  //SEND DATA OVER CAN
  canSend();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();
}
