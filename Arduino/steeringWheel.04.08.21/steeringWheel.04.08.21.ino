/*ADDED FEATURES:
  Added blue LED when requested state and reported state don't match
  Added lcd brightness adjustment to joystick up/down long press
  Parsed incoming CAN messages/LCD positions:
  Highest Battery Voltage Per Cell(0,0,0), Speed(0,1,0), SOC(0,2,0), Bus Current (0,3,0), Battery Module Temp(0,4,0)
  Lowest Battery Voltage Per Cell(1,0,0), Cruise Speed(1,1,0), Cruise State (1,2,0), Aux Voltage %(1,3,0), Array Total Power(1,4,0)
  Relay State (0,0,1), Array1Power(0,1,1), Array2Power(0,2,1), Array3Power(0,3,1), Array4Power(0,4,1)
*/

#include <mcp_can.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include <Adafruit_NeoPixel.h>

#define DATA_NUM 5
#define PIN 8
#define NUMPIXELS 10

const int SPI_CS_PIN = 10;

LiquidCrystal lcd(A2,A1,A5,A4,A3,A0);
MCP_CAN CAN(SPI_CS_PIN);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int longPressCycles=50;
int screen=0;
const int screens=2; //One screen just used as a buffer, not displayed
int dataList[2][DATA_NUM][screens];//={{{0,0},{0,0},{0,0},{0,0},{0,0}},{{0,0},{0,0},{0,0},{0,0},{0,0}}};//Essentially a screen buffer that stores all the data to be displayed
//int dataList[2][DATA_NUM][screens]={{{325,0},{45,0},{18,0},{22,0},{425,0}},{{324,0},{45,0},{0,0},{10,0},{800,0}}}; //values for troubleshooting

int buttonArray[3][5] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};//stores how many cycles the button is pushed
bool longPress[3][5] = {{false,false,false,false,false},{false,false,false,false,false},{false,false,false,false,false}};

bool rightTurn = false;
bool leftTurn = false;
bool drl = false;
//bool horn = false;
bool regen = false;
bool trackers = false;
bool power = false;
bool powerError = false;

bool rightTurnReq = false;
bool leftTurnReq = false;
bool drlReq = false;
bool hornReq = false;
bool regenReq = false;
bool trackersReq = false;
bool powerReq = false;
bool hazardReq = false;

unsigned char gear=0;
unsigned char gearReq=0;
char errorCode = 0;
int targetSpeed = 0;
int targetSpeedReq = 0;
int lastTargetSpeed = 0;
int reportedSpeed = 0;

int brightness = 20; //neopixel base brightness.  0-255
int screenBright = 50; //screen brightness 0-255

int canInt = A6; //analog in only, must use analogRead(canInterrupt)<512 instead of digitalRead.
int regenPot = A7;

int motResetAddr = 0x503;
int sendAddr = 0x714;
unsigned char sendBuf[5] = {0, 0, 0, 0, 0};
unsigned char motReset[8]={0,0,0,0,0,0,0,0};
unsigned char dataOut[8]={0,0,0,0,0,0,0,0};

int arrayPowerList[6] = {0, 0, 0, 0, 0, 0};

int cycleTime = 75;
int delays = 5;
unsigned long sendTime = 0;
unsigned long sendDelay = 80;

void setup(){
  while (CAN_OK != CAN.begin(CAN_500KBPS)) delay(100);

  for(int i=0;i<3;i++) pinMode(i, OUTPUT);
  for(int j=0;j<5;j++) pinMode(j+3, INPUT_PULLUP);
  for (int i=0;i<3;i++) digitalWrite(i, HIGH);
  
  lcd.begin(16,2);
  lcd.print("Hello World");

  pixels.begin();
  
  analogWrite(9,screenBright);
}

float msgToFloat(uint8_t buf[8], char val){//send the whole message buffer to this function and select first or second half with val.  val=0 first, val=1 second
  float a;
  ((uint8_t*)&a)[0] = buf[val*4];
  ((uint8_t*)&a)[1] = buf[1+val*4];
  ((uint8_t*)&a)[2] = buf[2+val*4];
  ((uint8_t*)&a)[3] = buf[3+val*4];
  
  return a;
}

unsigned int msgToShort(unsigned char buf[8], int startByte){//send the whole message buffer to this function and select first byte of two byte value
  unsigned int a;
  ((uint8_t*)&a)[0] = buf[1+startByte];
  ((uint8_t*)&a)[1] = buf[startByte];
  return a;
  //return buf[startByte] | (buf[startByte+1]<<8);
}

unsigned int twoBytesToShort(unsigned char buf[2]){
  return (buf[0]<<8)| buf[1];
}

void arrayList(int i, int power){
  int powerSum = 0;
  arrayPowerList[i]=power;
  for(int j = 0; j<6; j++) powerSum+=arrayPowerList[j];
  dataList[1][4][0] = powerSum;
}

void canSend(){//Send button status
  int regenVal = analogRead(regenPot);
  sendBuf[0] = (char)map(regenVal, 0, 1024, 0, 255); //regen value
  sendBuf[1] = 0x55;
  //Assembeling GPIO in the form of ((boolValOfInterest&0b00000001)<<(8-desiredPosition))| [cont]
  sendBuf[2] = ((drlReq&0b1)<<7)|(((rightTurnReq|hazardReq)&0b1)<<6)|(((leftTurnReq|hazardReq)&0b1)<<5)|((hornReq&0b1)<<4)|((regenReq&0b1)<<3)|((trackersReq&0b1)<<2)|(gearReq&0b11);
  sendBuf[3] = ((powerReq&0b1)<<7);
  sendBuf[4] = (char)targetSpeedReq;
  CAN.sendMsgBuf(sendAddr, 0, 5, sendBuf);
  sendTime = millis();
}

void canRead(){//read telem values from pi or bus, watch for if cruise is allowed
  unsigned char len = 0;
  unsigned char buf[8];

  if(CAN_MSGAVAIL == CAN.checkReceive()){ // check if data coming
    
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    int id = CAN.getCanId();
    switch (id){
      case 0x402:
        dataList[0][3][0] = (int) msgToFloat(buf, 1);  //DC bus current from MC
        break;
      case 0x403:
        reportedSpeed = (int) (2.23694*msgToFloat(buf, 1)); //Speed converted to mph from MC
        dataList[0][1][0] =  reportedSpeed;
        break;
      case 0x6b0:
        dataList[0][2][0] = (int) buf[4]/2; //State of Charge (SOC)
        dataList[0][0][1] = (int) buf[5];
      case 0x6b1:
        if(id==0x6b1){
          if(buf[4]>20) dataList[0][4][0] = (int)buf[4]; //highest battery module temperature
          else dataList[0][4][0] = (int)buf[5]; //lowest battery module temp.  if the battery is above 20c show highest temp, else show lowest temp
        }
        break;
      case 0x6b2:
        dataList[0][0][0] = msgToShort(buf,0)/100; //High module voltage *1000... /100 so that the result is a 3 digit number
        dataList[1][0][0] = msgToShort(buf,2)/100; //Low module voltage *1000
        break;
      case 0x600:
        arrayList(0,(msgToShort(buf,0)*msgToShort(buf,2))/1000); //removed 3 0s
        break;
      case 0x601:
        arrayList(1,(msgToShort(buf,0)*msgToShort(buf,2))/1000);
        break;
      case 0x602:
        arrayList(2,(msgToShort(buf,0)*msgToShort(buf,2))/1000);
        break;
      case 0x603:
        arrayList(3,(msgToShort(buf,0)*msgToShort(buf,2))/1000);
        break;
      case 0x604:
        arrayList(4,(msgToShort(buf,0)*msgToShort(buf,2))/1000);
        break;
      case 0x605:
        arrayList(5,(msgToShort(buf,0)*msgToShort(buf,2))/1000);
        break;
      case 0x716:
        if(id!=0x716) break;
        drl = (buf[2]&0b10000000)>>7;
        rightTurn = (buf[2]&0b01000000)>>6;
        leftTurn = (buf[2]&0b00100000)>>5;
        regen = (buf[2]&0b00001000)>>3;
        trackers = (buf[2]&0b00000100)>>2;
        gear = buf[2]&0b00000011;
        power = (buf[3]&0b10000000)>>7;//Aknowledge GPIO from computer, used to switch neopixels mainly.
        powerError = (buf[3] & 0b01000000) >> 6; //Forces POWER ERROR display
        dataList[1][3][0] = (int) buf[0];
        errorCode = buf[5];
        targetSpeed = (int)buf[4];
        if(targetSpeed == 0){
          dataList[1][1][0] = lastTargetSpeed;
          targetSpeedReq = 0;
        }else{
          dataList[1][1][0] = targetSpeed;
        }
        break;
    }
    
  }
  
}

void buttons(int i, int j, int cycles){
  if(cycles==0)return;
  if(i==0){
    if(j==0){//Joystick Up - DRL
      if(cycles<longPressCycles){
        drlReq=!drlReq;
        return;
      }else if(cycles>=longPressCycles){
        if(screenBright<=250){
          screenBright+=5;
          analogWrite(9,screenBright);
        }
        return;
      }
      return;
    }else if(j==1){//Joystick Right - R/T
      rightTurnReq = !rightTurnReq;
      return;
    }else if(j==2){//Joystick Down
      if(cycles<longPressCycles){
        return;
      }else if(cycles>=longPressCycles){
        if(screenBright>=5){
          screenBright-=5;
          analogWrite(9,screenBright);
        }
        return;
      }
      return;
    }else if(j==3){//Joystick Push - Hazards
      hazardReq=!hazardReq;
      return;
    }else if(j==4){//Joystick Left - L/T
      leftTurnReq = !leftTurnReq;
      return;
    }
  }else if(i==1){
    if(j==0){//Horn - Should be handled elsewhere
      return;
    }else if(j==1){//Press: Gear - Hold: MC Reset
      if(cycles<longPressCycles){
        gearReq=(gearReq+1)%4; //0bxxxxxx00, 0bxxxxxx10=N; 0bxxxxxx01=F; 0bxxxxxx11=R
        return;
      }else if(cycles>=longPressCycles){
        CAN.sendMsgBuf(0x503, 0, 8, motReset); //ofload mot reset from ras pi
        return;
      }
    }else if(j==2){//Resume/Accelerate Cruise Control
      if(targetSpeed==0){
        targetSpeedReq = lastTargetSpeed;
      }else{
        targetSpeedReq+=2;
        lastTargetSpeed=targetSpeedReq;
      }
      return;
    }else if(j==3){//Set/Decelerate Cruise Control
      if(targetSpeed == 0){
        targetSpeedReq=reportedSpeed;
        return;
      }else{
        targetSpeedReq-=2;
        lastTargetSpeed=targetSpeedReq;
        return;
      }
    }else if(j==4){//N/A
      return;
    }
  }else if(i==2){
    if(j==0){//Regen
      regenReq = !regenReq;
      return;
    }else if(j==1){//Power Tracker Power
      trackersReq = !trackersReq;
      return;
    }else if(j==2){//N/A
      return;
    }else if(j==3){//Cruise Cancel
      targetSpeedReq = 0;
      return;
    }else if(j==4){//On Switch - should be handeled elsewhere
      return;
    }
  }else{
    return; //wrong i supplied
  }
  
}

void readButtons(){ //READ MULTIPLEXED BUTTONS
  for(int i=0;i<3;i++){
    digitalWrite(i, LOW);
    for(int j=0;j<5;j++){
      if(i==1 && j==0){
        hornReq = !digitalRead(j+3);
      }else if(i==2 && j==4){
        powerReq = !digitalRead(j+3);
      }else if(!digitalRead(j+3)){
        buttonArray[i][j]++;
        if(buttonArray[i][j]>longPressCycles){
          buttons(i,j, buttonArray[i][j]);
          buttonArray[i][j]=0;
          longPress[i][j] = true;
        }
      }else{
        if(buttonArray[i][j]>0){ //Add functionality so it doesn't do the short press op after the long press
          if(!longPress[i][j]) buttons(i,j, buttonArray[i][j]);
          else longPress[i][j] = false;
          buttonArray[i][j]=0;
        }
      }
    }
    digitalWrite(i, HIGH);
  }
}

void showScreen(){//Print to char LCD screen
  if(powerError){
    lcd.setCursor(0,0);
    lcd.print("POWER ERROR #");
    lcd.print(errorCode);
    lcd.print("    ");
    lcd.setCursor(0,1);
    lcd.print("RESTART REQUIRED");
  }else{
    for(int i = 0; i<2; i++){
      lcd.setCursor(0,i);
      for(int j = 0; j<DATA_NUM;j++){
        //if (dataList[i,j,screen]==0) lcd.print("0");
        //if (dataList[i][j][screen]<100) lcd.print("0");
        if(i==1 && j==2){
          if(targetSpeed==0) lcd.print("OFF ");
          else lcd.print("ON ");
        //}else if(i==0 && j==4){
        //  lcd.print("sh ");
        }else{
          if (dataList[i][j][screen]<10) lcd.print("0");
          lcd.print(dataList[i][j][screen]);
          lcd.print(" ");
        }
      }
      //lcd.print(" ");
    }
  }
}

void lightPixels(){//LIGHT NEOPIXELS - Order: Regen, RT, DRL, LT, Cruise Cancel, PT, POWER, GEAR, Horn, Cruise
  pixels.setPixelColor(0, pixels.Color((brightness-(brightness*regen)), brightness*regen, brightness*(regenReq!=regen)));
  pixels.setPixelColor(1, pixels.Color(0, brightness*rightTurn, brightness*(rightTurn!=rightTurnReq)));
  pixels.setPixelColor(2, pixels.Color(0, brightness*drl, brightness*(drl!=drlReq)));
  pixels.setPixelColor(3, pixels.Color(0, brightness*leftTurn, brightness*(leftTurn!=leftTurnReq)));
  pixels.setPixelColor(5, pixels.Color((brightness-(brightness*trackers)),brightness*trackers,brightness*(trackers!=trackersReq)));
  pixels.setPixelColor(6, pixels.Color(brightness-(brightness*power),brightness*power,brightness*(power!=powerReq)));
  if(gearReq==3) pixels.setPixelColor(7, pixels.Color(brightness,0,brightness*(gearReq!=gear)));
  else if(gearReq==1) pixels.setPixelColor(7, pixels.Color(0,0,brightness*(gearReq!=gear)));
  else pixels.setPixelColor(7, pixels.Color(0,brightness,brightness*(gearReq!=gear)));
  
  pixels.show();  
}

void loop(){

  //CHECK CAN BUS
  if(analogRead(canInt)<512) canRead();
  
  //SEND DATA OVER CAN
  if((millis()-sendTime) >= sendDelay) canSend();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();
  
  //READ MULTIPLEXED BUTTONS
  readButtons();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();
  
  //DISPLAY DATA ON SCREEN - Screen variable toggles which set of data is displayed
  showScreen();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();
  
  //LIGHT NEOPIXELS
  lightPixels();

  //CHECK CAN BUS AGAIN
  if(analogRead(canInt)<512) canRead();
}
