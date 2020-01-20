#include <ASTCanLib.h>
#include <Timer.h>

#define MESSAGE_ID        0u       // Message ID
#define MESSAGE_PROTOCOL  1u       // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8u       // Data length: 8 bytes
#define MESSAGE_RTR       0u       // rtr bit

#define NUMBER_OF_DIGIT   3u

#define SEGMENT_ON  LOW
#define SEGMENT_OFF HIGH
#define DISPLAY_BRIGHTNESS  500
#define DIGIT_ON  HIGH
#define DIGIT_OFF  LOW

// Function prototypes
void processButton();
void processCan();
void serialPrintData(st_cmd_t *msg);
void displayNumber(int toDisplay);
void displayValue();
void lightNumber(int numberToDisplay);

void getNMEA2Kdata(st_cmd_t *msg);

// CAN message object
st_cmd_t Msg;
// Buffer for CAN data
uint8_t Buffer[8] = {};

Timer t;
uint16_t Depth = 0; 
uint16_t Speed = 0;
uint16_t Temperature = 0;  
uint8_t paramSel = 0;

bool lossEcho = false;
bool dot = false;

// pin mapping 7seg
int digit1 = 5; //PWM Display pin 2, second digit
int digit2 = 6; //PWM Display pin 6, third digit
int digit3 = 7; //PWM Display pin 8, fourth digit
int segA = 10; //Display pin 14
int segB = 11; //Display pin 16
int segC = 12; //Display pin 13
int segD = 13; //Display pin 3
int segE = 14; //Display pin 5
int segF = 15; //Display pin 11
int segG = 16; //Display pin 15
int dp = 17; //Display pin 7

// pin mapping button
int buttonPin = 9;

void setup() 
{    
    // 7seg LEDs
    pinMode(segA, OUTPUT);  pinMode(segB, OUTPUT);  pinMode(segC, OUTPUT);  pinMode(segD, OUTPUT);  
    pinMode(segE, OUTPUT);  pinMode(segF, OUTPUT);  pinMode(segG, OUTPUT);  
    pinMode(digit1, OUTPUT);  pinMode(digit2, OUTPUT);  pinMode(digit3, OUTPUT);
    pinMode(dp, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    
    // CAN
    canInit(250000u);            // Initialise CAN port 250kbps. must be before Serial.begin
    Serial.begin(1000000u);       // start serial port 1Mbps
    Msg.pt_data = &Buffer[0];    // reference message data to buffer
    
    // Initialise CAN packet.
    // All of these will be overwritten by a received packet
    Msg.ctrl.ide = MESSAGE_PROTOCOL;  // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
    Msg.id.ext   = MESSAGE_ID;        // Set message ID
    Msg.dlc      = MESSAGE_LENGTH;    // Data length: 8 bytes
    Msg.ctrl.rtr = MESSAGE_RTR;       // Set rtr bit

    //attachInterrupt(digitalPinToInterrupt(buttonPin), processButton, FALLING);
  
    int tickEvent = t.every(15, displayValue, 0);
  
    displayNumber(Depth); // test LCD segments, counter 0-9
    delay(10);
    for(int i=0; i<10; i++)
    {
      dot = !dot;
      for(int j=0; j<10; j++)
      {
        delay(10);
        displayNumber(i*111);
      }
    }
    delay(10);
    displayNumber(Depth);
}

void loop() 
{
    t.update();
    if(digitalRead(buttonPin) == LOW)
    {
        processButton();
    }
    processCan(); 
}


void displayValue()
{
  //displayNumber(millis() / 1000);
  if(paramSel == 0)
  {
    displayNumber(Depth);
  }
  else if(paramSel == 1)
  {
    displayNumber(Speed);
  }
  else if(paramSel == 2)
  {
    displayNumber(Temperature);
  }
  else displayNumber(0);
}

void processCan()
{
  clearBuffer(&Buffer[0]);  // Clear the message buffer
  Msg.cmd = CMD_RX_DATA;  // Send command to the CAN port controller

  // Wait for the command to be accepted by the controller
  while (can_cmd(&Msg) != CAN_CMD_ACCEPTED)
  {
      t.update();
  }
  // Wait for command to finish executing
  while (can_get_status(&Msg) == CAN_STATUS_NOT_COMPLETED)
  {
      t.update();
  }

  getNMEA2Kdata(&Msg);
  //serialPrintData(&Msg);

}

void getNMEA2Kdata(st_cmd_t *msg)
{
  if ( (uint16_t)msg->id.ext == 0x0B23)  // depth 
  {
      Serial.print("NMEA2k Depth: 0x0B23 # ");    
      uint32_t Du32 = (uint32_t)( ((uint32_t)Msg.pt_data[4] << 24) | ((uint32_t)Msg.pt_data[3] << 16) |
                                  ((uint32_t)Msg.pt_data[2] << 8) | (uint32_t)Msg.pt_data[1] );
      Serial.println(Du32);
      if (Du32 == 0xFFFFFFFF) // DST800 no echo
      {
          dot = false;
          lossEcho = true;
          Depth = 888;
      }
      else
      {
          double Dd = (double)Du32 * 0.1; // depth in decimeter, with a dot before the last digit display is in meter
          Dd = round(Dd);
          Depth = (uint16_t)Dd;
          if(Depth >= 1000) // DST800 is 100m=1000dm max
          {
            dot = false;
            Depth = 999;
          }
          else
          {
            dot = true;
          }
          lossEcho = false;
      }
  }
  else if ( (uint16_t)msg->id.ext == 0x0323) // speed
  {
      Serial.print("NMEA2k Speed: 0x323 # ");
      uint16_t Su16 = (uint16_t)( ((uint16_t)Msg.pt_data[2] << 8) | Msg.pt_data[1] );
      Serial.println(Su16);
      double Sd = (double)Su16 *0.01; // speed in m/s
      Sd = Sd *10; // speed in decimeter/s
      Speed = (uint16_t)Sd;
  }
  else if ( (uint16_t)msg->id.ext == 0x0723) // water temp
  {
      Serial.print("NMEA2k Water Temp: 0x723 # ");
      uint16_t Tu16 = (uint16_t)( ((uint16_t)Msg.pt_data[3] << 8) | Msg.pt_data[2] );
      Serial.println(Tu16); // in degK
      double Td = (double)Tu16 * 0.01 - 273.15; // temp in degC
      Td = Td *10; // temperature in deciCelcius
      Temperature = (uint16_t)Td;
  }
  else 
  {
    char textBuffer[50] = {0};
    sprintf(textBuffer, "NMEA2k: unmanaged PGN IDext=%d\r\n", msg->id.ext);
    Serial.print(textBuffer);
  }
  // ... others NMEA2K message
}


void processButton()
{
  delay(250);
  if(digitalRead(buttonPin) == LOW)
  {
    Serial.println("Button pressed");
    paramSel = paramSel +1;
    if(paramSel == 3)
    {
      paramSel =0;
    }
  }
}


void serialPrintData(st_cmd_t *msg) {
  char textBuffer[50] = {0};
  if (msg->ctrl.ide > 0) 
  {
    sprintf(textBuffer, "id ext %08x ", msg->id.ext);
  }
  else
  {
    sprintf(textBuffer, "id std %04x ", msg->id.std);
  }
  Serial.print(textBuffer);

  //  IDE
  sprintf(textBuffer, "ide %d ", msg->ctrl.ide);
  Serial.print(textBuffer);
  //  RTR
  sprintf(textBuffer, "rtr %d ", msg->ctrl.rtr);
  Serial.print(textBuffer);
  //  DLC
  sprintf(textBuffer, "dlc %d ", msg->dlc);
  Serial.print(textBuffer);
  //  Data
  sprintf(textBuffer, "data ");
  Serial.print(textBuffer);

  for (int i = 0; i < msg->dlc; i++) {
    sprintf(textBuffer, "%02X ", msg->pt_data[i]);
    Serial.print(textBuffer);
  }
  Serial.print("\r\n");
}


void displayNumber(int toDisplay) {
  long beginTime = millis();

  for (int digit = NUMBER_OF_DIGIT ; digit > 0 ; digit--) 
  {
    //Turn on a digit for a short amount of time
    switch (digit) 
    {
      case 1:
        digitalWrite(digit1, DIGIT_ON);
        digitalWrite(dp, SEGMENT_OFF);
        break;
      case 2:
        digitalWrite(digit2, DIGIT_ON);
        if(dot == true) 
          digitalWrite(dp, SEGMENT_ON);
        else 
          digitalWrite(dp, SEGMENT_OFF);
        break;
      case 3:
        digitalWrite(digit3, DIGIT_ON);
        digitalWrite(dp, SEGMENT_OFF);
        break;
    }
    //Turn on the right segments for this digit
    lightNumber(toDisplay % 10);
    toDisplay /= 10;
    delayMicroseconds(DISPLAY_BRIGHTNESS); //Display this digit for a fraction of a second (between 1us and 5000us, 500 is pretty good)
    //Turn off all segments
    lightNumber(10);
    //Turn off all digits
    digitalWrite(digit1, DIGIT_OFF);
    digitalWrite(digit2, DIGIT_OFF);
    digitalWrite(dp, SEGMENT_OFF);
    digitalWrite(digit3, DIGIT_OFF);
  }
  while ( (millis() - beginTime) < 10) ; //Wait for 20ms to pass before we paint the display again
}

//Given a number, turns on those segments
//If number == 10, then turn off number
void lightNumber(int numberToDisplay) 
{
  switch (numberToDisplay) 
  {
    case 0:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_OFF);
      break;
    case 1:
      digitalWrite(segA, SEGMENT_OFF);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_OFF);
      break;
    case 2:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_OFF);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 3:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 4:
      digitalWrite(segA, SEGMENT_OFF);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 5:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_OFF);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 6:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_OFF);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 7:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_OFF);
      break;
    case 8:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 9:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 10:
      digitalWrite(segA, SEGMENT_OFF);
      digitalWrite(segB, SEGMENT_OFF);
      digitalWrite(segC, SEGMENT_OFF);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_OFF);
      break;
  }
}
