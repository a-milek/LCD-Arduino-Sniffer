#define E_PIN 3
#define RW_PIN 4
#define RS_PIN 2

#include <ctype.h>

uint8_t counter = 0;
static uint8_t databuff = 0;
static uint8_t commandbuff = 0;
uint8_t ddram[128];
uint8_t ddram_ac;
bool ddram_change = false;

enum State { R,
             DH,
             DL,
             CH,
             CL,
             D8,
             C8 };
static State currentState = R;
bool mode_8bit = true;

void execCommand(uint8_t command) {
  // Serial.print("Command: ");
  // Serial.println(command, HEX);

  if ((command & 0b10000000) != 0) {
    ddram_ac = command & 0b01111111;
    // Serial.println(ddram_ac, HEX);
  } else if ((command & 0b01000000) != 0) {
    //dont care cgram address
  } else if ((command & 0b00100000) != 0) {
    //dont care, fuction set
  } else if ((command & 0b00010000) != 0) {
    //tbd display cursor shift
  } else if ((command & 0b00001000) != 0) {
    //display on off, dont care
  } else if ((command & 0b00000100) != 0) {
    //tbd entry mode set
  } else if ((command & 0b00000010) != 0) {
    ddram_ac = 0;
  } else if ((command & 0b00000001) != 0) {
    memset(ddram, ' ', sizeof(ddram));
    ddram_ac = 0;
  }
}
void execData(uint8_t data) {
  Serial.print((char)data);
  // Serial.print(ddram_ac);

  // // Serial.print("Data: ");
  // if (isprint(data)) {
  //   // Serial.println((char)data);
  // } else {
  //   // Serial.print("hex:");
  //   // Serial.println(data, HEX);
  // }

  ddram[ddram_ac] = data;
  if (ddram_ac < sizeof(ddram)) {
    ddram_ac++;
  } else {
    // Serial.print("ERR");
  }
  ddram_change = true;
  // printDdram();
}

void printDdram() {
  Serial.println("LCD: ");
  for (uint8_t i = 0; i < sizeof(ddram); i++) {
    Serial.print((char)ddram[i]);
  }
  Serial.println();
}


void Control(uint8_t pinSnapshot, uint8_t rs_pin) {
  // Serial.println();
  uint8_t buff = pinSnapshot;
  bool retry = true;

  

    // ddram_change = false;
    // Serial.print("S:");
    // Serial.println(currentState);
    switch (currentState) {
      case R:
        databuff = 0;
        commandbuff = 0;
        //Serial.print(rs_pin);
        if (rs_pin) {
          currentState = mode_8bit ? D8 : DH;
          // continue;

        } else {
          currentState = mode_8bit ? C8 : CH;
          // continue;
        }
        break;

      case DH:
        if (rs_pin) {  // Data high byte
          databuff = (buff << 4);
          // Serial.print("DH: ");
          // Serial.println(buff, BIN);
          currentState = DL;
        } else {
          currentState = R;
          // continue;
        }

        break;

      case DL:  // Data low byte
        if (rs_pin) {
          databuff |= buff;
          currentState = R;
          // Serial.print("DL: ");
          // Serial.println(buff, BIN);

          execData(databuff);

        } else {
          currentState = R;
          // continue;
        }

        break;

      case CH:  // Command high byte
        if (rs_pin == 0) {
          commandbuff = (buff << 4);
          currentState = CL;
        } else {
          currentState = R;
          // continue;
        }
        break;

      case CL:  // Command low byte
        if (rs_pin == 0) {
          commandbuff |= buff;
          currentState = R;
          execCommand(commandbuff);
        } else {
          currentState = R;
          // continue;
        }

        break;

      case C8:
        if (rs_pin == 0) {
          commandbuff = buff;
          // Serial.print(buff);
          execCommand(commandbuff);
          currentState = R;
        } else {
          // Serial.print("C8 Err");
          currentState = R;
          // continue;
        }
        break;

      case D8:
        if (rs_pin) {  // Data high byte
          databuff = buff;
          currentState = R;
          execData(databuff);

        } else {

          // Serial.print("D8 Err");
          currentState = R;
          // continue;
        }

        break;
    }
    retry = false;
  
}



void readPinsISR() {
  // Serial.println("co jest");

  uint8_t pinB = PINB;
  uint8_t pinD = PIND;
  uint8_t rs_pin = pinD & (1 << 2);
#define BITMAP(pin, s, d) (((pin & (1 << s)) >> s) << d)

  PORTC |= (1 << 0);
  uint8_t a =
    BITMAP(pinD, 5, 0) |  // DB0 → bit 0
    BITMAP(pinD, 6, 1) |  // DB1 → bit 1
    BITMAP(pinD, 7, 2) |  // DB2 → bit 2
    BITMAP(pinB, 0, 3) |  // DB3 → bit 3
    BITMAP(pinB, 1, 4) |  // DB4 → bit 4
    BITMAP(pinB, 2, 5) |  // DB5 → bit 5
    BITMAP(pinB, 3, 6) |  // DB6 → bit 6
    BITMAP(pinB, 4, 7) |  // DB7 → bit 7
    0;

  //pinSnapshot = (PIND & 0b11100000) | (PINB & 0b00011111);  // Read full PINB


  Control(a, rs_pin);
  PORTC &= ~(1 << 0);
  //0output = 0
}

void setup() {

  DDRC |= (1 << 0);
  // PORTC |= (1 << 0);
  // delay(10);
  // PORTC &= ~(1 << 0);

  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(E_PIN), readPinsISR, FALLING);
  memset(ddram, ' ', sizeof(ddram));
}

void loop() {

  if (ddram_change) {
    // printDdram();
    ddram_change = false;
  }
}