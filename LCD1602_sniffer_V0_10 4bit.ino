#define E_PIN 3
#define RW_PIN 4
#define RS_PIN 2


#include <ctype.h>

volatile uint8_t pinSnapshot = 0;
volatile bool pinReadReady = false;
uint8_t counter = 0;

static uint8_t databuff = 0;
static uint8_t commandbuff = 0;

uint8_t ddram[128];
uint8_t ddram_ac;
enum State { R,
             DH,
             DL,
             CH,
             CL };
static State currentState = R;


void execCommand(uint8_t command) {
  Serial.print("Command: ");
  Serial.println(command, HEX);

  if ((command & 0b10000000) != 0) {
    ddram_ac = command & 0b01111111;
    Serial.println(ddram_ac, HEX);
  } else if ((command & 0b01000000) != 0) {
    //dont care cgram address
    Serial.println('X');
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
  } else if ((command & 0b00000001)!= 0) {
    memset(ddram, ' ', sizeof(ddram));
    ddram_ac = 0;
  }
}
void execData(uint8_t data) {
  Serial.print("Data: ");
  if (isprint(data)) {
    Serial.println((char)data);
  } else {
    Serial.println(data, HEX);
  }

  ddram[ddram_ac] = data;
  if (ddram_ac < sizeof(ddram)) {
    ddram_ac++;
  }
  printDdram();
}

void printDdram() {
  for (uint8_t i = 0; i < 0x27; i++) {
    Serial.print((char)ddram[i]);
  }
  Serial.println();
  for (uint8_t i = 0x40; i < 0x67; i++) {
    Serial.print((char)ddram[i]);
  }
  Serial.println();
}

void Control() {
  Serial.println();
  uint8_t buff = pinSnapshot & 0x0F;
  bool retry = true;

  do {
    // Serial.print("S:");
    // Serial.println(currentState);
    switch (currentState) {
      case R:
        databuff = 0;
        commandbuff = 0;
        if (digitalRead(RS_PIN) == HIGH) {
          currentState = DH;
          continue;

        } else {
          currentState = CH;
          continue;
        }
        break;

      case DH:
        if (digitalRead(RS_PIN) == HIGH) {  // Data high byte
          databuff = (buff << 4);
          // Serial.print("DH: ");
          // Serial.println(buff, BIN);
          currentState = DL;
        } else {
          currentState = R;
          continue;
        }

        break;

      case DL:  // Data low byte
        if (digitalRead(RS_PIN) == HIGH) {
          databuff |= buff;
          currentState = R;
          // Serial.print("DL: ");
          // Serial.println(buff, BIN);

          execData(databuff);

        } else {
          currentState = R;
          continue;
        }

        break;

      case CH:  // Command high byte
        if (digitalRead(RS_PIN) == LOW) {
          commandbuff = (buff << 4);
          currentState = CL;
        } else {
          currentState = R;
          continue;
        }
        break;

      case CL:  // Command low byte
        if (digitalRead(RS_PIN) == LOW) {
          commandbuff |= buff;
          currentState = R;
          execCommand(commandbuff);
        } else {
          currentState = R;
          continue;
        }

        break;
    }
    retry = false;
  } while (retry);
}


void readPinsISR() {
  pinSnapshot = PINB;  // Read full PINB
  pinReadReady = true;
  Control();  // Set flag for loop to process
}



void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(E_PIN), readPinsISR, FALLING);
}



void loop() {}
