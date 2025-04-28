#define E_PIN 3
#define RW_PIN 4
#define RS_PIN 2

#define COLS 20

#define USE_DIRECT_ISR

#include <ctype.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BUFF_SIZE 256


uint8_t counter = 0;
static uint8_t databuff = 0;
static uint8_t commandbuff = 0;


static volatile struct {
  uint8_t pinD;
  uint8_t pinB;
} circle_buff[BUFF_SIZE];
static volatile uint8_t wp = 0;
static volatile uint8_t rp = 0;

uint8_t ddram[80 * 2];
uint8_t ddram_ac;

bool ddram_change = false;
enum mode_e {
  DDRAM,
  CGRAM
} mode;



// #define USE_COUNTERS

#ifdef USE_COUNTERS
static uint8_t cmd[16];
static uint8_t cmd_cnt = 0;

static struct counters_s {
  int8_t ddram_overflow_cnt;
  int8_t data_cnt;
  int8_t latest_cmd;

  int8_t shift_cnt;
} counters;
#endif

#
static enum __attribute__((packed)) State {
  R,
  DH,
  DL,
  CH,
  CL,
  D8,
  C8
} currentState;

static bool mode_8bit = true;



static void execCommand(uint8_t command) {
  // Serial.print("Command: ");
  // Serial.println(command, HEX);
#ifdef USE_COUNTERS
  counters.latest_cmd = command;
  cmd[cmd_cnt] = command;
  cmd_cnt++;
  if (cmd_cnt >= sizeof(cmd)) {
    cmd_cnt = 0;
  }
#endif

  if ((command & 0b10000000) != 0) {
    mode = DDRAM;
    ddram_ac = command & 0b01111111;
    // Serial.println(ddram_ac, HEX);
  } else if ((command & 0b01000000) != 0) {
    //dont care cgram address
    mode = CGRAM;

  } else if ((command & 0b00100000) != 0) {
    //dont care, fuction set
  } else if ((command & 0b00010000) != 0) {
//tbd display cursor shift
#ifdef USE_COUNTERS
    counters.shift_cnt++;
#endif
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

static void execData(uint8_t data) {
#ifdef USE_COUNTERS
  counters.data_cnt++;
#endif
  if (mode == DDRAM) {
    // if (isprint((char)data)) {
    ddram[ddram_ac % sizeof(ddram)] = data;
    ddram_ac++;
    ddram_change = true;
    // ddram_change = true;
    // printDdram();
  }
}

static char fprintable(char in) {
  if (isprint(in)) return in;
  return '?';
}

static void printDdram() {

#ifdef USE_COUNTERS
  if (1) {
    Serial.print("Ovf: ");
    Serial.println(counters.ddram_overflow_cnt);

    Serial.print("Dt: ");
    Serial.println(counters.data_cnt);
    Serial.print("Sft: ");
    Serial.println(counters.shift_cnt);

    Serial.println("Cmds: ");

    for (uint8_t i = 0; i < sizeof(cmd); i++) {
      Serial.print(cmd[i], HEX);
      Serial.print(", ");
    }
    Serial.println();

    Serial.print("Lastest: ");
    Serial.println(counters.latest_cmd, HEX);

    
    Serial.print("Wp: ");
    Serial.println(wp, HEX);


  }
#endif



  if (0) {
    Serial.println("LCD: ");
    for (uint8_t i = 0; i < sizeof(ddram); i++) {
      Serial.print(fprintable(ddram[i]));
    }
    Serial.println();
  }

  //
  Serial.println("LCD Proper:");
  for (uint8_t i = 0; i < COLS; i++) {
    Serial.print(fprintable(ddram[0 + i]));
  }
  Serial.println();
  for (uint8_t i = 0; i < COLS; i++) {
    Serial.print(fprintable(ddram[0x40 + i]));
  }
  Serial.println();
  for (uint8_t i = 0; i < COLS; i++) {
    Serial.print(fprintable(ddram[0x14 + i]));
  }
  Serial.println();
  for (uint8_t i = 0; i < COLS; i++) {
    Serial.print(fprintable(ddram[0x54 + i]));
  }
  Serial.println();
}


void __attribute__((noinline)) control(uint8_t pinSnapshot, uint8_t rs_pin, uint8_t rw_pin) {
  // Serial.println();
  uint8_t buff = pinSnapshot;

retry:

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
        goto retry;

      } else {
        currentState = mode_8bit ? C8 : CH;
        // continue;
        goto retry;
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
        if (rw_pin == 0) {
          // Serial.print(buff);
          execCommand(commandbuff);
        }
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

        if (rw_pin == 0) {
          execData(databuff);
        }
      } else {

        // Serial.print("D8 Err");
        currentState = R;
        // continue;
      }

      break;
  }
}



void __attribute__((noinline)) handle_pins(uint8_t pinB, uint8_t pinD) {
  uint8_t rs_pin = pinD & (1 << 2);
  uint8_t rw_pin = pinD & (1 << 4);
#define BITMAP(pin, s, d) (((pin & (1 << s)) >> s) << d)

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

  control(a, rs_pin, rw_pin);
}


#ifdef NO____USE_DIRECT_ISR

// SPI interrupt routine
ISR(INT1_vect) {

  uint8_t pinD = PIND;
  uint8_t pinB = PINB;

  PORTC |= (1 << 0);
  handle_pins(pinB, pinD);

  PORTC &= ~(1 << 0);
}
/
#endif


#ifdef USE_DIRECT_ISR

  // SPI interrupt routine
  ISR(INT1_vect) {

  uint8_t pinD = PIND;
  uint8_t pinB = PINB;

  PORTC &= ~(1 << 0);

  circle_buff[wp].pinD = pinD;
  circle_buff[wp].pinB = pinB;

  PORTC |= (1 << 0);
  wp++;
  wp = wp % BUFF_SIZE;
}  // end of interrupt routine SPI_STC_vect

#endif



#ifndef USE_DIRECT_ISR
static void readPinsISR() {
  // Serial.println("co jest");

  uint8_t pinD = PIND;
  uint8_t pinB = PINB;

  uint8_t rs_pin = pinD & (1 << 2);
  uint8_t rw_pin = pinD & (1 << 4);
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


  control(a, rs_pin, rw_pin);
  PORTC &= ~(1 << 0);
  //0output = 0
}
#endif

void setup() {


#ifdef USE_DIRECT_ISR
  EICRA &= ~(bit(ISC11) | bit(ISC10));  // clear existing flags
  EICRA |= (1 << ISC11);                // ISC01 = 1, ISC00 = 0 → falling edge
  EIMSK |= (1 << INT1);
#endif



  // 	// enable timer 0 overflow interrupt
  // #if defined(TIMSK) && defined(TOIE0)
  // 	cbi(TIMSK, TOIE0);
  // #elif defined(TIMSK0) && defined(TOIE0)
  // 	cbi(TIMSK0, TOIE0);
  // #else
  // 	#error	Timer 0 overflow interrupt not set correctly
  // #endif




  DDRC |= (1 << 0);
  // PORTC |= (1 << 0);
  // delay(10);
  // PORTC &= ~(1 << 0);

  Serial.begin(115200);
#ifndef USE_DIRECT_ISR
  attachInterrupt(digitalPinToInterrupt(E_PIN), readPinsISR, FALLING);
#endif
  memset(ddram, ' ', sizeof(ddram));
  Serial.println(("--------------------------------- RESET ---------------------------------"));
  ddram_change = true;
}

void loop() {
#if 1
  while (Serial.available() > 0) {
    Serial.read();
    ddram_change = true;
  }

  while (wp != rp) {
    handle_pins(circle_buff[rp].pinB, circle_buff[rp].pinD);
    rp++;
    rp = rp % BUFF_SIZE;
  }


  if (ddram_change) {
    ddram_change = false;
    delay(50);
    printDdram();
  }
#endif
}
