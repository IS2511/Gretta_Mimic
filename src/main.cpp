#include <Arduino.h>



// === LIBRARIES ===

// Main lib, used to communicate with engines
#include <SCServo.h>
// I2C support library
#include <Wire.h>
// Graphics library
#include <Adafruit_GFX.h>
// Display support library
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
// #include <Adafruit_SSD1306.h> // Planning to switch from LCD to OLED
// Timers libraries
#include <TimerOne.h>
// #include <TimerThree.h> // DEPRECATED: Was planned to time hands sensors reads
// IR control receiver library
#include <IRremote.h>
// Thread libraries
#include <Thread.h>
// #include <ThreadController.h> // Not needed, no dynamic threads used
#include <StaticThreadController.h>
// #include <SD.h> // DEPRECATED: Was never used



// === DEFINES ===

#define MOVIE_DEBUG_LVL1_ON false
#define MOVIE_DEBUG_LVL2_ON false
#define MOVIE_FRAMES_COUNT 40

#define IR_DEBUG_LVL1_ON false
#define IR_DEBUG_LVL2_ON false
#define IR_HOLD_RECV_COUNT 7
#define IR_CLEAR_TIME 200

#define SCS_COUNT 15
#define SCS_READ_RETRY_COUNT 3
#define SCS_MAX_LOAD 100

#define LCD_ON true
#define DISPLAY_ON true // Display refers to OLED display
#define LCD_UPDATE_MILLIS 1000
#define DISPLAY_CHAR_WIDTH 6  // OLED config
#define DISPLAY_CHAR_HEIGHT 8 // OLD config

#define HAND_ON false // Touch detection
#define HAND_PIN_R 35
#define HAND_PIN_L 34

/*
 * On the current working (can be demonstrated) version of Gretta (0.2.3)
 * The hand sensors are outdated in terms of design
 * And are being replaced with new ones in version 0.2.4 (currently testing)
 * Capacity touch sensors used in version 0.2.3 are a bit heavy on calculation
 * So that part of code is moved to a separate Arduino Nano
 * And here I only read 2 GPIO pins like hands 'is pressed?' state
*/



// Uhh... Just don't look here, ok?
// It's for debuging, never used in actual releases

// Replace lcd -> Serial
// #define lcd Serial
// #define clear() println("\n\n")
// #define setCursor(...) print("")
// #define print(...) println(__VA_ARGS__)

// Replace lcd -> OLED
// #define lcd display
// #define clear() clearDisplay()
// #define setCursor(x, y) setCursor(x+16, y+16)
// #define print(...) println(__VA_ARGS__)



// === PINOUT ===

// Connect this pin to the Arduino RESET pin
// And you can reset using IR control
// NOTE: Disconnect the pinReset/RESET pin if uploading code!
byte pinReset = 12; // RESET - pin
// Connect the receiver data pin to this one
byte pinIR = 25; // IR signal - pin
byte pinLED = LED_BUILTIN;
// DEPRECATED: Was never used (SD card)
// byte chipSelect = 46; // SD card CS - pin
// MOSI - 51
// SCK - 52
// MISO - 50



// === VARIABLES ===

// Utility variables
bool startupFinished = false;
unsigned long loopCount = 0;
// Servos connection vars from the lib
SCSCL sc;
SMSCL sm;
// LiquidCrystal_I2C lcd(0x3F, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
// Setting the LCD on address 0x27 (alternative 0x3F) to 16 chars 2 lines
LiquidCrystal_I2C lcd(0x3F, 16, 2);
// Custom character "\", used for stylish loading
uint8_t charBackslash [8] = { 0x00, 0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00 };
char loadingChar[4] = {'|', '/', '-', '\0'};
// I couldn't resist... I mean it's an LCD display, it's meant for this!

// Planned on migration from Gretta 0.2.3 to 0.2.4
// // DISPLAY:
// // yellow top 16px height
// // Symbol: 6px width | 8px height
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
// Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
// // Available colors: BLACK, WHITE, INVERSE
// static const byte PROGMEM displayWidth = 128;
// static const byte PROGMEM displayHeight = 64;

// Image storage for OLED display
// static const byte lightning_xbm_width = 10;
// static const byte lightning_xbm_height = 14;
// static const unsigned char PROGMEM lightning_xbm[] = {
//   0x80, 0x00, 0x40, 0x00, 0x60, 0x00, 0x30, 0x00, 0x38, 0x00, 0x1c, 0x00,
//   0xfe, 0x03, 0xff, 0x01, 0xe0, 0x00, 0x70, 0x00, 0x30, 0x00, 0x18, 0x00,
//   0x08, 0x00, 0x04, 0x00 };


// The movie and servos part
typedef struct {
  u16 position[SCS_COUNT]; // Positions of all servos
  word time = 1000;          // Time to complete the move (in ms)
  bool recorded = false;     // There is no easy way to check this, so storing
} frame;
unsigned long lastFrameStart = 0;
frame movie[MOVIE_FRAMES_COUNT]; // Show me your moves
byte frameId = 0;
bool modePlay = false; // false - record, true - play
bool movieLoop = false;
bool pause = true;
byte ID[SCS_COUNT] = { // Servos IDs, predefined by design
  1, 2, 3, 11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26
};
// IDs structure:
// 1, 2, 3 - 3 neck axis
// 1x - right hand top to bottom (1-6)
// 2x - left hand top to bottom (1-6)
bool isSC[SCS_COUNT] = { // Different connection protocols (SCSCL and SMSCL)
  true, true, true,
  false, false, false, false, true, true,
  false, false, false, false, true, true
};
bool isOnline[SCS_COUNT] = {
  true, true, true, true, true, true, true, true, true, true, true, true, true, true, true
}; // All true for debug

// The IR control part
IRrecv irrecv(pinIR);
decode_results payload;
byte hold = 0;
unsigned long lastPayload = 0;

typedef enum { IDLE, PRESSED, HOLD, RELEASED } KeyState;

bool lcdUpdateNeeded = false;
bool lcdBusy = true;
// Boolean set in the IR thread, used when
// The main thread is not responding
bool stopEverything = false;


KeyState button[17]; // IR remote state
long int lastButton = 0;
unsigned long int buttonTimeout[17] = {};
static const long int buttonMap[17] = {
  0xFF629D,
  0xFFC23D,
  0xFFA857,
  0xFF22DD,
  0xFF02FD,

  0xFF6897,
  0xFF9867,
  0xFFB04F,
  0xFF30CF,
  0xFF18E7,
  0xFF7A85,
  0xFF10EF,
  0xFF38C7,
  0xFF5AA5,
  0xFF4AB5,

  0xFF42BD,
  0xFF52AD
};
char chars[17] = {
  'U', 'R', 'D', 'L', 'K', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '*', '#'
};



// === UTILITY FUNCTIONS ===

// Reverses the byte and returns it
byte byteReverse(byte b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

word wordReverse(word w) {
  return (byteReverse(w&B11111111))<<8 | (byteReverse(w>>8));
}

char enumToChar(KeyState state) {
  char ch;
  switch(state) {
    case IDLE: ch = 'I'; break;
    case PRESSED: ch = 'P'; break;
    case HOLD: ch = 'H'; break;
    case RELEASED: ch = 'R'; break;
  }
  return ch;
}

char buttonToID(long int value) {
  byte i = 0;
  for (i = 0; i < 17; i++) {
    if (buttonMap[i] == value) break;
  }
  return i;
}

byte charToID(char value) {
  byte i = 0;
  for (i = 0; i < 17; i++) {
    if (chars[i] == value) break;
  }
  return i;
}

KeyState buttonGet(long int value) {
  byte i = 0;
  for (i = 0; i < 17; i++) {
    if (buttonMap[i] == value) break;
  }
  return button[i];
}

void buttonSet(long int value, KeyState state) {
  button[buttonToID(value)] = state;
}

// DEPRECATED: Was used for plotting some debug info
// float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
//   return ( (x - in_min) * (out_max - out_min) / (in_max - in_min) - out_min );
// }

// Stylish padding
String pad(word x, byte l, bool zero = true) {
  String str = "";
  for (size_t i = 0; i < l-String(x).length(); i++) {
    if (zero)
			str += "0";
		else
			str += " ";
  }
  str += String(x);
  return str;
}

/*
  UP - FF629D
  RIGHT - FFC23D
  DOWN - FFA857
  LEFT - FF22DD
  OK - FF02FD

  1 - FF6897
  2 - FF9867
  3 - FFB04F
  4 - FF30CF
  5 - FF18E7
  6 - FF7A85
  7 - FF10EF
  8 - FF38C7
  9 - FF5AA5
  0 - FF4AB5

  * - FF42BD
  # - FF52AD
*/

/*
  Controls

  Press OK once at the start to begin

  Hold # to change mode (record/play+pause)

  In record mode:
  Press OK to record a frame (will autoincrement)
  Press or hold > and < to change frame number (frameId)
  Press /\ and \/ to change frame time

  In play/pause mode:
  Press OK to pause/play
  Press or hold > and < to change frame number (frameId)

  Hold * to check (ping) online motors

  Hold 2 to turn all torque off

  Hold 5 to clear the movie

  Hold 8 to toggle loop playing

  Press 0 to stop any action

  * WARNING * Can cause damage
  Hold 0 at any time to reset via GPIO pinReset+RESET (pull to ground)
  Disconnect the pinReset/RESET pin if uploading code!

*/



// === THREADS ===

// This threads is responsible for
// Listening to IR receiver
class IRThread: public Thread
{
public:
  // Last IR data received
	volatile unsigned long value;
  // Check if new data was received
  volatile bool update = false;

	void run() {
    if (irrecv.decode(&payload)) {
      value = payload.value;
      update = true;
      irrecv.resume(); // Receive the next value (continue waiting)
    }
		runned();
	}
};

// This threads is responsible for
// Listening to Arduino Nano sending hand status on 2 pins
class HandThread: public Thread
{
public:
	volatile bool handPressedR = false;
  volatile bool handPressedL = false;

	void run() {
    handPressedR = (digitalRead(HAND_PIN_R) == HIGH);
    handPressedL = (digitalRead(HAND_PIN_L) == HIGH);

		runned();
	}
};

HandThread handthread = HandThread();

IRThread irthread = IRThread();

// This threads is responsible for
// Updating KeyState from received IR data
Thread irupdate = Thread();

StaticThreadController<3> controller (&irthread, &irupdate, &handthread);

// Used to time threads controller check
void timerCallback() {
	controller.run();
}



// === SERVOS FUNCTIONS ===

bool Ping(byte i) {
  int p;
  if (isSC[i])
    p = sc.Ping(ID[i]);
  else
    p = sm.Ping(ID[i]);

  return (p == ID[i]);
}

int ReadVoltage(byte i) {
  int v;
  if (isSC[i])
    v = sc.ReadVoltage(ID[i]);
  else
    v =  sm.ReadVoltage(ID[i]);

  return v;
}

int ReadPos(byte i) {
  int pos;
  if (isSC[i])
    pos = sc.ReadPos(ID[i]);
  else
    pos = sm.ReadPos(ID[i]);

  return pos;
}

void SyncWritePos2() { // Name taken from the library
  // sc.SyncWritePos2(ID, byte(SCS_COUNT), movie[frameId].position, word(movie[frameId].time), word(0));
  // delay(10);
  // sm.SyncWritePos2(ID, byte(SCS_COUNT), movie[frameId].position, word(movie[frameId].time), word(0));

  // Function in library is not working properly... Replacing!
  for (byte i = 0; i < SCS_COUNT; i++) {
    if (!isOnline[i]) continue;
    delay(1);
    if (!isSC[i])
      sm.WritePos(ID[i], movie[frameId].position[i], word(movie[frameId].time), word(0));
    else
      sc.WritePos(ID[i], movie[frameId].position[i], word(movie[frameId].time), word(0));

  }

}








void handleIR();
void handleMotors();
void lcdUpdate();
void checkTorque();

void setup()
{
  // Preventing reset
  digitalWrite(pinReset, HIGH);

  // Connecting Serial ports
  Serial.begin(115200);
  Serial1.begin(1000000); // 1Mbit/s
  // Setting serial pointers for servo lib
  sc.pSerial = &Serial1;
  sm.pSerial = &Serial1;
  Serial.println(F("Serial ports initialized!"));

  // Initializing LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, charBackslash);
  lcd.clear();
  lcd.print("Hello world!");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  Serial.println(F("LCD is online! (Or I think it is)"));

  // Initializing IR receiver
  irrecv.enableIRIn(); // Starting the IR receiver

  // Setting pins modes
  pinMode(pinReset, OUTPUT);
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, LOW);

  // Setting up threads configs and starting
  irthread.setInterval(100);

  handthread.setInterval(100);

  irupdate.onRun(handleIR);
  irupdate.setInterval(100);

  Timer1.initialize(10000); // this*10e-3 to convert to ms
  Timer1.attachInterrupt(timerCallback, 10000);

  // Now we are ready!
  Serial.println(F("Starting..."));


  lcd.setCursor(0, 1);
  lcd.print("Press OK!     ");

}

// It's stylish, trust me
// This one is a running back and forth dot
void animateLoading(byte row, byte start, byte end) {
  if (end-start < 1) return;
  lcd.setCursor(start, row);
  String blank = "";
  byte length = end-start+1;
  for (byte i = 0; i < length; i++) blank += " ";
  blank[((loopCount/100)%(length*2) >= length) ? ((loopCount/100)%length) : (length-(loopCount/100)%length)-1] = '.';
  lcd.print(blank);
}
// This one is a spinning stick
void animateLoading(byte row, byte col) {
  lcd.setCursor(col, row);
  lcd.print(loadingChar[(millis()/300)%4]);
}

void lcdUpdate() {
  if (lcdBusy) return;

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("F: "); // 10 chars left
  lcd.print(frameId, DEC);
  lcd.setCursor(0, 0);
  lcd.print("Status: "); // 8 chars left
  if (pause) {
    if (modePlay) {
      lcd.print("pause");
      lcd.setCursor(6, 1);
      lcd.print("Hi :)");
    } else {
      lcd.print("record");
      lcd.setCursor(6, 1);
      lcd.print("R");
      lcd.print((movie[frameId].recorded) ? "O" : "X");
      lcd.print(" t");
      lcd.print(movie[frameId].time, DEC);
    }
  } else {
    lcd.print("play");
    lcd.setCursor(6, 1);
    lcd.print("t");
    // lcd.print((), DEC);
    lcd.print(movie[frameId].time, DEC);
  }

}

// Blocks loop!
void checkOnline() {

  lcdBusy = true;
  lcd.clear();
  bool allOnline = true;
  String blank = "";
  for (byte i = 0; i < SCS_COUNT; i++) blank += "X";
  lcd.print(blank);
  // delete &blank; // Optimization?
  lcd.setCursor(0, 1);
  lcd.print("^");
  for (byte i = 0; i < SCS_COUNT; i++) {
    if (i != 0) {
      lcd.setCursor(i-1, 1);
      lcd.print(" ^");
    }
    for (byte retry = 0; retry < SCS_READ_RETRY_COUNT; retry++) {
      lcd.setCursor(15, 1);
      // lcd.print(loadingChar[(retry+i)%4]); // Maybe...?
      isOnline[i] = false;
      // delay(10); // Safety delay, not needed
      if (Ping(i) && retry != SCS_READ_RETRY_COUNT-1) {
        lcd.setCursor(i, 0);
        lcd.print("O");
        isOnline[i] = true;
        break;
      }
      if (retry == SCS_READ_RETRY_COUNT-1) allOnline = false;
      if (stopEverything) return;
    }

  }
  lcd.setCursor(0, 1);
  if (allOnline) {
    lcd.print("All online!");
    delay(1000);
  } else {
    lcd.print("Not all online! ");
    lcd.setCursor(0, 1);
    delay(1000);
    lcd.print("Continue? (OK)  ");
    while (button[charToID('K')] != PRESSED) delay(100);
  }
  lcd.clear();
  delay(600);
  lcdBusy = false;
  lcdUpdateNeeded = true;

}

// Blocks loop!
void recordPositions() {

  lcdBusy = true;
  lcd.clear();
  bool recorded = true;
  int pos = 0;
  String blank = "";
  for (byte i = 0; i < SCS_COUNT; i++) blank += "X";
  lcd.print(blank);
  // delete &blank; // Optimization?
  lcd.setCursor(0, 1);
  lcd.print("^");
  for (byte i = 0; i < SCS_COUNT; i++) {
    if (!isOnline[i]) continue;
    if (i != 0) {
      lcd.setCursor(i-1, 1);
      lcd.print(" ^");
    }
    for (byte retry = 0; retry < SCS_READ_RETRY_COUNT; retry++) {
      delay(10);
      // lcd.setCursor(15, 1);
      // lcd.print(loadingChar[(millis()/300)%4]);
      animateLoading(1, 15); // Replacing ^
      pos = ReadPos(i);
      if (pos != -1 && retry != SCS_READ_RETRY_COUNT-1) {
        lcd.setCursor(i, 0);
        lcd.print("O");
        movie[frameId].position[i] = pos;
        break;
      }
      if (retry == SCS_READ_RETRY_COUNT-1) recorded = false;
      if (stopEverything) return;
    }

  }
  movie[frameId].recorded = recorded;
  lcd.setCursor(0, 1);
  if (recorded) {
    lcd.print("All recorded!");
    lcd.setCursor(0, 1);
    frameId++;
    delay(500);
  } else {
    lcd.print("FAILED REC!");
    lcd.setCursor(0, 1);
    delay(1000);
    lcd.print("Continue? (OK)  ");
    while (button[charToID('K')] != PRESSED) delay(100);
  }
  lcd.clear();
  delay(100);
  lcdBusy = false;
  lcdUpdateNeeded = true;

}






void loop()
{
  // Kinda garbage code, but I'm too lazy to do it properly
  // Not using while so loop repeats and resets watchdog
  if (!startupFinished) {
    // animateLoading(1, 10, 15); // TODO: Try this
    animateLoading(1, 14);


    lcd.setCursor(14, 0);
    lcd.print((handthread.handPressedL ? "X" : "O"));
    lcd.print((handthread.handPressedR ? "X" : "O"));

    if (button[charToID('K')] == PRESSED) {
      startupFinished = true;
      lcd.clear();
      button[charToID('K')] = IDLE;
      lcdBusy = false;
      lcdUpdate();
      delay(100);
    }
    return;
  }


  if (button[charToID('*')] == HOLD) checkOnline();

  if (button[charToID('K')] == PRESSED) {
    if (modePlay) { // Pause/Play
      if (!pause) {
        lastFrameStart = 0;
        frameId++;
      }
      if ( !(pause && !movie[frameId].recorded) )
        pause = !pause;
      lcdUpdate();
    } else { // Record positions
      recordPositions();
    }
    delay(400);
  }

  if (button[charToID('#')] == HOLD) { // Change mode
    modePlay = !modePlay;
    lcdUpdate();
    delay(600);
  }

  if (button[charToID('5')] == HOLD) { // Clear movie
    lcdBusy = true;
    lcd.clear();
    lcd.print("Clearing movie");
    lcd.setCursor(0, 1);
    lcd.print("loading...");

    lastFrameStart = 0;
    for (byte i = 0; i < 60; i++) {
      // movie[i].start = 0;
      movie[i].time = 1000;
      for (byte j = 0; j < SCS_COUNT; j++) movie[i].position[j] = 0;
    }
    delay(600);
    lcdBusy = false;
    lcdUpdate();
  }

  if (button[charToID('2')] == HOLD) { // Turn all torque off
    for (byte i = 0; i < SCS_COUNT; i++) {
      if (isSC[i])
        sc.EnableTorque(ID[i], 0);
      else
        sm.EnableTorque(ID[i], 0);
    }
    lcd.setCursor(0, 1);
    lcd.print("Torque off!");
    delay(600);
    lcdUpdateNeeded = true;
  }

  if (button[charToID('8')] == HOLD) { // Loop on/off
    movieLoop = !movieLoop;
    lcd.setCursor(0, 1);
    lcd.print("Loop: ");
    if (movieLoop)
      lcd.print("ENABLED!");
    else
      lcd.print("disabled.");
    delay(600);
    lcdUpdateNeeded = true;
  }



  // !!! TODO !!! (WIP)
  // if (HAND_ON)
  // if ( pause && (handthread.handPressedL || handthread.handPressedR) ) {
  //   frameId++;
  //   pause = false;
  // }


  // The main motors control
  if (!pause) handleMotors();
  // checkTorque();

  // Frame time borders
  if (movie[frameId].time == 65336 || movie[frameId].time == 65136 || movie[frameId].time == 64936) {
    movie[frameId].time = 65400;
    lcdUpdateNeeded = true;
  }
  if (movie[frameId].time == 64 || movie[frameId].time == 264 || movie[frameId].time == 464) {
    movie[frameId].time = 0;
    lcdUpdateNeeded = true;
  }


  if ( lcdUpdateNeeded ) {
    lcdUpdate();
    lcdUpdateNeeded = false;

    // Maybe if needed?
    // lcd.setCursor(0, 15);
    // lcd.print(handthread.handPressedR ? "X" : "O");
    // lcd.print(handthread.handPressedL ? "X" : "O");
  }

  if (stopEverything) {
    stopEverything = false;
    lcd.setCursor(0, 1);
    lcd.print("Everything stop!");
    delay(600);
    lcdBusy = false;
    lcdUpdateNeeded = true;
  }

  loopCount++;
}



// Not used anywhere
void checkTorque() {

  for (byte i = 0; i < SCS_COUNT; i++) {
    for (byte retry = 0; retry < SCS_READ_RETRY_COUNT; retry++) {
      word load = 0;
      if (isSC[i]) {
        load = sc.ReadLoad(ID[i]);
        if (load > SCS_MAX_LOAD) sc.EnableTorque(ID[i], 0);
      } else {
        load = sm.ReadLoad(ID[i]);
        if (load > SCS_MAX_LOAD) sm.EnableTorque(ID[i], 0);
      }

    }
  }

}

// Low response time button controls
void handleButtons() {

  if (button[charToID('0')] == PRESSED) stopEverything = true;

  if (button[charToID('R')] == PRESSED || button[charToID('R')] == HOLD) {
    frameId++;
    lcdUpdateNeeded = true;
  }
  if (button[charToID('L')] == PRESSED || button[charToID('L')] == HOLD) {
    frameId--;
    lcdUpdateNeeded = true;
  }

  if (button[charToID('U')] == PRESSED || button[charToID('U')] == HOLD) {
    movie[frameId].time += 200;
    lcdUpdateNeeded = true;
  }
  if (button[charToID('D')] == PRESSED || button[charToID('D')] == HOLD) {
    movie[frameId].time -= 200;
    lcdUpdateNeeded = true;
  }


}


void handleMotors() {

  if (millis() > lastFrameStart + movie[frameId].time) {
    lcdUpdateNeeded = true;
    byte frameIdOld = frameId;
    frameId++;
    if (frameId > 59) {
      pause = true;
      frameId = 59;
      lastFrameStart = 0;
    }
    if (MOVIE_DEBUG_LVL1_ON) {
      Serial.print(F("Frame: "));
      Serial.print(frameIdOld, DEC);
      Serial.print(F(" -> "));
      Serial.println(frameId, DEC);
    }

    if (!movie[frameId].recorded) {
      pause = true;
      lastFrameStart = 0;
      if (movieLoop && !movie[frameId+1].recorded) {
        frameId = 0;
        pause = false;
      }
    } else {
      if (MOVIE_DEBUG_LVL1_ON) {
        Serial.print(F("Writing frame: "));
        Serial.println(frameId, DEC);
      }
      if (MOVIE_DEBUG_LVL2_ON) {
        Serial.print(F("Info: t"));
        Serial.print(movie[frameId].time, DEC);
        Serial.print(F(" Pos:"));
        for (byte i = 0; i < SCS_COUNT; i++) {
          Serial.print(F(" "));
          Serial.print(pad(movie[frameId].position[i], 2, false));
        }
        Serial.println(F(""));
      }
      lastFrameStart = millis();
      SyncWritePos2();
    }

  }
  // if (lastFrameStart == 0 && !pause) {
  //   lcdUpdateNeeded = true;
  //   lastFrameStart = millis();
  //   SyncWritePos2();
  // }

}

void handleIR() {

  if (irthread.update) { // irrecv.decode(&payload)
    irthread.update = false;
    long int value = irthread.value;

    lastPayload = millis();

    if (IR_DEBUG_LVL2_ON) Serial.println(value, HEX);

    if (value == 0xFFFFFFFF) {
      hold++;
      if (hold > IR_HOLD_RECV_COUNT) buttonSet(lastButton, HOLD);
      if (IR_DEBUG_LVL1_ON) {
        Serial.println("+HOLD! "+String( chars[buttonToID(lastButton)] )+" : "+String( enumToChar(buttonGet(lastButton)) ));
      }
    } else {
      hold = 0;
      buttonSet(lastButton, IDLE);
      if (IR_DEBUG_LVL1_ON) {
        Serial.println("-HOLD! "+String( chars[buttonToID(lastButton)] )+" : "+String( enumToChar(buttonGet(lastButton)) ));
      }
      lastButton = value;
      buttonSet(lastButton, PRESSED);
      if (IR_DEBUG_LVL1_ON) {
        Serial.println("PRESS! "+String( chars[buttonToID(lastButton)] )+" : "+String( enumToChar(buttonGet(lastButton)) ));
      }
    }

    if (buttonGet(lastButton) == PRESSED || buttonGet(lastButton) == HOLD)
      buttonTimeout[buttonToID(lastButton)] = millis();

    handleButtons(); // Low delay buttons
  }

  for (byte i = 0; i < 17; i++) {
    if ( millis() > buttonTimeout[i] + IR_CLEAR_TIME ) {
      if (button[i] != IDLE) {
        button[i] = IDLE;
        if (IR_DEBUG_LVL1_ON) {
          Serial.println("TIMEOUT! "+String( chars[i] )+" : I");
        }
      }

    }
  }
  if ( millis() > lastPayload + IR_CLEAR_TIME ) {
    hold = 0;
    lastButton = 0;
  }

  // Reset function
  // Very low level
  if (button[charToID('0')] == HOLD) digitalWrite(pinReset, LOW);


}
