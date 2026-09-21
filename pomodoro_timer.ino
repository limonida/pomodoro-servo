/*
 * Arduino Pomodoro / Simple Timer with Pause
 * 
 * PIN MAP:
 * - TM1637 Display: CLK=6, DIO=5
 * - Servo: Pin 3
 * - Button 1: Pin 7
 * - Button 2: Pin 4
 * - Button 3: Pin 2
 * - Buzzer: Pin 12
 * - Mode Switch: Pin 8
 * 
 * MODE SWITCH LOGIC:
 * - HIGH (INPUT_PULLUP not pressed) = MODE_POMODORO
 * - LOW (pressed to ground) = MODE_TIMER
 * 
 * STATE MACHINE APPROACH:
 * - Top-level modes: MODE_POMODORO, MODE_TIMER
 * - Pomodoro states: IDLE, WORK_RUNNING, WORK_PAUSED, WORK_FINISHED_BEEP,
 *   WAITING_BREAK, BREAK_RUNNING, BREAK_PAUSED, BREAK_FINISHED_BEEP,
 *   ADJUST_P, ADJUST_BS, ADJUST_BL
 * - Timer states: IDLE, RUNNING, PAUSED, FINISHED_BEEP, WAITING_BREAK,
 *   BREAK_RUNNING, BREAK_PAUSED, BREAK_FINISHED_BEEP, ADJUST_T, ADJUST_BT
 * - Global display state: DISPLAY_OFF
 * 
 * NON-BLOCKING COUNTDOWN METHOD:
 * - Uses millis() for all timing
 * - remainingMillis tracks countdown progress
 * - lastTickMillis used for drift-safe elapsed time calculation
 * - displayedMinutes calculated from remainingMillis
 * 
 * PAUSE/RESUME METHOD:
 * - Button 1 toggles pause/resume during active countdowns
 * - While paused, remainingMillis is frozen
 * - D3/D4 flash every 1000ms showing frozen value
 * - Resuming sets lastTickMillis = millis() to prevent time loss
 * 
 * EEPROM BEHAVIOR:
 * - Settings stored: P, bS, bL, T, bT
 * - On startup: read and validate settings, load defaults if invalid
 * - During adjustment: use working copy, save only on completion
 * - Counters (pomodoroCount, longBreakCount) are RAM-only
 */

#include <EEPROM.h>
#include <Servo.h>
#include <TM1637Display.h>

// Pin definitions
const uint8_t SERVO_PIN = 3;
const uint8_t SERVO_CLOCKWISE_POSITION = 180;
const uint8_t SERVO_COUNTER_CLOCKWISE_POSITION = 0;

const uint8_t BUTTON1_PIN = 7;
const uint8_t BUTTON2_PIN = 4;
const uint8_t BUTTON3_PIN = 2;

const uint8_t BUZZER_PIN = 12;
const uint8_t BUZZER_ON = HIGH;
const uint8_t BUZZER_OFF = LOW;

const uint8_t MODE_SWITCH_PIN = 8;

// Initialize display
TM1637Display display(6, 5);

// Initialize servo
Servo servo;

// 7-segment digit map
const uint8_t DIGIT_MAP[10] = {
  0x3F, // 0
  0x06, // 1
  0x5B, // 2
  0x4F, // 3
  0x66, // 4
  0x6D, // 5
  0x7D, // 6
  0x07, // 7
  0x7F, // 8
  0x6F  // 9
};

const uint8_t BLANK_DIGIT = 0x00;

// Settings structure
struct Settings {
  uint8_t P;   // Pomodoro work minutes (default 25, min 5, max 90, step 5)
  uint8_t bS;  // Short break minutes (default 5, min 1, max 30, step 1)
  uint8_t bL;  // Long break minutes (default 15, min 5, max 60, step 5)
  uint8_t T;   // Simple timer minutes (default 15, min 1, max 99, step 5)
  uint8_t bT;  // Simple timer break minutes (default 5, min 1, max 30, step 1)
};

// EEPROM address
const int EEPROM_ADDR = 0;

// Default settings
const Settings DEFAULT_SETTINGS = {25, 5, 15, 15, 5};

// Current settings (working copy)
Settings currentSettings;

// Physical mode enum
enum PhysicalMode {
  MODE_POMODORO,
  MODE_TIMER
};

// Pomodoro states
enum PomodoroState {
  POMODORO_IDLE,
  POMODORO_WORK_RUNNING,
  POMODORO_WORK_PAUSED,
  POMODORO_WORK_FINISHED_BEEP,
  POMODORO_WAITING_BREAK,
  POMODORO_BREAK_RUNNING,
  POMODORO_BREAK_PAUSED,
  POMODORO_BREAK_FINISHED_BEEP,
  POMODORO_ADJUST_P,
  POMODORO_ADJUST_BS,
  POMODORO_ADJUST_BL
};

// Timer states
enum TimerState {
  TIMER_IDLE,
  TIMER_RUNNING,
  TIMER_PAUSED,
  TIMER_FINISHED_BEEP,
  TIMER_WAITING_BREAK,
  TIMER_BREAK_RUNNING,
  TIMER_BREAK_PAUSED,
  TIMER_BREAK_FINISHED_BEEP,
  TIMER_ADJUST_T,
  TIMER_ADJUST_BT
};

// Display state
enum DisplayState {
  DISPLAY_ON,
  DISPLAY_OFF
};

// Global state variables
PhysicalMode physicalMode = MODE_POMODORO;
PomodoroState pomodoroState = POMODORO_IDLE;
TimerState timerState = TIMER_IDLE;
DisplayState displayState = DISPLAY_ON;

// Runtime counters (RAM only)
uint8_t pomodoroCount = 0;
uint8_t longBreakCount = 0;

// Countdown variables
unsigned long remainingMillis = 0;
unsigned long lastTickMillis = 0;
uint8_t displayedMinutes = 0;
bool isCountdownActive = false;

// Beep variables
bool isBeeping = false;
unsigned long beepStartMillis = 0;

// Pause flash variables
bool isPaused = false;
bool pauseVisible = true;
unsigned long pauseFlashMillis = 0;

// Button debounce variables
bool button1LastState = HIGH;
bool button2LastState = HIGH;
bool button3LastState = HIGH;
unsigned long button1DebounceTime = 0;
unsigned long button2DebounceTime = 0;
unsigned long button3DebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// Button 3 long press tracking
unsigned long button3PressStartMillis = 0;
bool button3LongPressTriggered = false;
const unsigned long LONG_PRESS_DURATION = 3000;

// Mode switch debounce
bool modeSwitchLastState = HIGH;
unsigned long modeSwitchDebounceTime = 0;

// Pending break value for Pomodoro waiting state
uint8_t pendingBreakMinutes = 0;

// Helper function prototypes
void loadDefaultSettings();
bool validateSettings(const Settings& s);
void readSettingsFromEEPROM();
void writeSettingsToEEPROM();
void servoClockwiseAction();
void servoCounterClockwiseAction();
void displayDigits(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
void displayBlank();
void updateDisplay();
void startCountdown(uint8_t minutes);
void updateCountdown();
void handleButton1();
void handleButton2();
void handleButton3();
void checkModeSwitch();
void enterPomodoroAdjustP();
void enterPomodoroAdjustBS();
void enterPomodoroAdjustBL();
void enterTimerAdjustT();
void enterTimerAdjustBT();
void incrementAdjustValue(uint8_t& value, uint8_t step, uint8_t minVal, uint8_t maxVal);
void decrementAdjustValue(uint8_t& value, uint8_t step, uint8_t minVal, uint8_t maxVal);

void setup() {
  // Initialize pins
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);
  
  // Attach servo
  servo.attach(SERVO_PIN);
  
  // Initialize display
  display.setBrightness(7);
  
  // Read settings from EEPROM
  readSettingsFromEEPROM();
  
  // Read initial mode switch state
  physicalMode = (digitalRead(MODE_SWITCH_PIN) == HIGH) ? MODE_POMODORO : MODE_TIMER;
  
  // Update display based on mode
  updateDisplay();
}

void loop() {
  unsigned long now = millis();
  
  // Check mode switch
  checkModeSwitch();
  
  // Handle buttons only if display is on
  if (displayState == DISPLAY_ON) {
    handleButton1();
    handleButton2();
    handleButton3();
  }
  
  // Update countdown if active and not paused
  if (isCountdownActive && !isPaused) {
    updateCountdown();
  }
  
  // Handle pause flashing
  if (isPaused && displayState == DISPLAY_ON) {
    if (now - pauseFlashMillis >= 1000) {
      pauseFlashMillis = now;
      pauseVisible = !pauseVisible;
      updateDisplay();
    }
  }
  
  // Handle beeping
  if (isBeeping) {
    if (now - beepStartMillis >= 1000) {
      digitalWrite(BUZZER_PIN, BUZZER_OFF);
      isBeeping = false;
      
      // Transition after beep
      if (physicalMode == MODE_POMODORO) {
        if (pomodoroState == POMODORO_WORK_FINISHED_BEEP) {
          pomodoroState = POMODORO_WAITING_BREAK;
          displayedMinutes = pendingBreakMinutes;
          updateDisplay();
        } else if (pomodoroState == POMODORO_BREAK_FINISHED_BEEP) {
          pomodoroState = POMODORO_IDLE;
          displayedMinutes = currentSettings.P;
          updateDisplay();
        }
      } else {
        if (timerState == TIMER_FINISHED_BEEP) {
          timerState = TIMER_WAITING_BREAK;
          displayedMinutes = currentSettings.bT;
          updateDisplay();
        } else if (timerState == TIMER_BREAK_FINISHED_BEEP) {
          timerState = TIMER_IDLE;
          displayedMinutes = currentSettings.T;
          updateDisplay();
        }
      }
    }
  }
  
  // Update display periodically for running countdowns
  if (isCountdownActive && !isPaused && displayState == DISPLAY_ON) {
    updateDisplay();
  }
}

void loadDefaultSettings() {
  currentSettings = DEFAULT_SETTINGS;
}

bool validateSettings(const Settings& s) {
  if (s.P < 5 || s.P > 90) return false;
  if (s.bS < 1 || s.bS > 30) return false;
  if (s.bL < 5 || s.bL > 60) return false;
  if (s.T < 1 || s.T > 99) return false;
  if (s.bT < 1 || s.bT > 30) return false;
  return true;
}

void readSettingsFromEEPROM() {
  EEPROM.get(EEPROM_ADDR, currentSettings);
  if (!validateSettings(currentSettings)) {
    loadDefaultSettings();
    writeSettingsToEEPROM();
  }
}

void writeSettingsToEEPROM() {
  EEPROM.put(EEPROM_ADDR, currentSettings);
}

void servoClockwiseAction() {
  // MG996R servos can draw high current.
  // Use an external 5V supply if needed.
  // Connect servo supply ground to Arduino ground.
  servo.write(SERVO_CLOCKWISE_POSITION);
}

void servoCounterClockwiseAction() {
  // MG996R servos can draw high current.
  // Use an external 5V supply if needed.
  // Connect servo supply ground to Arduino ground.
  servo.write(SERVO_COUNTER_CLOCKWISE_POSITION);
}

void displayDigits(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
  uint8_t segments[4];
  segments[0] = DIGIT_MAP[d1];
  segments[1] = DIGIT_MAP[d2];
  segments[2] = DIGIT_MAP[d3];
  segments[3] = DIGIT_MAP[d4];
  display.setSegments(segments, 4);
}

void displayBlank() {
  uint8_t segments[4] = {BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT};
  display.setSegments(segments, 4);
}

void updateDisplay() {
  if (displayState == DISPLAY_OFF) {
    displayBlank();
    return;
  }
  
  if (isPaused && !pauseVisible) {
    // Show D1/D2 but blank D3/D4 during pause flash
    if (physicalMode == MODE_POMODORO) {
      displayDigits(pomodoroCount, longBreakCount, 0, 0);
      // Manually blank D3/D4
      uint8_t segments[4];
      segments[0] = DIGIT_MAP[pomodoroCount];
      segments[1] = DIGIT_MAP[longBreakCount];
      segments[2] = BLANK_DIGIT;
      segments[3] = BLANK_DIGIT;
      display.setSegments(segments, 4);
    } else {
      // Timer mode: D1/D2 blank
      uint8_t segments[4] = {BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT};
      display.setSegments(segments, 4);
    }
    return;
  }
  
  uint8_t d1, d2, d3, d4;
  
  if (physicalMode == MODE_POMODORO) {
    switch (pomodoroState) {
      case POMODORO_IDLE:
        d1 = pomodoroCount;
        d2 = longBreakCount;
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        break;
      case POMODORO_WORK_RUNNING:
      case POMODORO_WORK_PAUSED:
        d1 = pomodoroCount;
        d2 = longBreakCount;
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        break;
      case POMODORO_WORK_FINISHED_BEEP:
        d1 = pomodoroCount;
        d2 = longBreakCount;
        d3 = 0;
        d4 = 0;
        break;
      case POMODORO_WAITING_BREAK:
        d1 = pomodoroCount;
        d2 = longBreakCount;
        d3 = pendingBreakMinutes / 10;
        d4 = pendingBreakMinutes % 10;
        break;
      case POMODORO_BREAK_RUNNING:
      case POMODORO_BREAK_PAUSED:
        d1 = pomodoroCount;
        d2 = longBreakCount;
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        break;
      case POMODORO_BREAK_FINISHED_BEEP:
        d1 = pomodoroCount;
        d2 = longBreakCount;
        d3 = 0;
        d4 = 0;
        break;
      case POMODORO_ADJUST_P:
      case POMODORO_ADJUST_BS:
      case POMODORO_ADJUST_BL:
        d1 = 0;
        d2 = 0;
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        break;
      default:
        d1 = 0;
        d2 = 0;
        d3 = 0;
        d4 = 0;
        break;
    }
    displayDigits(d1, d2, d3, d4);
  } else {
    // Timer mode
    switch (timerState) {
      case TIMER_IDLE:
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        displayDigits(0, 0, d3, d4);
        // Blank D1/D2
        {
          uint8_t segments[4];
          segments[0] = BLANK_DIGIT;
          segments[1] = BLANK_DIGIT;
          segments[2] = DIGIT_MAP[d3];
          segments[3] = DIGIT_MAP[d4];
          display.setSegments(segments, 4);
        }
        break;
      case TIMER_RUNNING:
      case TIMER_PAUSED:
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        {
          uint8_t segments[4];
          segments[0] = BLANK_DIGIT;
          segments[1] = BLANK_DIGIT;
          segments[2] = DIGIT_MAP[d3];
          segments[3] = DIGIT_MAP[d4];
          display.setSegments(segments, 4);
        }
        break;
      case TIMER_FINISHED_BEEP:
        {
          uint8_t segments[4];
          segments[0] = BLANK_DIGIT;
          segments[1] = BLANK_DIGIT;
          segments[2] = DIGIT_MAP[0];
          segments[3] = DIGIT_MAP[0];
          display.setSegments(segments, 4);
        }
        break;
      case TIMER_WAITING_BREAK:
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        {
          uint8_t segments[4];
          segments[0] = BLANK_DIGIT;
          segments[1] = BLANK_DIGIT;
          segments[2] = DIGIT_MAP[d3];
          segments[3] = DIGIT_MAP[d4];
          display.setSegments(segments, 4);
        }
        break;
      case TIMER_BREAK_RUNNING:
      case TIMER_BREAK_PAUSED:
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        {
          uint8_t segments[4];
          segments[0] = BLANK_DIGIT;
          segments[1] = BLANK_DIGIT;
          segments[2] = DIGIT_MAP[d3];
          segments[3] = DIGIT_MAP[d4];
          display.setSegments(segments, 4);
        }
        break;
      case TIMER_BREAK_FINISHED_BEEP:
        {
          uint8_t segments[4];
          segments[0] = BLANK_DIGIT;
          segments[1] = BLANK_DIGIT;
          segments[2] = DIGIT_MAP[0];
          segments[3] = DIGIT_MAP[0];
          display.setSegments(segments, 4);
        }
        break;
      case TIMER_ADJUST_T:
      case TIMER_ADJUST_BT:
        d3 = displayedMinutes / 10;
        d4 = displayedMinutes % 10;
        {
          uint8_t segments[4];
          segments[0] = BLANK_DIGIT;
          segments[1] = BLANK_DIGIT;
          segments[2] = DIGIT_MAP[d3];
          segments[3] = DIGIT_MAP[d4];
          display.setSegments(segments, 4);
        }
        break;
      default:
        displayBlank();
        break;
    }
  }
}

void startCountdown(uint8_t minutes) {
  remainingMillis = (unsigned long)minutes * 60000UL;
  lastTickMillis = millis();
  displayedMinutes = minutes;
  isCountdownActive = true;
  isPaused = false;
}

void updateCountdown() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastTickMillis;
  
  if (elapsed > 0) {
    if (remainingMillis > elapsed) {
      remainingMillis -= elapsed;
    } else {
      remainingMillis = 0;
    }
    lastTickMillis = now;
  }
  
  // Calculate displayed minutes (ceiling division)
  uint8_t newDisplayedMinutes = (remainingMillis + 59999UL) / 60000UL;
  
  if (newDisplayedMinutes != displayedMinutes) {
    displayedMinutes = newDisplayedMinutes;
  }
  
  // Check if countdown reached zero
  if (remainingMillis == 0) {
    isCountdownActive = false;
    isPaused = false;
    
    if (physicalMode == MODE_POMODORO) {
      if (pomodoroState == POMODORO_WORK_RUNNING) {
        pomodoroState = POMODORO_WORK_FINISHED_BEEP;
        digitalWrite(BUZZER_PIN, BUZZER_ON);
        beepStartMillis = millis();
        servoCounterClockwiseAction();
        
        // Determine pending break
        if (pomodoroCount >= 4) {
          pendingBreakMinutes = currentSettings.bL;
        } else {
          pendingBreakMinutes = currentSettings.bS;
        }
      } else if (pomodoroState == POMODORO_BREAK_RUNNING) {
        pomodoroState = POMODORO_BREAK_FINISHED_BEEP;
        digitalWrite(BUZZER_PIN, BUZZER_ON);
        beepStartMillis = millis();
        
        if (pendingBreakMinutes == currentSettings.bL) {
          // Long break finished
          pomodoroCount = 0;
          longBreakCount++;
          if (longBreakCount > 9) {
            longBreakCount = 0;
          }
        }
        // Short break finished - counters unchanged
      }
    } else {
      // Timer mode
      if (timerState == TIMER_RUNNING) {
        timerState = TIMER_FINISHED_BEEP;
        digitalWrite(BUZZER_PIN, BUZZER_ON);
        beepStartMillis = millis();
        servoCounterClockwiseAction();
      } else if (timerState == TIMER_BREAK_RUNNING) {
        timerState = TIMER_BREAK_FINISHED_BEEP;
        digitalWrite(BUZZER_PIN, BUZZER_ON);
        beepStartMillis = millis();
      }
    }
  }
}

void handleButton1() {
  bool reading = digitalRead(BUTTON1_PIN);
  
  if (reading != button1LastState) {
    button1DebounceTime = millis();
  }
  
  if (millis() - button1DebounceTime > DEBOUNCE_DELAY) {
    if (reading == LOW && button1LastState == HIGH) {
      // Button pressed - falling edge
      if (physicalMode == MODE_POMODORO) {
        switch (pomodoroState) {
          case POMODORO_IDLE:
            // Start Pomodoro work countdown
            if (pomodoroCount < 4) {
              pomodoroCount++;
            }
            servoClockwiseAction();
            startCountdown(currentSettings.P);
            pomodoroState = POMODORO_WORK_RUNNING;
            break;
          case POMODORO_WORK_RUNNING:
            // Pause
            isPaused = true;
            pauseVisible = true;
            pauseFlashMillis = millis();
            pomodoroState = POMODORO_WORK_PAUSED;
            break;
          case POMODORO_WORK_PAUSED:
            // Resume
            isPaused = false;
            lastTickMillis = millis();
            pomodoroState = POMODORO_WORK_RUNNING;
            break;
          case POMODORO_WAITING_BREAK:
            // Start break countdown
            startCountdown(pendingBreakMinutes);
            pomodoroState = POMODORO_BREAK_RUNNING;
            break;
          case POMODORO_BREAK_RUNNING:
            // Pause
            isPaused = true;
            pauseVisible = true;
            pauseFlashMillis = millis();
            pomodoroState = POMODORO_BREAK_PAUSED;
            break;
          case POMODORO_BREAK_PAUSED:
            // Resume
            isPaused = false;
            lastTickMillis = millis();
            pomodoroState = POMODORO_BREAK_RUNNING;
            break;
          case POMODORO_ADJUST_P:
            // Save and move to bS adjustment
            pomodoroState = POMODORO_ADJUST_BS;
            displayedMinutes = currentSettings.bS;
            break;
          case POMODORO_ADJUST_BS:
            // Save and move to bL adjustment
            pomodoroState = POMODORO_ADJUST_BL;
            displayedMinutes = currentSettings.bL;
            break;
          case POMODORO_ADJUST_BL:
            // Save bL and write to EEPROM, exit adjustment
            writeSettingsToEEPROM();
            pomodoroState = POMODORO_IDLE;
            displayedMinutes = currentSettings.P;
            break;
          default:
            break;
        }
      } else {
        // Timer mode
        switch (timerState) {
          case TIMER_IDLE:
            // Start timer countdown
            servoClockwiseAction();
            startCountdown(currentSettings.T);
            timerState = TIMER_RUNNING;
            break;
          case TIMER_RUNNING:
            // Pause
            isPaused = true;
            pauseVisible = true;
            pauseFlashMillis = millis();
            timerState = TIMER_PAUSED;
            break;
          case TIMER_PAUSED:
            // Resume
            isPaused = false;
            lastTickMillis = millis();
            timerState = TIMER_RUNNING;
            break;
          case TIMER_WAITING_BREAK:
            // Start break countdown
            startCountdown(currentSettings.bT);
            timerState = TIMER_BREAK_RUNNING;
            break;
          case TIMER_BREAK_RUNNING:
            // Pause
            isPaused = true;
            pauseVisible = true;
            pauseFlashMillis = millis();
            timerState = TIMER_BREAK_PAUSED;
            break;
          case TIMER_BREAK_PAUSED:
            // Resume
            isPaused = false;
            lastTickMillis = millis();
            timerState = TIMER_BREAK_RUNNING;
            break;
          case TIMER_ADJUST_T:
            // Save and move to bT adjustment
            timerState = TIMER_ADJUST_BT;
            displayedMinutes = currentSettings.bT;
            break;
          case TIMER_ADJUST_BT:
            // Save bT and write to EEPROM, exit adjustment
            writeSettingsToEEPROM();
            timerState = TIMER_IDLE;
            displayedMinutes = currentSettings.T;
            break;
          default:
            break;
        }
      }
      updateDisplay();
    }
    button1LastState = reading;
  }
}

void handleButton2() {
  bool reading = digitalRead(BUTTON2_PIN);
  
  if (reading != button2LastState) {
    button2DebounceTime = millis();
  }
  
  if (millis() - button2DebounceTime > DEBOUNCE_DELAY) {
    if (reading == LOW && button2LastState == HIGH) {
      // Button pressed - falling edge
      if (physicalMode == MODE_POMODORO) {
        if (pomodoroState == POMODORO_IDLE) {
          // Enter adjustment mode
          enterPomodoroAdjustP();
        } else if (pomodoroState == POMODORO_ADJUST_P || 
                   pomodoroState == POMODORO_ADJUST_BS || 
                   pomodoroState == POMODORO_ADJUST_BL) {
          // Decrement value
          uint8_t step, minVal, maxVal;
          
          if (pomodoroState == POMODORO_ADJUST_P) {
            step = 5; minVal = 5; maxVal = 90;
            decrementAdjustValue(currentSettings.P, step, minVal, maxVal);
            displayedMinutes = currentSettings.P;
          } else if (pomodoroState == POMODORO_ADJUST_BS) {
            step = 1; minVal = 1; maxVal = 30;
            decrementAdjustValue(currentSettings.bS, step, minVal, maxVal);
            displayedMinutes = currentSettings.bS;
          } else if (pomodoroState == POMODORO_ADJUST_BL) {
            step = 5; minVal = 5; maxVal = 60;
            decrementAdjustValue(currentSettings.bL, step, minVal, maxVal);
            displayedMinutes = currentSettings.bL;
          }
          updateDisplay();
        }
      } else {
        // Timer mode
        if (timerState == TIMER_IDLE) {
          // Enter adjustment mode
          enterTimerAdjustT();
        } else if (timerState == TIMER_ADJUST_T || 
                   timerState == TIMER_ADJUST_BT) {
          // Decrement value
          uint8_t step, minVal, maxVal;
          
          if (timerState == TIMER_ADJUST_T) {
            step = 5; minVal = 1; maxVal = 99;
            decrementAdjustValue(currentSettings.T, step, minVal, maxVal);
            displayedMinutes = currentSettings.T;
          } else if (timerState == TIMER_ADJUST_BT) {
            step = 1; minVal = 1; maxVal = 30;
            decrementAdjustValue(currentSettings.bT, step, minVal, maxVal);
            displayedMinutes = currentSettings.bT;
          }
          updateDisplay();
        }
      }
    }
    button2LastState = reading;
  }
}

void handleButton3() {
  bool reading = digitalRead(BUTTON3_PIN);
  
  if (reading != button3LastState) {
    button3DebounceTime = millis();
  }
  
  if (millis() - button3DebounceTime > DEBOUNCE_DELAY) {
    if (reading == LOW && button3LastState == HIGH) {
      // Button pressed - falling edge
      button3PressStartMillis = millis();
      button3LongPressTriggered = false;
    } else if (reading == HIGH && button3LastState == LOW) {
      // Button released - rising edge
      unsigned long pressDuration = millis() - button3PressStartMillis;
      
      if (physicalMode == MODE_POMODORO) {
        if (pomodoroState == POMODORO_ADJUST_P || 
            pomodoroState == POMODORO_ADJUST_BS || 
            pomodoroState == POMODORO_ADJUST_BL) {
          // Adjustment mode: short press increments, long press ignored
          if (pressDuration < LONG_PRESS_DURATION && !button3LongPressTriggered) {
            uint8_t step, minVal, maxVal;
            
            if (pomodoroState == POMODORO_ADJUST_P) {
              step = 5; minVal = 5; maxVal = 90;
              incrementAdjustValue(currentSettings.P, step, minVal, maxVal);
              displayedMinutes = currentSettings.P;
            } else if (pomodoroState == POMODORO_ADJUST_BS) {
              step = 1; minVal = 1; maxVal = 30;
              incrementAdjustValue(currentSettings.bS, step, minVal, maxVal);
              displayedMinutes = currentSettings.bS;
            } else if (pomodoroState == POMODORO_ADJUST_BL) {
              step = 5; minVal = 5; maxVal = 60;
              incrementAdjustValue(currentSettings.bL, step, minVal, maxVal);
              displayedMinutes = currentSettings.bL;
            }
            updateDisplay();
          }
          // Long press in adjustment mode is ignored
        } else {
          // Non-adjustment mode
          if (pressDuration >= LONG_PRESS_DURATION && !button3LongPressTriggered) {
            // Long press: turn off display
            button3LongPressTriggered = true;
            isCountdownActive = false;
            isPaused = false;
            isBeeping = false;
            digitalWrite(BUZZER_PIN, BUZZER_OFF);
            displayState = DISPLAY_OFF;
            displayBlank();
          } else if (pressDuration < LONG_PRESS_DURATION && !button3LongPressTriggered) {
            // Short press
            if (pomodoroState == POMODORO_IDLE) {
              // Reset counters
              pomodoroCount = 0;
              longBreakCount = 0;
              displayedMinutes = currentSettings.P;
              updateDisplay();
            } else if (pomodoroState == POMODORO_WORK_RUNNING || 
                       pomodoroState == POMODORO_WORK_PAUSED) {
              // Stop work countdown, reset counters
              isCountdownActive = false;
              isPaused = false;
              pomodoroCount = 0;
              longBreakCount = 0;
              pomodoroState = POMODORO_IDLE;
              displayedMinutes = currentSettings.P;
              updateDisplay();
            } else if (pomodoroState == POMODORO_WAITING_BREAK) {
              // Reset counters
              pomodoroCount = 0;
              longBreakCount = 0;
              pomodoroState = POMODORO_IDLE;
              displayedMinutes = currentSettings.P;
              updateDisplay();
            } else if (pomodoroState == POMODORO_BREAK_RUNNING || 
                       pomodoroState == POMODORO_BREAK_PAUSED) {
              // Stop break countdown, reset counters
              isCountdownActive = false;
              isPaused = false;
              pomodoroCount = 0;
              longBreakCount = 0;
              pomodoroState = POMODORO_IDLE;
              displayedMinutes = currentSettings.P;
              updateDisplay();
            }
          }
        }
      } else {
        // Timer mode
        if (timerState == TIMER_ADJUST_T || 
            timerState == TIMER_ADJUST_BT) {
          // Adjustment mode: short press increments, long press ignored
          if (pressDuration < LONG_PRESS_DURATION && !button3LongPressTriggered) {
            uint8_t step, minVal, maxVal;
            
            if (timerState == TIMER_ADJUST_T) {
              step = 5; minVal = 1; maxVal = 99;
              incrementAdjustValue(currentSettings.T, step, minVal, maxVal);
              displayedMinutes = currentSettings.T;
            } else if (timerState == TIMER_ADJUST_BT) {
              step = 1; minVal = 1; maxVal = 30;
              incrementAdjustValue(currentSettings.bT, step, minVal, maxVal);
              displayedMinutes = currentSettings.bT;
            }
            updateDisplay();
          }
          // Long press in adjustment mode is ignored
        } else {
          // Non-adjustment mode
          if (pressDuration >= LONG_PRESS_DURATION && !button3LongPressTriggered) {
            // Long press: turn off display
            button3LongPressTriggered = true;
            isCountdownActive = false;
            isPaused = false;
            isBeeping = false;
            digitalWrite(BUZZER_PIN, BUZZER_OFF);
            displayState = DISPLAY_OFF;
            displayBlank();
          } else if (pressDuration < LONG_PRESS_DURATION && !button3LongPressTriggered) {
            // Short press - does nothing in timer mode except wake display
            if (displayState == DISPLAY_OFF) {
              // Wake display
              displayState = DISPLAY_ON;
              display.setBrightness(7);
              timerState = TIMER_IDLE;
              displayedMinutes = currentSettings.T;
              updateDisplay();
            }
          }
        }
      }
    }
    
    // Check for long press while button is held
    if (reading == LOW && !button3LongPressTriggered) {
      if (millis() - button3PressStartMillis >= LONG_PRESS_DURATION) {
        // Only trigger long press action in non-adjustment states
        bool inAdjustmentMode = false;
        if (physicalMode == MODE_POMODORO) {
          inAdjustmentMode = (pomodoroState == POMODORO_ADJUST_P || 
                              pomodoroState == POMODORO_ADJUST_BS || 
                              pomodoroState == POMODORO_ADJUST_BL);
        } else {
          inAdjustmentMode = (timerState == TIMER_ADJUST_T || 
                              timerState == TIMER_ADJUST_BT);
        }
        
        if (!inAdjustmentMode) {
          button3LongPressTriggered = true;
          isCountdownActive = false;
          isPaused = false;
          isBeeping = false;
          digitalWrite(BUZZER_PIN, BUZZER_OFF);
          displayState = DISPLAY_OFF;
          displayBlank();
        }
      }
    }
    
    button3LastState = reading;
  }
}

void checkModeSwitch() {
  bool reading = digitalRead(MODE_SWITCH_PIN);
  
  if (reading != modeSwitchLastState) {
    modeSwitchDebounceTime = millis();
  }
  
  if (millis() - modeSwitchDebounceTime > DEBOUNCE_DELAY) {
    if (reading != modeSwitchLastState) {
      PhysicalMode newMode = (reading == HIGH) ? MODE_POMODORO : MODE_TIMER;
      
      if (newMode != physicalMode) {
        // Mode changed
        physicalMode = newMode;
        
        // Stop any active countdown, cancel pause, stop beep
        isCountdownActive = false;
        isPaused = false;
        isBeeping = false;
        digitalWrite(BUZZER_PIN, BUZZER_OFF);
        
        // Exit adjustment mode and enter idle state of new mode
        if (physicalMode == MODE_POMODORO) {
          pomodoroState = POMODORO_IDLE;
          displayedMinutes = currentSettings.P;
        } else {
          timerState = TIMER_IDLE;
          displayedMinutes = currentSettings.T;
        }
        
        // Update display if on
        if (displayState == DISPLAY_ON) {
          updateDisplay();
        }
      }
      modeSwitchLastState = reading;
    }
  }
}

void enterPomodoroAdjustP() {
  pomodoroState = POMODORO_ADJUST_P;
  displayedMinutes = currentSettings.P;
  updateDisplay();
}

void enterPomodoroAdjustBS() {
  pomodoroState = POMODORO_ADJUST_BS;
  displayedMinutes = currentSettings.bS;
  updateDisplay();
}

void enterPomodoroAdjustBL() {
  pomodoroState = POMODORO_ADJUST_BL;
  displayedMinutes = currentSettings.bL;
  updateDisplay();
}

void enterTimerAdjustT() {
  timerState = TIMER_ADJUST_T;
  displayedMinutes = currentSettings.T;
  updateDisplay();
}

void enterTimerAdjustBT() {
  timerState = TIMER_ADJUST_BT;
  displayedMinutes = currentSettings.bT;
  updateDisplay();
}

void incrementAdjustValue(uint8_t& value, uint8_t step, uint8_t minVal, uint8_t maxVal) {
  int newValue = (int)value + (int)step;
  if (newValue > maxVal) {
    newValue = maxVal;
  }
  value = (uint8_t)newValue;
}

void decrementAdjustValue(uint8_t& value, uint8_t step, uint8_t minVal, uint8_t maxVal) {
  int newValue = (int)value - (int)step;
  if (newValue < minVal) {
    newValue = minVal;
  }
  value = (uint8_t)newValue;
}
