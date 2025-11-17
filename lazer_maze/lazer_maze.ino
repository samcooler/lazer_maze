// Laser Maze State Machine - Hotel Room Speed Edition

// ==================== PIN DEFINITIONS ====================
#define TRIP_WIRE_PIN 10          // Trip wire continuity sensor (LOW=intact, HIGH=broken)
#define ENTRANCE_BEAM_PIN 13      // Optional entrance break beam (HIGH=broken, LOW=blocked)
#define PRIZE_SENSOR_PIN 12       // Prize sensor (triggers victory)

// Relay outputs for AC lights
#define UV_LIGHT_1_PIN 5          // First UV "laser" light
#define UV_LIGHT_2_PIN 6          // Second UV "laser" light  
#define PRIZE_LIGHT_PIN 9         // White prize light
#define SPARE_OUTPUT_PIN 15       // Spare output for future use

// Serial1 pins 0(RX) and 1(TX) for MP3 player

// ==================== MP3 COMMANDS ====================
#define CMD_PLAY_W_INDEX 0x03
#define CMD_SET_VOLUME 0x06
#define CMD_SEL_DEV 0x09
#define DEV_TF 0x02

// Sound file indices (in order on SD card: 0001-0005)
#define SOUND_ARM 0x0001          // Armed/entrance sound
#define SOUND_ALARM_1 0x0002      // First alarm option
#define SOUND_ALARM_2 0x0003      // Second alarm option
#define SOUND_VICTORY_1 0x0004    // First victory option
#define SOUND_VICTORY_2 0x0005    // Second victory option

// Sound selection (change these to pick which sounds to use)
#define SOUND_ALARM SOUND_ALARM_1      // Use SOUND_ALARM_1 or SOUND_ALARM_2
#define SOUND_VICTORY SOUND_VICTORY_1  // Use SOUND_VICTORY_1 or SOUND_VICTORY_2

// ==================== GAME STATES ====================
enum GameState {
  STATE_STARTUP,
  STATE_WAITING,
  STATE_ARMED,
  STATE_ALARMED,
  STATE_VICTORY
};

// ==================== TIMING CONSTANTS ====================
#define ARMED_DURATION 15000      // How long to stay armed before going to waiting (15 sec)
#define VICTORY_DURATION 30000    // How long victory state lasts (30 sec)
#define ALARM_FLASH_COUNT 6       // Number of light flashes during alarm
#define ALARM_FLASH_INTERVAL 300  // Flash speed in ms
#define PRIZE_BLINK_INTERVAL 3000 // Prize light blink in waiting (3 sec)
#define UV_PULSE_INTERVAL 2000    // UV pulse in alarmed state (2 sec)
#define DEBOUNCE_DELAY 50         // Sensor debounce time

// ==================== TEST MODE ====================
#define OUTPUT_TEST_MODE false    // Set to true to test relay outputs
#define SENSOR_TEST_MODE false    // Set to true to test sensor inputs
#define AUDIO_TEST_MODE false     // Set to true to test audio files

// ==================== OPTIONAL SENSORS ====================
#define USE_ENTRANCE_BEAM false  // Set to true when entrance beam sensor is connected
#define USE_PRIZE_SENSOR false    // Set to true when prize sensor is connected

// ==================== GLOBAL STATE ====================
GameState currentState = STATE_STARTUP;
unsigned long stateStartTime = 0;
unsigned long lastBlinkTime = 0;
bool blinkState = false;

// Sensor states
bool tripWireIntact = true;
bool entranceBeamBroken = false;
bool prizeSensorTriggered = false;

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  
  // Wait 1 second for Serial connection to establish
  delay(2000);
  
  // Configure pins
  pinMode(TRIP_WIRE_PIN, INPUT_PULLUP);
  
  // Only configure optional sensor pins if enabled
  if (USE_ENTRANCE_BEAM) {
    pinMode(ENTRANCE_BEAM_PIN, INPUT_PULLUP);
  }
  if (USE_PRIZE_SENSOR) {
    pinMode(PRIZE_SENSOR_PIN, INPUT_PULLUP);
  }
  
  pinMode(UV_LIGHT_1_PIN, OUTPUT);
  pinMode(UV_LIGHT_2_PIN, OUTPUT);
  pinMode(PRIZE_LIGHT_PIN, OUTPUT);
  pinMode(SPARE_OUTPUT_PIN, OUTPUT);  // Future use
  
  // All lights off initially
  allLightsOff();
  
  // Initialize sensor states based on configuration
  tripWireIntact = true;
  entranceBeamBroken = false;
  prizeSensorTriggered = false;
  
  // Print startup info
  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("    LASER MAZE - Hotel Room Edition");
  Serial.println("========================================");
  Serial.println("Hardware Configuration:");
  Serial.print("  - Trip Wire Sensor: Pin ");
  Serial.println(TRIP_WIRE_PIN);
  Serial.print("  - Entrance Beam: ");
  Serial.println(USE_ENTRANCE_BEAM ? "ENABLED" : "DISABLED");
  Serial.print("  - Prize Sensor: ");
  Serial.println(USE_PRIZE_SENSOR ? "ENABLED" : "DISABLED");
  Serial.print("  - UV Light 1: Pin ");
  Serial.println(UV_LIGHT_1_PIN);
  Serial.print("  - UV Light 2: Pin ");
  Serial.println(UV_LIGHT_2_PIN);
  Serial.print("  - Prize Light: Pin ");
  Serial.println(PRIZE_LIGHT_PIN);
  Serial.println("========================================");
  Serial.println();
  
  // Run diagnostics if test modes are enabled
  if (OUTPUT_TEST_MODE) {
    Serial.println("*** OUTPUT TEST MODE - TESTING RELAY OUTPUTS ***");
    // Skip normal initialization, stay in test mode
  } else if (SENSOR_TEST_MODE) {
    Serial.println("*** SENSOR TEST MODE - MONITORING SENSOR INPUTS ***");
    // Skip normal initialization, stay in test mode
  } else if (AUDIO_TEST_MODE) {
    Serial.println("*** AUDIO TEST MODE ***");
    // Initialize MP3 for audio testing
    mp3_command(CMD_SEL_DEV, DEV_TF);
    delay(200);
    mp3_command(CMD_SET_VOLUME, 25);
    delay(100);
  } else {
    // Initialize MP3 player
    mp3_command(CMD_SEL_DEV, DEV_TF);
    delay(200);
    mp3_command(CMD_SET_VOLUME, 25);
    delay(100);
    
    Serial.println("=== LASER MAZE INITIALIZED ===");
    // Initialize state machine timing (currentState already set to STATE_STARTUP globally)
    stateStartTime = millis();
  }
}

// ==================== MAIN LOOP ====================
void loop() {
  if (OUTPUT_TEST_MODE) {
    // Test relay outputs only
    runOutputTest();
    return;
  }
  
  if (SENSOR_TEST_MODE) {
    // Monitor sensor inputs only
    runSensorTest();
    return;
  }
  
  if (AUDIO_TEST_MODE) {
    // Audio test mode
    runAudioTest();
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Read sensors
  updateSensors();
  
  // State machine
  switch(currentState) {
    case STATE_STARTUP:
      handleStartup(currentTime);
      break;
      
    case STATE_WAITING:
      handleWaiting(currentTime);
      break;
      
    case STATE_ARMED:
      handleArmed(currentTime);
      break;
      
    case STATE_ALARMED:
      handleAlarmed(currentTime);
      break;
      
    case STATE_VICTORY:
      handleVictory(currentTime);
      break;
  }
  
  delay(20);
}

// ==================== STATE HANDLERS ====================

void handleStartup(unsigned long currentTime) {
  // Play startup sequence: blink each light once
  unsigned long elapsed = currentTime - stateStartTime;
  
  if (elapsed < 500) {
    digitalWrite(UV_LIGHT_1_PIN, HIGH);
  } else if (elapsed < 1000) {
    digitalWrite(UV_LIGHT_1_PIN, LOW);
    digitalWrite(UV_LIGHT_2_PIN, HIGH);
  } else if (elapsed < 1500) {
    digitalWrite(UV_LIGHT_2_PIN, LOW);
    digitalWrite(PRIZE_LIGHT_PIN, HIGH);
  } else if (elapsed < 2000) {
    digitalWrite(PRIZE_LIGHT_PIN, LOW);
  } else {
    // Startup complete, go directly to armed
    enterState(STATE_ARMED);
  }
}

void handleWaiting(unsigned long currentTime) {
  unsigned long elapsed = currentTime - stateStartTime;
  
  // Prize light blinks periodically
  if (currentTime - lastBlinkTime >= PRIZE_BLINK_INTERVAL) {
    blinkState = !blinkState;
    digitalWrite(PRIZE_LIGHT_PIN, blinkState ? HIGH : LOW);
    lastBlinkTime = currentTime;
  }
  
  // Check for entrance beam trigger (only if enabled)
  if (USE_ENTRANCE_BEAM && elapsed > 2000 && entranceBeamBroken) {
    Serial.println("Entrance beam broken - ARMING!");
    enterState(STATE_ARMED);
    return;
  }
  
  // Auto-arm after 5 seconds if entrance beam not enabled
  if (!USE_ENTRANCE_BEAM && elapsed > 5000) {
    Serial.println("Auto-arming (entrance beam disabled) - ARMING!");
    enterState(STATE_ARMED);
    return;
  }
  
  // Check for trip wire break
  if (!tripWireIntact) {
    Serial.println("Trip wire broken in WAITING - ALARM!");
    enterState(STATE_ALARMED);
    return;
  }
}

void handleArmed(unsigned long currentTime) {
  unsigned long elapsed = currentTime - stateStartTime;
  static unsigned long lastPrizeFlash = 0;
  static bool prizeFlashing = false;
  
  // Flash prize light for 2 seconds every 8 seconds
  unsigned long flashCycle = (currentTime - stateStartTime) % 8000;
  if (flashCycle < 2000) {
    if (!prizeFlashing) {
      digitalWrite(PRIZE_LIGHT_PIN, HIGH);
      prizeFlashing = true;
    }
  } else {
    if (prizeFlashing) {
      digitalWrite(PRIZE_LIGHT_PIN, LOW);
      prizeFlashing = false;
    }
  }
  
  // Check for prize sensor trigger (only if enabled) - VICTORY!
  if (USE_PRIZE_SENSOR && prizeSensorTriggered) {
    Serial.println("PRIZE SENSOR TRIGGERED - VICTORY!");
    enterState(STATE_VICTORY);
    return;
  }
  
  // Check for trip wire break - immediate alarm
  if (!tripWireIntact) {
    Serial.println("TRIP WIRE BROKEN - ALARM!");
    enterState(STATE_ALARMED);
    return;
  }
  
  // Stay in ARMED mode indefinitely (no timeout)
}

void handleAlarmed(unsigned long currentTime) {
  unsigned long elapsed = currentTime - stateStartTime;
  static unsigned long lastUVToggle = 0;
  static unsigned long lastWhiteFlash = 0;
  static unsigned long lastAlarmSound = 0;
  static bool uvState = false;
  static bool whiteState = false;
  
  // Play alarm sound every 20 seconds
  if (currentTime - lastAlarmSound >= 20000) {
    mp3_command(CMD_PLAY_W_INDEX, SOUND_ALARM);
    lastAlarmSound = currentTime;
  }
  
  // Alternate UV lights every 400ms
  if (currentTime - lastUVToggle >= 400) {
    uvState = !uvState;
    // Toggle between UV1 and UV2
    digitalWrite(UV_LIGHT_1_PIN, uvState ? HIGH : LOW);
    digitalWrite(UV_LIGHT_2_PIN, uvState ? LOW : HIGH);
    lastUVToggle = currentTime;
  }
  
  // Flash white light at different interval (250ms)
  if (currentTime - lastWhiteFlash >= 250) {
    whiteState = !whiteState;
    digitalWrite(PRIZE_LIGHT_PIN, whiteState ? HIGH : LOW);
    lastWhiteFlash = currentTime;
  }
  
  // Reset immediately when trip wire is restored
  if (tripWireIntact) {
    Serial.println("Trip wire restored - returning to ARMED");
    enterState(STATE_ARMED);
  }
}

void handleVictory(unsigned long currentTime) {
  unsigned long elapsed = currentTime - stateStartTime;
  
  // Victory state lasts 30 seconds
  if (elapsed >= VICTORY_DURATION) {
    Serial.println("Victory complete - returning to ARMED");
    enterState(STATE_ARMED);
  }
}

// ==================== STATE TRANSITIONS ====================

const char* getStateName(GameState state) {
  switch(state) {
    case STATE_STARTUP: return "STARTUP";
    case STATE_WAITING: return "WAITING";
    case STATE_ARMED: return "ARMED";
    case STATE_ALARMED: return "ALARMED";
    case STATE_VICTORY: return "VICTORY";
    default: return "UNKNOWN";
  }
}

void enterState(GameState newState) {
  Serial.print("STATE CHANGE: ");
  Serial.print(getStateName(currentState));
  Serial.print(" -> ");
  Serial.println(getStateName(newState));
  
  currentState = newState;
  stateStartTime = millis();
  lastBlinkTime = millis();
  blinkState = false;
  
  // State entry actions
  switch(newState) {
    case STATE_STARTUP:
      allLightsOff();
      // No sound on startup
      break;
      
    case STATE_WAITING:
      allLightsOff();
      digitalWrite(PRIZE_LIGHT_PIN, HIGH);  // Prize light starts on
      // No sound on waiting
      break;
      
    case STATE_ARMED:
      allLightsOff();
      digitalWrite(UV_LIGHT_1_PIN, HIGH);
      digitalWrite(UV_LIGHT_2_PIN, HIGH);
      mp3_command(CMD_PLAY_W_INDEX, SOUND_ARM);
      break;
      
    case STATE_ALARMED:
      mp3_command(CMD_PLAY_W_INDEX, SOUND_ALARM);
      // Lights controlled by handler
      break;
      
    case STATE_VICTORY:
      allLightsOff();
      digitalWrite(PRIZE_LIGHT_PIN, HIGH);  // Prize light on
      mp3_command(CMD_PLAY_W_INDEX, SOUND_VICTORY);
      break;
  }
}

// ==================== DIAGNOSTICS ====================

// ==================== OUTPUT TEST MODE ====================
void runOutputTest() {
  static unsigned long lastDiagTime = 0;
  static bool inDiagSequence = false;
  static unsigned long diagStartTime = 0;
  
  unsigned long currentTime = millis();
  
  // Run diagnostic sequence every 5 seconds
  if (!inDiagSequence && (currentTime - lastDiagTime >= 5000)) {
    inDiagSequence = true;
    diagStartTime = currentTime;
    lastDiagTime = currentTime;
    Serial.println("\n>>> TESTING RELAY OUTPUTS <<<");
  }
  
  // Run the relay test sequence
  if (inDiagSequence) {
    unsigned long elapsed = currentTime - diagStartTime;
    
    if (elapsed < 1000) {
      // UV Light 1 ON
      if (elapsed == 0 || (currentTime - diagStartTime) < 100) {
        Serial.println("UV Light 1 (Pin 5) ON");
      }
      digitalWrite(UV_LIGHT_1_PIN, HIGH);
    } else if (elapsed < 1200) {
      digitalWrite(UV_LIGHT_1_PIN, LOW);
    } else if (elapsed < 2200) {
      // UV Light 2 ON
      if (elapsed >= 1200 && elapsed < 1300) {
        Serial.println("UV Light 2 (Pin 6) ON");
      }
      digitalWrite(UV_LIGHT_2_PIN, HIGH);
    } else if (elapsed < 2400) {
      digitalWrite(UV_LIGHT_2_PIN, LOW);
    } else if (elapsed < 3400) {
      // Prize Light ON
      if (elapsed >= 2400 && elapsed < 2500) {
        Serial.println("Prize Light (Pin 9) ON");
      }
      digitalWrite(PRIZE_LIGHT_PIN, HIGH);
    } else if (elapsed < 3600) {
      digitalWrite(PRIZE_LIGHT_PIN, LOW);
    } else if (elapsed < 4600) {
      // Spare Output ON
      if (elapsed >= 3600 && elapsed < 3700) {
        Serial.println("Spare Output (Pin 15) ON");
      }
      digitalWrite(SPARE_OUTPUT_PIN, HIGH);
    } else if (elapsed < 4800) {
      digitalWrite(SPARE_OUTPUT_PIN, LOW);
      Serial.println(">>> RELAY TEST COMPLETE <<<\n");
      inDiagSequence = false;
    }
  }
  
  delay(10);
}

// ==================== SENSOR TEST MODE ====================
void runSensorTest() {
  static unsigned long lastSensorPrint = 0;
  unsigned long currentTime = millis();
  
  // Display sensor status continuously every 200ms
  if (currentTime - lastSensorPrint >= 200) {
    bool tripWire = digitalRead(TRIP_WIRE_PIN);
    bool entranceBeam = digitalRead(ENTRANCE_BEAM_PIN);
    bool prizeSensor = digitalRead(PRIZE_SENSOR_PIN);
    
    Serial.print("SENSORS | Trip Wire (Pin 10): ");
    Serial.print(tripWire ? "HIGH (broken)" : "LOW (intact)");
    Serial.print(" | Entrance Beam (Pin 13): ");
    Serial.print(entranceBeam ? "HIGH (broken)" : "LOW (blocked)");
    Serial.print(" | Prize (Pin 12): ");
    Serial.println(prizeSensor ? "HIGH (triggered)" : "LOW (ready)");
    
    lastSensorPrint = currentTime;
  }
  
  delay(10);
}

// ==================== AUDIO TEST MODE ====================
void runAudioTest() {
  static unsigned long lastTestTime = 0;
  static int currentSound = 1;
  
  unsigned long currentTime = millis();
  
  // Play each sound every 7 seconds
  if (currentTime - lastTestTime >= 7000) {
    Serial.print("\n>>> PLAYING SOUND ");
    Serial.print(currentSound);
    Serial.print(": ");
    
    switch(currentSound) {
      case 1:
        Serial.println("ARM (0001.mp3)");
        mp3_command(CMD_PLAY_W_INDEX, SOUND_ARM);
        break;
      case 2:
        Serial.println("ALARM_1 (0002.mp3)");
        mp3_command(CMD_PLAY_W_INDEX, SOUND_ALARM_1);
        break;
      case 3:
        Serial.println("ALARM_2 (0003.mp3)");
        mp3_command(CMD_PLAY_W_INDEX, SOUND_ALARM_2);
        break;
      case 4:
        Serial.println("VICTORY_1 (0004.mp3)");
        mp3_command(CMD_PLAY_W_INDEX, SOUND_VICTORY_1);
        break;
      case 5:
        Serial.println("VICTORY_2 (0005.mp3)");
        mp3_command(CMD_PLAY_W_INDEX, SOUND_VICTORY_2);
        break;
    }
    
    currentSound++;
    if (currentSound > 5) {
      currentSound = 1;
      Serial.println("\n>>> AUDIO TEST LOOP COMPLETE - RESTARTING <<<");
    }
    
    lastTestTime = currentTime;
  }
  
  delay(100);
}

// ==================== HELPER FUNCTIONS ====================

void updateSensors() {
  // Read trip wire (LOW = intact, HIGH = broken)
  tripWireIntact = digitalRead(TRIP_WIRE_PIN) == LOW;
  
  // Read entrance beam only if enabled (HIGH = broken, LOW = blocked)
  if (USE_ENTRANCE_BEAM) {
    entranceBeamBroken = digitalRead(ENTRANCE_BEAM_PIN) == HIGH;
  } else {
    entranceBeamBroken = false;  // Ignored when disabled
  }
  
  // Read prize sensor only if enabled (HIGH = triggered)
  if (USE_PRIZE_SENSOR) {
    prizeSensorTriggered = digitalRead(PRIZE_SENSOR_PIN) == HIGH;
  } else {
    prizeSensorTriggered = false;  // Ignored when disabled
  }
}

void allLightsOff() {
  digitalWrite(UV_LIGHT_1_PIN, LOW);
  digitalWrite(UV_LIGHT_2_PIN, LOW);
  digitalWrite(PRIZE_LIGHT_PIN, LOW);
  digitalWrite(SPARE_OUTPUT_PIN, LOW);
}

void mp3_command(int8_t command, int16_t dat) {
  int8_t frame[8] = { 0 };
  frame[0] = 0x7e;                // starting byte
  frame[1] = 0xff;                // version
  frame[2] = 0x06;                // length
  frame[3] = command;
  frame[4] = 0x00;                // no feedback
  frame[5] = (int8_t)(dat >> 8);  // data high byte
  frame[6] = (int8_t)(dat);       // data low byte
  frame[7] = 0xef;                // ending byte
  
  for (uint8_t i = 0; i < 8; i++) {
    Serial1.write(frame[i]);
  }
}
