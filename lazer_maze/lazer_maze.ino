/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-mp3-player
 */

// Pin definitions
#define IR_SENSOR_PIN 10          // IR breakbeam sensor
#define PRIZE_RELAY_PIN 6         // Prize light relay
#define LAZERS_RELAY_PIN 5        // Laser light relay
// Serial1 uses pins 0 (RX) and 1 (TX) for MP3 player communication

// MP3 player commands
#define CMD_PLAY_NEXT 0x01
#define CMD_PLAY_PREV 0x02
#define CMD_PLAY_W_INDEX 0x03
#define CMD_SET_VOLUME 0x06
#define CMD_SEL_DEV 0x09
#define CMD_PLAY_W_VOL 0x22
#define CMD_PLAY 0x0D
#define CMD_PAUSE 0x0E
#define CMD_SINGLE_CYCLE 0x19

#define DEV_TF 0x02
#define SINGLE_CYCLE_ON 0x00
#define SINGLE_CYCLE_OFF 0x01

// Game state constants
#define BEAM_HOLD_TIME 5000       // 5 seconds in milliseconds
#define SOUND_1 0x0001            // First sound file
#define SOUND_2 0x0002            // Second sound file

// Game state variables
bool beamConnected = false;
bool gameReset = false;
bool sound1Played = false;
unsigned long beamStartTime = 0;
unsigned long lastBeamCheck = 0;

// Relay state variables for testing
bool relayState = false;
unsigned long lastToggleTime = 0;
#define TOGGLE_INTERVAL 1000      // Toggle every 1000ms (1 second)


void setup() {
  Serial.begin(9600);     // USB Serial for debugging
  Serial1.begin(9600);    // Hardware Serial1 for MP3 player
  
  // Initialize IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);  // Use internal pullup resistor
  
  // Initialize relay pins
  pinMode(PRIZE_RELAY_PIN, OUTPUT);
  pinMode(LAZERS_RELAY_PIN, OUTPUT);
  digitalWrite(PRIZE_RELAY_PIN, LOW);    // Start with relays OFF
  digitalWrite(LAZERS_RELAY_PIN, LOW);
  
  delay(500);  // wait chip initialization is complete

  mp3_command(CMD_SEL_DEV, DEV_TF);  // select the TF card
  delay(200);                        // wait for 200ms
  
  mp3_command(CMD_SET_VOLUME, 25);   // Set reasonable volume
  delay(100);
  
  // Initialize game state
  gameReset = false;
  sound1Played = false;
  beamConnected = false;
  beamStartTime = 0;
  
  Serial.println("Laser Maze initialized. Waiting for beam...");
}

void loop() {
  // Read current sensor state (LOW = beam connected, HIGH = beam broken for most IR sensors)
  bool currentBeamState = (digitalRead(IR_SENSOR_PIN) == HIGH);
  unsigned long currentTime = millis();
  
  // Check if beam state changed
  if (currentBeamState != beamConnected) {
    beamConnected = currentBeamState;
    
    if (beamConnected) {
      // Beam just connected
      beamStartTime = currentTime;
      Serial.println("Beam connected - starting timer...");
    } else {
      // Beam just broken
      Serial.println("Beam broken!");
      
      // If we're in the post-reset state and beam breaks, play sound 2
      if (gameReset && sound1Played) {
        Serial.println("Playing sound 2 - beam broken after reset!");
        mp3_command(CMD_PLAY_W_INDEX, SOUND_2);
        
        // Reset game state for next round
        gameReset = false;
        sound1Played = false;
      }
      
      beamStartTime = 0; // Reset timer
    }
  }
  
  // If beam is connected and we haven't reset yet, check for 5-second hold
  if (beamConnected && !gameReset && beamStartTime > 0) {
    if (currentTime - beamStartTime >= BEAM_HOLD_TIME) {
      // Beam held for 5 seconds!
      Serial.println("Beam held for 5 seconds - resetting and playing sound 1!");
      
      gameReset = true;
      sound1Played = true;
      mp3_command(CMD_PLAY_W_INDEX, SOUND_1);
      
      Serial.println("Game reset! Now waiting for beam to break...");
    }
  }
  
  // Toggle relays every second for testing
  if (currentTime - lastToggleTime >= TOGGLE_INTERVAL) {
    relayState = !relayState;
    digitalWrite(PRIZE_RELAY_PIN, relayState ? HIGH : LOW);
    digitalWrite(LAZERS_RELAY_PIN, relayState ? HIGH : LOW);
    lastToggleTime = currentTime;
    
    Serial.print("Relays toggled to: ");
    Serial.println(relayState ? "ON" : "OFF");
  }
  
  // Small delay to prevent excessive polling
  delay(50);
}

void mp3_command(int8_t command, int16_t dat) {
  Serial.print("MP3 Command: ");
  Serial.print(command, HEX);
  Serial.print(", Data: ");
  Serial.println(dat, HEX);

  int8_t frame[8] = { 0 };
  frame[0] = 0x7e;                // starting byte
  frame[1] = 0xff;                // version
  frame[2] = 0x06;                // the number of bytes of the command without starting byte and ending byte
  frame[3] = command;             //
  frame[4] = 0x00;                // 0x00 = no feedback, 0x01 = feedback
  frame[5] = (int8_t)(dat >> 8);  // data high byte
  frame[6] = (int8_t)(dat);       // data low byte
  frame[7] = 0xef;                // ending byte
  for (uint8_t i = 0; i < 8; i++) {
    Serial1.write(frame[i]);        // Use Serial1 for MP3 communication
  }
}
