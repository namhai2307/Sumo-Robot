#include <Arduino.h>
#include <TFT_eSPI.h>


//Motor Pins 
#define Motor1_Pin1 1
#define Motor1_Pin2 2
#define Motor2_Pin1 3
#define Motor2_Pin2 10

//Black and White Sensor Pins
#define IR_BACK_RIGHT  16
#define IR_BACK_LEFT   21
#define IR_FRONT_LEFT  17
#define IR_FRONT_RIGHT 12

//Ultrasonic sensor Pins
#define Trig_Pin 11 //For all 3 sensors we are using the same trig Pin 
#define Echo_Left 43 // Left Ultrasonic sensor Pin
#define Echo_Front 44 // Front Ultrasonic sensor Pin
#define Echo_Right 18 // Right Ultrasonic sensor Pin

TFT_eSPI tft = TFT_eSPI(); //This will create an object named tft from TFT_eSPI libraray ( basically used to control and draw graph on TFT display)

enum BotState {
  SEARCH,
  ATTACK,
  RETREAT,
  COUNTERATTACK
};
BotState current_state = SEARCH ;

long ultrasonic_left = 999; // 999 is a default value saying no object detected or we can no object detected
long ultrasonic_front = 999;
long ultrasonic_right = 999;

bool ir_back_right = false;
bool ir_back_left = false;
bool ir_front_left = false;
bool ir_front_right = false;

void MoveBackward(int speed){
  digitalWrite(Motor1_Pin1, HIGH);
  digitalWrite(Motor1_Pin2, LOW);
  digitalWrite(Motor2_Pin1, HIGH);
  digitalWrite(Motor2_Pin2, LOW);


}

void MoveForward(int speed){
  digitalWrite(Motor1_Pin1, LOW);
  digitalWrite(Motor1_Pin2, HIGH);
  digitalWrite(Motor2_Pin1, LOW);
  digitalWrite(Motor2_Pin2, HIGH);

}

void MoveLeft(int speed){
  digitalWrite(Motor1_Pin1, LOW); //Left motor backward
  digitalWrite(Motor1_Pin2, HIGH);
  digitalWrite(Motor2_Pin1, HIGH); //Right motor forward
  digitalWrite(Motor2_Pin2, LOW);

}

void MoveRight(int speed){
  digitalWrite(Motor1_Pin1, HIGH); //Left motor forward
  digitalWrite(Motor1_Pin2, LOW);
  digitalWrite(Motor2_Pin1, LOW); //Right motor backward
  digitalWrite(Motor2_Pin2, HIGH);

}

void StopMotor(){
  digitalWrite(Motor1_Pin1, LOW); 
  digitalWrite(Motor1_Pin2, LOW);
  digitalWrite(Motor2_Pin1, LOW); 
  digitalWrite(Motor2_Pin2, LOW);

}

int findClosestOppnent(){
  int direction = 0 ;// 0 = no valid sensor found // 1 = left sensor  //2 = front sensor // 3 = right sensor
  int smallest_distance = 999;

  if (ultrasonic_left < 50 ){
    smallest_distance = ultrasonic_left ;
    direction = 1 ; //left sensor
  }
  if (ultrasonic_front < smallest_distance && ultrasonic_front < 50){
    smallest_distance = ultrasonic_front;
    direction = 2; //front sensor
  }

  if (ultrasonic_right < smallest_distance && ultrasonic_right < 50){
    smallest_distance = ultrasonic_right;
    direction = 3; //right sensor
  }

  return direction ;
 
}

long getUltrasonicDistance(int Echo_Pin){
  // Clear the trigger pin first
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(5);
  
  // Send trigger pulse
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  
  // Read the echo pin with longer timeout for better reliability
  long duration = pulseIn(Echo_Pin, HIGH, 50000); // Increased timeout to 50ms
  
  // If no echo received or timeout occurred
  if (duration == 0) return 999; 
  
  // Convert to distance in centimeters
  long distance = duration * 0.01715;
  
  // Validate reasonable distance range (2cm to 400cm for typical HC-SR04)
  if (distance < 2 || distance > 400) {
    return 999;
  }
  
  return distance;
}

bool ReadIRSensor(int pin) {
  // Returns true if sensor detects black (edge of field)
  return digitalRead(pin) == LOW;
}

void HandleBoundaryDetection() {
  // Check if both back sensors detect black (robot backing off field)
  if (ir_back_left && ir_back_right) {
    // Move forward at 100% until both sense white
    while (ReadIRSensor(IR_BACK_LEFT) && ReadIRSensor(IR_BACK_RIGHT)) {
      MoveForward(255);
      delay(10);
    }
    return;
  }
  
  // Check if both front sensors detect black (robot going off field)
  if (ir_front_left && ir_front_right) {
    // Move backward at 100% until both sense white
    while (ReadIRSensor(IR_FRONT_LEFT) && ReadIRSensor(IR_FRONT_RIGHT)) {
      MoveBackward(255);
      delay(10);
    }
    return;
  }
  
  // Check individual sensor corrections
  bool any_sensor_black = ir_back_left || ir_back_right || ir_front_left || ir_front_right;
  
  if (any_sensor_black) {
    unsigned long start_time = millis();
    
    while (millis() - start_time < 1000) { // Maximum 1 second correction
      // Read current sensor states
      bool back_left = ReadIRSensor(IR_BACK_LEFT);
      bool back_right = ReadIRSensor(IR_BACK_RIGHT);
      bool front_left = ReadIRSensor(IR_FRONT_LEFT);
      bool front_right = ReadIRSensor(IR_FRONT_RIGHT);
      
      // If no sensors detect black, break out
      if (!back_left && !back_right && !front_left && !front_right) {
        break;
      }
      
      // Left side corrections
      if (back_left || front_left) {
        // Left motor 100%, right motor 70%
        digitalWrite(Motor1_Pin1, HIGH); // Left forward
        digitalWrite(Motor1_Pin2, LOW);
        
        // Right motor at reduced speed (simulate 70% with PWM if available)
        digitalWrite(Motor2_Pin1, HIGH); // Right forward
        digitalWrite(Motor2_Pin2, LOW);
      }
      // Right side corrections
      else if (back_right || front_right) {
        // Right motor 100%, left motor 70%
        digitalWrite(Motor2_Pin1, HIGH); // Right forward
        digitalWrite(Motor2_Pin2, LOW);
        
        // Left motor at reduced speed
        digitalWrite(Motor1_Pin1, HIGH); // Left forward
        digitalWrite(Motor1_Pin2, LOW);
      }
      
      delay(10);
    }
    
    // Stop motors after correction
    StopMotor();
  }
}

void ReadAllSensors(){
  // Read ultrasonic sensors with delays to prevent interference
  ultrasonic_left = getUltrasonicDistance(Echo_Left); //Pin 43
  delay(60); // Wait 60ms between sensor readings to prevent echo interference
  
  ultrasonic_front = getUltrasonicDistance(Echo_Front); //Pin 44
  delay(60); // Wait 60ms between sensor readings
  
  ultrasonic_right = getUltrasonicDistance(Echo_Right); // Pin 18
  delay(60); // Wait 60ms after last sensor reading

  // Read IR sensors (these are fast, no delay needed)
  ir_back_right = ReadIRSensor(IR_BACK_RIGHT);   //Pin 16
  ir_back_left = ReadIRSensor(IR_BACK_LEFT);     //Pin 21
  ir_front_left = ReadIRSensor(IR_FRONT_LEFT);   //Pin 17
  ir_front_right = ReadIRSensor(IR_FRONT_RIGHT); //Pin 12

  // Handle boundary detection with priority
  HandleBoundaryDetection();
}


void DetermineBotState(){
  int attack_direction = findClosestOppnent();

  // Check if any front sensor detects edge
  if (ir_front_left || ir_front_right){
    current_state = RETREAT ;
  }

  // Check if any back sensor detects edge
  else if (ir_back_left || ir_back_right){
    current_state = COUNTERATTACK ;
  }
 
  else if( attack_direction > 0){
    current_state = ATTACK ;
  }
  else {
     current_state = SEARCH ;
  }

}

void searchBehavior(){
  // Woodpecker strategy - alternating left and right wheel movement
  static unsigned long lastSwitchTime = 0;
  static int searchPhase = 0; // 0 = left wheel, 1 = right wheel
  static bool searchActive = false;
  
  // Initialize search sequence when first called or after phase completion
  if (!searchActive) {
    lastSwitchTime = millis();
    searchActive = true;
  }
  
  // Switch between left and right wheel every 300ms
  if (millis() - lastSwitchTime >= 300) {
    searchPhase = (searchPhase + 1) % 2; // Toggle between 0 and 1
    lastSwitchTime = millis();
  }
  
  if (searchPhase == 0) {
    // Left wheel forward, right wheel stop - robot turns right while moving
    digitalWrite(Motor1_Pin1, LOW);  // Left motor forward
    digitalWrite(Motor1_Pin2, HIGH);
    digitalWrite(Motor2_Pin1, LOW);  // Right motor stop
    digitalWrite(Motor2_Pin2, LOW);
  } else {
    // Right wheel forward, left wheel stop - robot turns left while moving  
    digitalWrite(Motor1_Pin1, LOW);  // Left motor stop
    digitalWrite(Motor1_Pin2, LOW);
    digitalWrite(Motor2_Pin1, LOW);  // Right motor forward
    digitalWrite(Motor2_Pin2, HIGH);
  }
}

void attackBehavior() {
  // If left or right sensor < 50, turn that direction until front is lowest, then drive forward
  static int attackState = 0; // 0 = seeking, 1 = attacking
  static unsigned long lastSwitch = 0;
  
  // Always update sensor readings
  long left = ultrasonic_left;
  long front = ultrasonic_front;
  long right = ultrasonic_right;

  // If already attacking (front is lowest and < 50), drive forward
  if (attackState == 1) {
    if (front < 50 && front <= left && front <= right) {
      MoveForward(255);
      return;
    } else {
      attackState = 0; // Lost target, go back to seeking
    }
  }

  // If left or right sees something < 50, turn that way
  if (left < 50 && left < front && left <= right) {
    MoveRight(200); // Turn right when left sensor detects (to face left)
    attackState = 0;
    return;
  }
  if (right < 50 && right < front && right < left) {
    MoveLeft(200); // Turn left when right sensor detects (to face right)
    attackState = 0;
    return;
  }
  // If front is lowest and < 50, start attacking
  if (front < 50 && front <= left && front <= right) {
    attackState = 1;
    MoveForward(100);
    return;
  }
  // If nothing detected, stop
  StopMotor();
}

void retreateBehavior(){
  static unsigned long phaseStartTime = 0;
  static int retreatPhase = 1;  // 1 = backward, 2 = turn
  static bool retreatActive = false;
  
  // Initialize retreat sequence when first called
  if (!retreatActive) {
    phaseStartTime = millis();
    retreatPhase = 1;
    retreatActive = true;
  }
  
  // Phase 1: Move backward for LONGER - 800ms instead of 500ms
  if (retreatPhase == 1) {
    MoveBackward(100);
    
    if (millis() - phaseStartTime >= 800) {  // ← Changed from 500 to 800
      // Switch to turn phase
      retreatPhase = 2;
      phaseStartTime = millis();  // Reset timer for turn phase
    }
  }
  
  // Phase 2: Turn MORE - 600ms instead of 300ms
  else if (retreatPhase == 2) {
    MoveRight(200);  // Turn right to new direction
    
    if (millis() - phaseStartTime >= 600) {  // ← Changed from 300 to 600
      // Retreat sequence complete
      retreatActive = false;  // Reset for next retreat
      retreatPhase = 1;       // Reset phase for next time
    }
  }
}

void counterAttackBehavior(){
  MoveForward(255);
}


void ExecuteBehavior(){
  switch (current_state){
    case SEARCH:
      searchBehavior();
      break;
    case ATTACK:
      attackBehavior();
      break;
    case RETREAT:
      retreateBehavior();
      break;
    case COUNTERATTACK:
      counterAttackBehavior();
      break;
  }
}

void UpdateDisplay(){
  // Clear screen for fresh display
  tft.fillScreen(TFT_BLACK);
  
  // Display current state with color coding
  tft.setCursor(10, 10);
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.print("STATE: ");
  
  switch (current_state) {
    case SEARCH:
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.println("SEARCH");
      break;
    case ATTACK:
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("ATTACK");
      break;
    case RETREAT:
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.println("RETREAT");
      break;
    case COUNTERATTACK:
      tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
      tft.println("COUNTER");
      break;
  }
  
  // Display ultrasonic sensor readings with color coding
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.setCursor(10, 50);
  tft.print("L:");
  if (ultrasonic_left < 80) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  tft.print(ultrasonic_left);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("cm");
  
  tft.setCursor(10, 70);
  tft.print("F:");
  if (ultrasonic_front < 80) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  } else if (ultrasonic_front == 999) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  tft.print(ultrasonic_front);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("cm");
  
  tft.setCursor(10, 90);
  tft.print("R:");
  if (ultrasonic_right < 80) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
  }
  tft.print(ultrasonic_right);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("cm");
  
  // Display IR sensor status
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.setCursor(10, 110);
  tft.print("FL:");
  if (ir_front_left) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("EDGE");
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("OK");
  }
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(" FR:");
  if (ir_front_right) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("EDGE");
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("OK");
  }
  
  tft.setCursor(10, 130);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("BL:");
  if (ir_back_left) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("EDGE");
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("OK");
  }
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(" BR:");
  if (ir_back_right) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("EDGE");
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("OK");
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10,10);
  tft.println("SUMO BOT v1.0");
  tft.setCursor(10,40);
  tft.println("Initilaizing...");

  pinMode(Motor1_Pin1, OUTPUT);
  pinMode(Motor1_Pin2, OUTPUT);
  pinMode(Motor2_Pin1, OUTPUT);
  pinMode(Motor2_Pin2, OUTPUT);


  pinMode(IR_BACK_RIGHT, INPUT);  //Pin 16
  pinMode(IR_BACK_LEFT, INPUT);   //Pin 21
  pinMode(IR_FRONT_LEFT, INPUT);  //Pin 17
  pinMode(IR_FRONT_RIGHT, INPUT); //Pin 12

  pinMode(Trig_Pin, OUTPUT); //Pin 11 is used
  pinMode(Echo_Left, INPUT); //Pin 43 is used
  pinMode(Echo_Front, INPUT); //Pin 44 is used
  pinMode(Echo_Right, INPUT);// Pin 18 is used

  pinMode(15, OUTPUT);


}

void loop() {

  digitalWrite(15, HIGH);

  ReadAllSensors();

  // Debug output for front sensor troubleshooting
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500) { // Print debug info every 500ms
    Serial.print("Ultrasonic - L: ");
    Serial.print(ultrasonic_left);
    Serial.print("cm, F: ");
    Serial.print(ultrasonic_front);
    Serial.print("cm, R: ");
    Serial.print(ultrasonic_right);
    Serial.println("cm");
    
    if (ultrasonic_front == 999) {
      Serial.println("WARNING: Front sensor returning 999 (no echo detected)");
    }
    
    lastDebugTime = millis();
  }

  DetermineBotState();

  ExecuteBehavior();

  UpdateDisplay();

}