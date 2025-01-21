#include <EEPROM.h>

// Motor pins
#define ENA 5
#define IN1 7
#define IN2 8
#define ENB 6
#define IN3 9
#define IN4 10

// IR Sensor pins
#define IR1 A0  // Left sensor
#define IR2 A1  // Center sensor
#define IR3 A2  // Right sensor

// Button pins
#define BUTTON1 2
#define BUTTON2 3

// LED pins
#define LED1 11
#define LED2 12

// Constants
const int SPEED_NORMAL = 150;
const int SPEED_TURN = 130;
const int TURN_DELAY = 500;
const int MOVE_DELAY = 100;
const int SENSOR_THRESHOLD = 500;  // Adjust based on your IR sensors
const int PATH_MAX_LENGTH = 150;

// State variables
char path[PATH_MAX_LENGTH];
int pathIndex = 0;
bool explorationMode = false;
bool shortestPathMode = false;
unsigned long lastIntersectionTime = 0;
const unsigned long DEBOUNCE_TIME = 250;  // Debounce time for intersection detection

// PID Control variables
float Kp = 0.8;    // Proportional gain
float Ki = 0.1;    // Integral gain
float Kd = 0.2;    // Derivative gain
int lastError = 0;
int integral = 0;

void setup() {
  // Motor pins setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // IR Sensor pins
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);

  // Button & LED pins
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  Serial.begin(9600);
}

// Enhanced motor control with speed parameters
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 135);
  rightSpeed = constrain(rightSpeed, 0, 135);
  
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
}

void moveForward(int speed = SPEED_NORMAL) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeeds(speed, speed);
}

void moveBackward(int speed = SPEED_NORMAL) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setMotorSpeeds(speed, speed);
}

void turnLeft(int speed = SPEED_TURN) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeeds(speed, speed);
  delay(TURN_DELAY);
}

void turnRight(int speed = SPEED_TURN) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setMotorSpeeds(speed, speed);
  delay(TURN_DELAY);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  setMotorSpeeds(0, 0);
}

// PID line following
int calculatePID() {
  int error = readLineSensors();
  integral += error;
  int derivative = error - lastError;
  lastError = error;
  
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

// Read sensors and return position error
int readLineSensors() {
  int leftVal = analogRead(IR1);
  int centerVal = analogRead(IR2);
  int rightVal = analogRead(IR3);
  
  // Convert analog readings to digital (0 or 1)
  int left = (leftVal > SENSOR_THRESHOLD) ? 1 : 0;
  int center = (centerVal > SENSOR_THRESHOLD) ? 1 : 0;
  int right = (rightVal > SENSOR_THRESHOLD) ? 1 : 0;
  
  // Calculate position error
  if (center == 1) return 0;
  else if (left == 1) return -1;
  else if (right == 1) return 1;
  return lastError;  // Keep last error if no line detected
}

// Detect intersection type
char detectIntersection() {
  int left = analogRead(IR1) > SENSOR_THRESHOLD;
  int center = analogRead(IR2) > SENSOR_THRESHOLD;
  int right = analogRead(IR3) > SENSOR_THRESHOLD;
  
  if (left && center && right) return 'T';      // T intersection
  else if (left && center) return 'L';          // Left turn
  else if (center && right) return 'R';         // Right turn
  else if (!left && !center && !right) return 'D'; // Dead end
  else if (center) return 'F';                  // Forward
  return 'N';                                   // No intersection
}

// Enhanced path recording with intersection detection
void recordMove(char move) {
  if (pathIndex < PATH_MAX_LENGTH - 1) {
    // Debounce intersection detection
    unsigned long currentTime = millis();
    if (currentTime - lastIntersectionTime > DEBOUNCE_TIME) {
      path[pathIndex++] = move;
      lastIntersectionTime = currentTime;
      
      // Debug output
      Serial.print("Recorded move: ");
      Serial.println(move);
    }
  }
}

// Optimize recorded path
void optimizePath() {
  char optimizedPath[PATH_MAX_LENGTH];
  int optimizedIndex = 0;
  
  // Replace sequences with shortcuts
  for (int i = 0; i < pathIndex; i++) {
    if (i + 2 < pathIndex) {
      // Check for U-turn patterns
      if (path[i] == 'L' && path[i+1] == 'B' && path[i+2] == 'L') {
        optimizedPath[optimizedIndex++] = 'U';
        i += 2;
        continue;
      }
      // Check for simplified turns
      if (path[i] == 'L' && path[i+1] == 'R') {
        optimizedPath[optimizedIndex++] = 'F';
        i++;
        continue;
      }
    }
    optimizedPath[optimizedIndex++] = path[i];
  }
  
  // Copy optimized path back
  memcpy(path, optimizedPath, optimizedIndex);
  pathIndex = optimizedIndex;
}

// Main maze solving function using Left Wall Following
void solveMaze() {
  char intersection = detectIntersection();
  int pidValue = calculatePID();
  
  // Adjust motor speeds based on PID
  int leftSpeed = SPEED_NORMAL + pidValue;
  int rightSpeed = SPEED_NORMAL - pidValue;
  
  switch (intersection) {
    case 'L':  // Left turn available
      turnLeft();
      recordMove('L');
      break;
      
    case 'F':  // Forward path available
      moveForward(SPEED_NORMAL);
      recordMove('F');
      break;
      
    case 'R':  // Right turn available
      if (detectIntersection() != 'L') {  // No left turn available
        turnRight();
        recordMove('R');
      }
      break;
      
    case 'D':  // Dead end
      turnRight();
      turnRight();  // 180-degree turn
      recordMove('B');
      break;
      
    case 'T':  // T intersection, prefer left
      turnLeft();
      recordMove('L');
      break;
      
    default:  // Normal line following
      setMotorSpeeds(leftSpeed, rightSpeed);
      break;
  }
}

// Execute the optimized path
void runShortestPath() {
  for (int i = 0; i < pathIndex; i++) {
    switch (path[i]) {
      case 'F': 
        moveForward(SPEED_NORMAL);
        delay(MOVE_DELAY);
        break;
      case 'L': 
        turnLeft();
        break;
      case 'R': 
        turnRight();
        break;
      case 'B': 
        turnRight();
        turnRight();
        break;
      case 'U': 
        turnRight();
        turnRight();
        break;
    }
    // Brief pause between movements
    delay(50);
  }
  stopMotors();
}

void loop() {
  if (digitalRead(BUTTON1) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(BUTTON1) == LOW) {
      // Start maze solving
      explorationMode = true;
      shortestPathMode = false;
      pathIndex = 0;  // Reset path
      digitalWrite(LED1, HIGH);
      
      while (explorationMode) {
        solveMaze();
        // Check for end condition (implement your end detection logic)
        // For example, if both outer sensors detect the line at the end marker
        if (analogRead(IR1) > SENSOR_THRESHOLD && analogRead(IR3) > SENSOR_THRESHOLD) {
          explorationMode = false;
        }
      }
      
      digitalWrite(LED1, LOW);
      optimizePath();
    }
  }

  if (digitalRead(BUTTON2) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(BUTTON2) == LOW) {
      // Run shortest path
      shortestPathMode = true;
      digitalWrite(LED2, HIGH);
      
      runShortestPath();
      
      digitalWrite(LED2, LOW);
      shortestPathMode = false;
    }
  }
}