#include <Servo.h>
#include <SoftwareSerial.h>

// --- COMMUNICATION CONFIG ---
// RX Pin 2 (Connects to ESP32 TX)
// TX Pin 3 (Connects to ESP32 RX via Logic Converter)
SoftwareSerial espSerial(2, 3); 

// --- ULTRASONIC SENSOR CONFIG ---
const int trigPin = 12; 
const int echoPin = 11; 
float distance_cm, duration_us;

// --- SERVO CONFIG ---
const int BigServoPin = 9;  // The sorter arm
const int MiniServoPin = 10; // The lid opener

Servo bigServo;
Servo miniServo; //lid opener

// Variable to store the category received from ESP32
char trashCategory; 

void setup() {
  // Debugging on Computer Screen
  Serial.begin(9600);
  
  // Communication with ESP32
  espSerial.begin(9600); 

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
   
  bigServo.attach(BigServoPin);
  miniServo.attach(MiniServoPin);

  // Initial Positions
  bigServo.write(90);  // Center/Neutral
  miniServo.write(0);  // Lid Closed (Assuming 0 is closed)
  
  Serial.println("System Ready. Waiting for object...");
}

void loop() {
  detectDistance();
   
  // Filter out noise: distance must be valid (>0) and close (<20cm)
  if(distance_cm > 0 && distance_cm < 20)
  {
    Serial.println("Object Detected!");
    
    // 1. Stop momentarily to ensure object is stable
    delay(1000); 
    
    // 2. Trigger the ESP32 to Scan
    scanInput();   
    
    // 3. Operate Servos based on result
    litter(trashCategory);
    
    // 4. Reset/Wait before next scan
    Serial.println("Resetting...");
    delay(2000); 
  }
}

void detectDistance()
{
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration_us = pulseIn(echoPin, HIGH);
  distance_cm = 0.017 * duration_us;   
}

void scanInput()
{
  Serial.println("Requesting Scan from ESP32...");
  
  // Clear any old data in the buffer
  while (espSerial.available() > 0) { espSerial.read(); }

  // Send the Trigger Signal 'S' to ESP32
  espSerial.write('S'); 
  
  // Wait for response (Blocking wait)
  // This loop freezes the Arduino until ESP32 replies
  while (espSerial.available() == 0) {
    // You could add a timeout counter here if you want
  }

  // Read the classification result ('A', 'B', 'C', 'D', etc.)
  trashCategory = espSerial.read();
  
  Serial.print("ESP32 Classified: ");
  Serial.println(trashCategory);
}

void litter(char type)
{
  // 1. Move the Big Servo (Sorter) to the correct bin
  switch(type)
  {
    case 'A': // Category 1 (e.g., Bottle)
      Serial.println("Action: Bin A");
      bigServo.write(0);    
      break;

    case 'B': // Category 2 (e.g., Box)
      Serial.println("Action: Bin B");
      bigServo.write(45);
      break;

    case 'C': // Category 3
      Serial.println("Action: Bin C");
      bigServo.write(135);
      break;

    case 'D': // Category 4
      Serial.println("Action: Bin D");
      bigServo.write(180);
      break;
      
    default:
      Serial.println("Unknown Category. Returning to Neutral.");
      bigServo.write(90);
      return; // Exit function if unknown
  }

  // Allow time for Big Servo to reach position
  delay(1000); 

  // 2. Open the Lid (Mini Servo)
  miniServo.write(90); // Open
  delay(2000);         // Wait for trash to fall

  // 3. Close the Lid
  miniServo.write(0);  // Close
  delay(1000);

  // 4. Return Big Servo to Neutral
  bigServo.write(90);
}
