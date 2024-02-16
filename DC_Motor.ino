#include "TimerThree.h"     // for the LM393 Sensor
#include <LiquidCrystal.h>  // for the LCD display
#include <Wire.h>           // for the MPU6050 (I2C comms)
#include "IRremote.h"       // for the IR Receiver

#define pin1 3             // IR speed sensor #1
#define pin2 2             // IR speed sensor #2
#define trig 18             // for distance sensor
#define echo 19             // for distance sensor

int receiver = 37; // Signal Pin of IR receiver to Arduino Digital Pin 0
IRrecv irrecv(receiver);     // create instance of 'irrecv'
uint32_t last_decodedRawData = 0; // variable used to store the last decodedRawData

// L298N motors
int motor_left[] = {38, 39};
int motor_right[] = {40, 41};
int leftSpeed = 44;
int rightSpeed = 45;

// LCD Display
LiquidCrystal lcd(10, 8, 7, 6, 5, 4);  // Creates an LCD object. Parameters: (rs, enable, d4, d5, d6, d7)
int contrast = 11;

// Necessary variables for LM393;
int interval, wheel, counter1, counter2;  // Separate counters for each sensor
unsigned long previousMicros, usInterval, calc;

// Necessary variables for HC-SR04
long duration, distance;

// Necessary variables for MPU6050
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160; //min PWM value at which motor moves
float currentAngle; //due to how I orientated my MPU6050 on my car, angle = yaw
float targetAngle = 0;
int leftSpeedVal, rightSpeedVal;

// PID variables
double prevError = 0;
double integral = 0;
unsigned long prevTime = 0;
double error;

// PID Constants (Tune these values)
double Kp = 8.0;
double Ki = 0.1;
double Kd = 0.01;

void setupMPU6050() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
}
void setupHCSR04() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}
void setupLM393() {
  // Set up LM393 Optointerrupter sensors
  counter1 = 0;  // setting counter1 to 0 for the first sensor
  counter2 = 0;  // setting counter2 to 0 for the second sensor

  interval = 1;  // 1 second interval
  wheel = 20;    // number of encoder disc holes

  calc = 60 / interval;             // calculate interval to one minute
  usInterval = interval * 1000000;  // convert interval to microseconds

  pinMode(pin1, INPUT);  // setting pin 20 as input for the first sensor
  pinMode(pin2, INPUT);  // setting pin 21 as input for the second sensor

  Timer3.initialize(usInterval);                                 // initialize timer with interval time
  attachInterrupt(digitalPinToInterrupt(pin1), count1, CHANGE);  // Add interrupt for the first sensor
  attachInterrupt(digitalPinToInterrupt(pin2), count2, CHANGE);  // Add interrupt for the second sensor

  Timer3.attachInterrupt(output);  // executes output after interval time              
}
void setupLCD() {
  lcd.begin(16, 2);  // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display }
  pinMode(11, OUTPUT);
  analogWrite(11, 100);
}
void setupMotors() {
  for (int i = 0; i < 2; i++) {
    pinMode(motor_left[i], OUTPUT);
    pinMode(motor_right[i], OUTPUT);
    pinMode(leftSpeed, OUTPUT);
    pinMode(rightSpeed, OUTPUT);
  }
}

// Encoders LM393
// counts holes on disc for the first sensor (with filter)
void count1() {
  if (micros() - previousMicros >= 700) {
    counter1++;
    previousMicros = micros();
  }
}

// counts holes on disc for the second sensor (with filter)
void count2() {
  if (micros() - previousMicros >= 700) {
    counter2++;
    previousMicros = micros();
  }
}

// output to serial for both sensors
int globalSpeed1, globalSpeed2;
void output() {
  Timer3.detachInterrupt();  // interrupts the timer
  Serial.print("Left Counter: ");
  int speed1 = ((counter1)*calc) / wheel;
  Serial.println(counter1);
  globalSpeed1 = speed1;

  Serial.print("Right Counter: ");
  int speed2 = ((counter2)*calc) / wheel;
  Serial.println(counter2);
  globalSpeed2 = speed2;

                      // resetting the counter for the first sensor
                     // resetting the counter for the second sensor
  Timer3.attachInterrupt(output);  // restarts the timer for output
}

// LCD Screen
// prints data to the LCD Screen
void printData(int dis, int rpm1, int rpm2) {
  lcd.setCursor(0, 0);
  lcd.print("Distance(cm)=");

  if (dis < 500) {
    if (dis >= 10) {
      lcd.print(dis);
      lcd.print(" ");
    } else {
      lcd.print("0");
      lcd.print(dis);
      lcd.print(" ");
    }
  }

  lcd.setCursor(13, 1);

  lcd.setCursor(0, 1);
  lcd.print("rpm1=");
  lcd.print(rpm1);
  lcd.print("rpm2=");
  lcd.print(rpm2);
}

// MPU6050 Methods (don't worry, this is just behind-the-scenes stuff)
void calculate_IMU_error() {
  // We can call this function in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor

  /*
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  */

}

int timeElapsed = micros() / 1000000;

// Configures the pins to go forward
void moveForward() {
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], HIGH);
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], HIGH);
}

// Configures the pins to turn left
void turnLeft() {
  digitalWrite(motor_left[0], HIGH);
  digitalWrite(motor_left[1], LOW);
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], HIGH);
}

// Configures the pins to turn right
void turnRight() {
  digitalWrite(motor_right[0], HIGH);
  digitalWrite(motor_right[1], LOW);
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], HIGH);
}

// Changes the target angle depending on which direction is desired
void changeTargetAngle(String direction) {
  if (direction.equals("right")) {
    targetAngle -= 90;
  } else if (direction.equals("left")) {
    targetAngle += 90;
  }
}

boolean isDriving = false;
// Sets speed to 0
void stopCar() {
  digitalWrite(motor_left[0], LOW);
  digitalWrite(motor_left[1], LOW);
  digitalWrite(motor_right[0], LOW);
  digitalWrite(motor_right[1], LOW);
  leftSpeedVal = 0; 
  rightSpeedVal = 0; 
}

// Sets speed to default (~equilibrium)
void startCar() {
  leftSpeedVal = 215;
  rightSpeedVal = 255;
}

// Distance Sensor
void Distance() {
  // Distance Sensor
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = duration / 58; // THIS IS THE DISTANCE DATA FOR THE ROBOT!!!
}

void readMPU6050() {
// === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 2.62; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 5.40; // AccErrorY ~(-1.58)

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  currentAngle = yaw;
}

double correction() {
  // Calculate error
  error = currentAngle - targetAngle;
  if (error <= -180) {
    error += 360;
  }

  // Calculate time elapsed since the last iteration
  unsigned long currentTime = millis();
  double deltaTime = (currentTime - prevTime) / 1000.0; // Convert milliseconds to seconds

  // Proportional term
  double P = Kp * error;

  // Integral term
  integral += error * deltaTime; // Approximate integral as the sum of errors * time
  double I = Ki * integral;

  // Derivative term
  double D = Kd * ((error - prevError) / deltaTime); // Approximate derivative as (change in error / time)
  prevError = error;

  // Overall correction
  double correction = P + I + D;
  return correction;
}

int currentStep = 1;
int startCounter;
double rotError;
void moveOneSquare(int step) {
  if (currentStep == step) {
    startCounter = counter2;
    isDriving = true;
    currentStep++;
  }

  rotError = currentAngle - targetAngle;
  if (rotError >= 350) {
    rotError -= 360;
  }
  if (rotError <= -350) {
    rotError += 360;
  }

  if (abs(rotError) < 15) {
    if (counter2 - startCounter < 102) { // approximately how many interrupts needed for 50 cm
      moveForward();
      startCar();
      isDriving = true;
    } else {
      stopCar();
      isDriving = false;
    }
  }
  
  //Serial.print("startCounter: ");
  //Serial.println(startCounter);
}


boolean targetAngleChanged = false;
boolean isTurning = false;
int turnCount = 1; 
int leftTurnSpeed = 130;
int rightTurnSpeed = 170;
void turn(String direction, int targetTurnCount, int step) {
  if (currentStep == step && isDriving == false) {
    if (turnCount == targetTurnCount) {
      targetAngleChanged = false;
      turnCount++;
    }

    if (targetAngleChanged == false) {
      changeTargetAngle(direction);
      targetAngleChanged = true;
    }

    if (currentAngle - targetAngle > 10) {
        leftSpeedVal = leftTurnSpeed;
        rightSpeedVal = rightTurnSpeed;
        turnRight();
    } else if (currentAngle - targetAngle > -330 && currentAngle - targetAngle < 0) {
        leftSpeedVal = leftTurnSpeed;
        rightSpeedVal = rightTurnSpeed;
        turnRight();
    } else { // once you're done turning
        stopCar();
        currentStep++;
    } 
  }
}

void executeSequence() {
  // Perform sequence of movements here
  moveOneSquare(1);
  turn("right", 1, 2);
  moveOneSquare(3);
  turn("left", 2, 4);
  moveOneSquare(5);
  turn("right", 3, 6);
  moveOneSquare(7);
  turn("left", 4, 8);
  moveOneSquare(9);
  turn("left", 5, 10);
  moveOneSquare(11);
  turn("left", 6, 12);
  moveOneSquare(13);
  turn("right", 7, 14);
  moveOneSquare(15);
  turn("left", 8, 16);
  moveOneSquare(17);
  moveOneSquare(18);
  /*Serial.print("Current step: ");
  Serial.println(currentStep);
  Serial.print("Target angle: ");
  Serial.println(targetAngle);
  Serial.print("Current angle: ");
  Serial.println(currentAngle);*/
  Serial.print("Current error: ");
  Serial.println(rotError);
}

void setup() {
  Serial.begin(38400);
  setupMPU6050();
  setupHCSR04();
  setupMotors();
  setupLM393();
  setupLCD();
  irrecv.enableIRIn();

  calculate_IMU_error();
  prevTime = millis(); // Initialize the previous time
}

void loop() {

  readMPU6050();

  if (currentAngle >= 355) {
    currentAngle -= 360;
  }

  if (currentAngle <= -355) {
    currentAngle += 360;
  }

  if (targetAngle <= -355) {
    targetAngle += 360;
  }

  if (targetAngle >= 360) {
    targetAngle -= 360;
  }


  Distance();

  // Output data to LCD
  printData(distance, globalSpeed1, globalSpeed2);
  
  // Output current angle to serial 
  //Serial.print("Current Angle: ");
  //Serial.println(currentAngle);

  // Adjust left motor speed
  if (isDriving == true) {
    leftSpeedVal += correction();
    leftSpeedVal = constrain(leftSpeedVal, minSpeed, maxSpeed);
    rightSpeedVal = constrain(rightSpeedVal, minSpeed, maxSpeed);
    // Ensure right motor speed is within bounds
  }

  // Apply motor speeds
  analogWrite(leftSpeed, leftSpeedVal);
  analogWrite(rightSpeed, rightSpeedVal);

  // Update the previous time for correction()
  prevTime = currentTime;

  executeSequence();
}
