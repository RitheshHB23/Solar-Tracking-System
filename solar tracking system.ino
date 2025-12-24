#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// ---------------- Servo Setup ----------------
Servo servoX;     // East-West rotation
Servo servoY;     // Up-Down rotation

int posX = 80;    // Default EAST tilt
int posY = 90;    // Neutral vertical position

// --------------- LDR Pins ---------------------
int TL = A0;  // Top Left
int TR = A1;  // Top Right
int BL = A2;  // Bottom Left
int BR = A3;  // Bottom Right

// --------------- Solar Tracking Settings ---------------
int sensitivity = 20;
int stepSize = 1;
int sunlightThreshold = 150;     // below this = night time
unsigned long lastAdjustTime = 0;
unsigned long adjustInterval = 3600000; // 1 hour (in ms)

// --------------- MPU6050 --------------------
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

// If panel moved accidentally → auto correct back
int stabThreshold = 2000;

void setup() {
  servoX.attach(9);
  servoY.attach(10);

  servoX.write(posX);
  servoY.write(posY);

  Wire.begin();
  mpu.initialize();

  delay(1500);
}

void loop() {

  // =======================================================
  // 1. READ LDR VALUES
  // =======================================================
  int vTL = analogRead(TL);
  int vTR = analogRead(TR);
  int vBL = analogRead(BL);
  int vBR = analogRead(BR);

  int avgLight = (vTL + vTR + vBL + vBR) / 4;

  // =======================================================
  // 2. NIGHT MODE → Go to default EAST position
  // =======================================================
  if (avgLight < sunlightThreshold) {
    posX = 80;     // EAST slightly
    posY = 90;     // middle
    servoX.write(posX);
    servoY.write(posY);
    delay(500);
    return;        // stop tracking in night
  }

  // =======================================================
  // 3. LOW LIGHT (CLOUDY / RAINY) → Adjust once per hour
  // =======================================================
  if (avgLight < 300) {
    if (millis() - lastAdjustTime < adjustInterval) {
      return; // wait 1 hour before adjusting again
    }
    lastAdjustTime = millis(); 
  }

  // =======================================================
  // 4. SOLAR TRACKING (LDR)
  // =======================================================
  int topAvg = (vTL + vTR) / 2;
  int bottomAvg = (vBL + vBR) / 2;
  int leftAvg = (vTL + vBL) / 2;
  int rightAvg = (vTR + vBR) / 2;

  int verticalError = topAvg - bottomAvg;
  int horizontalError = leftAvg - rightAvg;

  if (abs(verticalError) > sensitivity) {
    if (verticalError < 0) posY += stepSize;  
    else posY -= stepSize; 
  }

  if (abs(horizontalError) > sensitivity) {
    if (horizontalError < 0) posX += stepSize;  
    else posX -= stepSize; 
  }

  // LIMITS
  posX = constrain(posX, 20, 160);
  posY = constrain(posY, 70, 160);

  servoX.write(posX);
  servoY.write(posY);

  // =======================================================
  // 5. MPU6050 STABILITY CHECK
  // =======================================================
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // if panel gets pushed / shaken → return to last stable position
  if (abs(gx) > stabThreshold || abs(gy) > stabThreshold) {
    servoX.write(posX);
    servoY.write(posY);
  }

  delay(50);
}

