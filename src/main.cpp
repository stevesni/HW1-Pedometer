#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>

Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16 canvas(240, 135);


void setReports();
#define BNO8X_RESET -1 //Reset pin not wired

bool firstRead = true;
unsigned long curTime = 0;
unsigned long prevTime = 0;
unsigned long stepCount = 0;
float gravity = 0;
float motion = 0;
float smoothed = 0;
float cutFreq = 3.5; //Higher freq oscillation removed
float cutTimeConst = 0;
float mean = 0;
float variance = 0;
float floorCut = 0.8;
float prev1 = 0;
float prev2 = 0;
float strideLength = 0.76; //In meters
float odometer = 0;

enum menuPages {
  steps,
  dist,
  stride,
  raw,
  mCount
};

menuPages menuMode = steps;

// Button Debounce and Flags
volatile long prevChangeTimeUp = 0;
volatile long prevChangeTimeDown = 0;
volatile long prevMenuTime = 0;
long debounceTime = 50;

volatile bool changeButtonFlagUp = false;
volatile bool changeButtonFlagDown = false;
volatile bool menuButtonFlag = false;


// Interrupt Handlers
void IRAM_ATTR buttonToChangeThingsDown() {
  long now = millis();
  if (now > prevChangeTimeDown + debounceTime) {
    changeButtonFlagDown = true;
    prevChangeTimeDown = now;
  }
}
void IRAM_ATTR buttonToChangeThingsUp() {
  long now = millis();
  if (now > prevChangeTimeUp + debounceTime) {
    changeButtonFlagUp = true;
    prevChangeTimeUp = now;
  }
}
void IRAM_ATTR buttonToChangeMenu() {
  long now = millis();
  if (now > prevMenuTime + debounceTime) {
    menuButtonFlag = true;
    prevMenuTime = now;
  }
}

Adafruit_BNO08x bno08x(BNO8X_RESET);
sh2_SensorValue_t sensorValue;

// SETUP //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  display.init(135, 240);
  display.setRotation(1);
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, 1);
  display.fillScreen(ST77XX_BLACK);
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  delay(1000);
  canvas.setTextColor(ST77XX_GREEN);
  canvas.setTextSize(2);
  Serial.begin(115200);

  Serial.println("Adafruit BNO08x test!");
     // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
}
  Serial.println("BNO08x Found!");
  setReports();

  // Pin declaration & Interupts
  pinMode(0, INPUT_PULLUP); // Stride Decrease/Steps Reset
  pinMode(1, INPUT_PULLDOWN); // Stride Increase
  pinMode(2, INPUT_PULLDOWN); // Menu Navigation
  attachInterrupt(digitalPinToInterrupt(0), buttonToChangeThingsDown, RISING);
  attachInterrupt(digitalPinToInterrupt(1), buttonToChangeThingsUp, RISING);
  attachInterrupt(digitalPinToInterrupt(2), buttonToChangeMenu, RISING);
}


// LOOP //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }


  // Handle Menu Switch Button
  if (menuButtonFlag) {
    menuButtonFlag = false;
    menuMode = (menuPages)(((int)menuMode + 1) % (int)mCount);
    Serial.print("Moved to menu: ");
    Serial.println((int)menuMode);
  }

  // Handle Change Up Button
  if (changeButtonFlagUp && menuMode == stride) {
    changeButtonFlagUp = false;
    strideLength += 0.01;
  }

  // Handle Change Down Button
  if (changeButtonFlagDown && menuMode == stride){
    changeButtonFlagDown = false;
    strideLength -= 0.01;
  }

  // Handle Count Reset Button
  if(changeButtonFlagDown && (menuMode == steps || menuMode == dist)){
    changeButtonFlagDown = false;
    stepCount = 0;
    odometer = 0;
  }



  // Data collection
  float x = sensorValue.un.accelerometer.x;
  float y =sensorValue.un.accelerometer.y;
  float z = sensorValue.un.accelerometer.z;

  //Prelim calculations & time stamping
  curTime = millis();
  float accel = sqrt(x*x + y*y + z*z); // Acceleration magnitude
  float deltaT = (curTime - prevTime)/1000.0; //Time delta converted to seconds
  float gravAvgCoef = deltaT/(1+deltaT); //Gravity moving avg coeff 1s storage

  prevTime = curTime; // Roll over time vars

  // Set first read to equal gravity
  if (firstRead){
    gravity = accel; // Set gravity to accel mag
    firstRead = false;
    return;

  // Compute all other readings
  } else {
    motion = accel - gravity; // Remove gravity
    gravity = gravity + gravAvgCoef * motion; // Gravity moving avg
    
    //Smoothing
    cutTimeConst = deltaT/((1/(2*M_PI*cutFreq))+deltaT); //Time const from cutoff freq
    smoothed = smoothed + cutTimeConst*(motion-smoothed); //Smoothed out Motion
    
    //Step identifying/filtering
    float threshCoef =  deltaT/(2.0+deltaT); //Step threshold coef w/ 2s storage
    mean = mean + threshCoef*(smoothed - mean); //Smoothed moving avg
    float diff = smoothed - mean; 
    variance = variance + threshCoef*(diff*diff-variance); 
    float thresh = mean + 0.7*sqrt(variance); 

    if (thresh < floorCut){
      thresh = floorCut;
    }

    if (prev1 > prev2 && prev1 >= smoothed && prev1>thresh){
      stepCount++;
      odometer += strideLength;
    }

    prev2 = prev1;
    prev1 = smoothed;

    // Serial printing
    Serial.print("GRAVITY: ");
    Serial.print(gravity);
    Serial.print("  MOTION: ");
    Serial.print(motion);
    Serial.print("  SMOOTH: ");
    Serial.print(smoothed);
    Serial.print("  THRESH: ");
    Serial.print(thresh);
    Serial.print("  STEPS: ");
    Serial.println(stepCount);


    canvas.fillScreen(ST77XX_BLUE);
    canvas.setCursor(0,20);

    // Printing to screen
    switch (menuMode){

      case steps:
      canvas.print("MENU: ");
      canvas.println("STEPS");

      canvas.print("STEP COUNT: ");
      canvas.println(stepCount);
      break;

      case dist:
      canvas.print("MENU: ");
      canvas.println("DISTANCE");

      canvas.print(odometer);
      canvas.println(" Meters");
      canvas.print(odometer*3.28084);
      canvas.println(" Feet");
      canvas.print(odometer/1000);
      canvas.println("km");
      canvas.print(odometer/1609.344);
      canvas.println("mi");
      break;

      case stride:
      canvas.print("MENU: ");
      canvas.println("STRIDE");

      canvas.print("Stride: ");
      canvas.print(strideLength);
      canvas.println(" m");
      break;

      case raw: 
      canvas.print("MENU: ");
      canvas.println("RAW ACCEL");

      canvas.print("X: ");
      canvas.println(x);
      canvas.print("Y: ");
      canvas.println(y);
      canvas.print("Z: ");
      canvas.println(z);
      canvas.print("MAG: ");
      canvas.println(accel);
      break;
    }
    display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas.width(), canvas.height());
  }

}


void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  } else {
    Serial.println("Set accelerometer report... success!");
  }
}