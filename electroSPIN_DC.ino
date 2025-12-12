#include <AccelStepper.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <WiFi.h>
#include <DHT.h>

WiFiClientSecure client;
UniversalTelegramBot* bot = nullptr;

bool telegramConnected = false;

String telegramToken = "";
String telegramChatID = "";

#define DHTPIN 22        // Signal pin for DHT21
#define DHTTYPE DHT21    // DHT21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
unsigned long lastDHTRead = 0;
const unsigned long dhtInterval = 2000; // Read every 2 seconds

float lastTemp = NAN;
float lastHum = NAN;

// Stepper pin definitions
#define X_STEP_PIN 6
#define X_DIR_PIN 5
#define Y_STEP_PIN 8
#define Y_DIR_PIN 7
#define DRUM_IN1 10  //STEP_PIN kangal
#define DRUM_IN2 9   //yoktu kangal
#define DRUM_ENA 15
#define PUMP1_STEP_PIN 12
#define PUMP1_DIR_PIN 11
#define PUMP2_STEP_PIN 14
#define PUMP2_DIR_PIN 13

const int X_LIMIT = 2;
const int Y_LIMIT = 3;
const int PUMP1_LIMIT = 4;
const int PUMP2_LIMIT = 21;
const int fan_relay = 18;
const int lamp_relay = 19;
const int lamp2_relay = 20;

///drum speed constants
const int MOTOR_MAX_RPM = 500;
const int MOTOR_MIN_RPM = 1;
const int MOTOR_MIN_PWM = 0;
const int MOTOR_MAX_PWM = 255;

//voltage generator
#define MOTOR_IN1  28
#define MOTOR_IN2  27
#define MOTOR_ENA  26  // PWM capable pin

// Mechanics
float pitchX = 8.0;
int pprX = 400;
float pitchY = 8.0;
int pprY = 400;
float pitchPUMP1 = 1.2; 
int pprPUMP1 = 1600;
float pitchPUMP2 = 1.2;
int pprPUMP2 = 1600;
int pprDRUM = 400;

float stepsPerMM_Pump1 = pprPUMP1 / pitchPUMP1;
float stepsPerMM_Pump2 = pprPUMP2 / pitchPUMP2;
float stepsPerMM_X = pprX / pitchX;
float stepsPerMM_Y = pprY / pitchY;

// --- FIXED: Independent Pump State Variables ---
bool pumping1 = false; 
bool pumping2 = false; 
// -----------------------------------------------

// Steppers
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperPUMP1(AccelStepper::DRIVER, PUMP1_STEP_PIN, PUMP1_DIR_PIN);
AccelStepper stepperPUMP2(AccelStepper::DRIVER, PUMP2_STEP_PIN, PUMP2_DIR_PIN);

// Control variables
String command = "";
int x_speed = 1000;
int y_speed = 1000;
int pump1_speed = 6400;
int pump2_speed = 6400;
int drum_speed = 0;
int drum_direction = 1;

bool homingX = false;
bool homingY = false;
bool homingPUMP1 = false;
bool homingPUMP2 = false;

bool manualX = false;
bool manualY = false;
bool manualPUMP1 = false;
bool manualPUMP2 = false;

int manualXDir = 1;
int manualYDir = 1;
int manualPump1Dir = 1;
int manualPump2Dir = 1;

// Y looping control
bool loopingY = false;
float yLoopStart = 0;
float yLoopEnd = 0;
int yLoopCount = 0;
int yLoopCurrentCycle = 0;
bool yGoingToEnd = true;

// Forward declaration
String readNextionCommand();
void handleNextionCommand();
bool connectToWiFi(const String& ssid, const String& password);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(X_LIMIT, INPUT_PULLUP);
  pinMode(Y_LIMIT, INPUT_PULLUP);
  pinMode(PUMP1_LIMIT, INPUT_PULLUP);
  pinMode(PUMP2_LIMIT, INPUT_PULLUP);
  
  pinMode(fan_relay, OUTPUT);
  pinMode(lamp_relay, OUTPUT);
  pinMode(lamp2_relay, OUTPUT);
  
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(fan_relay, LOW);
  digitalWrite(lamp_relay, LOW);
  digitalWrite(lamp2_relay, LOW);

  Serial.begin(9600);
  Serial1.begin(9600);

  stepperX.setAcceleration(10000);
  stepperX.setMaxSpeed(x_speed);
  stepperX.setCurrentPosition(0);

  stepperY.setAcceleration(10000);
  stepperY.setMaxSpeed(y_speed);
  stepperY.setCurrentPosition(0);

  stepperPUMP1.setAcceleration(10000);
  stepperPUMP1.setMaxSpeed(pump1_speed);
  stepperPUMP1.setCurrentPosition(0);

  stepperPUMP2.setAcceleration(10000);
  stepperPUMP2.setMaxSpeed(pump2_speed);
  stepperPUMP2.setCurrentPosition(0);

  pinMode(DRUM_IN1, OUTPUT);
  pinMode(DRUM_IN2, OUTPUT);
  pinMode(DRUM_ENA, OUTPUT);
    
  // Default to off
  digitalWrite(DRUM_IN1, LOW);
  digitalWrite(DRUM_IN2, LOW);
  analogWrite(DRUM_ENA, 0);
  
  Serial.println("System initialized.");

  dht.begin();

  //voltage generator:
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);
  
  // Default to off
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
}

void loop() {
  // ---------------------------------------------------------
  // HOMING ROUTINES
  // ---------------------------------------------------------
  if (homingX) {
    stepperX.setMaxSpeed(-pprX*2);
    stepperX.setSpeed(-pprX*2);
    stepperX.runSpeed();
    if (digitalRead(X_LIMIT) == HIGH) {
      stepperX.setCurrentPosition(0);
      stepperX.setSpeed(0);
      homingX = false;
      Serial.println("X homed.");
      Serial1.print("tsw btn_x_send,1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("nmb_x.val=0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("btn_x_send.aph=127"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("click btn_x_stop,0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("lbl_serial.txt=\"X axis homed.\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("val_home_x.val=1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
    }
  }

  if (homingY) {
    stepperY.setMaxSpeed(-pprY*2);
    stepperY.setSpeed(-pprY*2);
    stepperY.runSpeed();
    if (digitalRead(Y_LIMIT) == HIGH) {
      stepperY.setCurrentPosition(0);
      stepperY.setSpeed(0);
      homingY = false;
      Serial.println("Y homed.");
      Serial1.print("tsw btn_hmg_send,1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("btn_hmg_send.aph=127"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("nmb_hmg.val=0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("click btn_hmg_stop,0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("lbl_serial.txt=\"Homogenizator homed.\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("val_home_hmg.val=1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
    }
  }

  if (homingPUMP1) {
    stepperPUMP1.setMaxSpeed(-pprPUMP1*3);
    stepperPUMP1.setSpeed(-pprPUMP1*3);
    stepperPUMP1.runSpeed();
    if (digitalRead(PUMP1_LIMIT) == HIGH) {
      stepperPUMP1.setCurrentPosition(0);
      stepperPUMP1.setSpeed(0);
      homingPUMP1 = false;
      Serial.println("PUMP1 homed.");
      Serial1.print("tsw btn_syr1_send,1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("btn_syr1_send.aph=127"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("click btn_syr1_stop,0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("lbl_serial.txt=\"Pump1 homed.\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("val_home_syr1.val=1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
    }
  }

  if (homingPUMP2) {
    stepperPUMP2.setMaxSpeed(-pprPUMP2*3);
    stepperPUMP2.setSpeed(-pprPUMP2*3);
    stepperPUMP2.runSpeed();
    if (digitalRead(PUMP2_LIMIT) == HIGH) {
      stepperPUMP2.setCurrentPosition(0);
      stepperPUMP2.setSpeed(0);
      homingPUMP2 = false;
      Serial.println("PUMP2 homed.");
      Serial1.print("tsw btn_syr2_send,1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("btn_syr2_send.aph=127"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("click btn_syr2_stop,0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("lbl_serial.txt=\"Pump2 homed.\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      Serial1.print("val_home_syr2.val=1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
    }
  }

  // ---------------------------------------------------------
  // RUNNING ROUTINES (FIXED LOGIC)
  // ---------------------------------------------------------

  // --- X AXIS ---
  if (manualX) {
    if (!(stepperX.currentPosition() <= 0 && manualXDir < 0)) {
        stepperX.setMaxSpeed(pprX * manualXDir);
        stepperX.setSpeed(pprX * manualXDir);
        stepperX.runSpeed();
    }
  } else if (!homingX) {
    stepperX.run();
  }

  // --- Y AXIS ---
  if (manualY) {
    if (!(stepperY.currentPosition() <= 0 && manualYDir < 0)) {
        stepperY.setMaxSpeed(pprY * manualYDir);
        stepperY.setSpeed(pprY * manualYDir);
        stepperY.runSpeed();
    }
  } else if (!homingY) {
    stepperY.run();
  }

  // --- PUMP 1 (FIXED) ---
  if (manualPUMP1) {
    if (!(stepperPUMP1.currentPosition() <= 0 && manualPump1Dir < 0)) {
        stepperPUMP1.setMaxSpeed(pprPUMP1*2 * manualPump1Dir);
        stepperPUMP1.setSpeed(2*pprPUMP1 * manualPump1Dir);
        stepperPUMP1.runSpeed();
    }
  } else if (!homingPUMP1) {
    stepperPUMP1.run(); // Only runs Pump 1
  }

  // --- PUMP 2 (FIXED) ---
  if (manualPUMP2) {
    if (!(stepperPUMP2.currentPosition() <= 0)) {
        stepperPUMP2.setMaxSpeed(pprPUMP1*2 * manualPump2Dir); // Note: using pprPUMP1 scaling here per orig code, check if pprPUMP2 intended
        stepperPUMP2.setSpeed(2*pprPUMP2 * manualPump2Dir);
        stepperPUMP2.runSpeed();
    }
  } else if (!homingPUMP2) {
    stepperPUMP2.run(); // Only runs Pump 2
  }

  // --- Y Looping Logic ---
  if (loopingY && !stepperY.isRunning()) {
    if (yLoopCount != 0 && yLoopCurrentCycle >= yLoopCount) {
      loopingY = false;
      Serial.println("Y loop finished.");
    } else {
      long target = lround((yGoingToEnd ? yLoopEnd : yLoopStart) * stepsPerMM_Y);
      stepperY.moveTo(target);
      yGoingToEnd = !yGoingToEnd;
      if (!yGoingToEnd) yLoopCurrentCycle++;
      Serial.print("Y moving to ");
      Serial.println(target / stepsPerMM_Y);
    }
  }

  // Check Nextion
  if (Serial1.available()) {
    handleNextionCommand();
  }

  // DHT Sensor
  if (millis() - lastDHTRead > dhtInterval) {
    lastDHTRead = millis();
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    if (!isnan(temp) && !isnan(hum)) {
      bool shouldSend = false;
      if (lastTemp != temp || lastHum != hum) {
        shouldSend = true;
      }
      if (shouldSend) {
        lastTemp = temp;
        lastHum = hum;
        Serial1.print("txt_temp.txt=\""); Serial1.print(temp, 1); Serial1.print(" °C\"");
        Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        
        Serial1.print("txt_hum.txt=\""); Serial1.print(hum, 1); Serial1.print(" %\"");
        Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);

        Serial.print("Sent to Nextion: Temp = "); Serial.print(temp);
        Serial.print(" C, Humidity = "); Serial.println(hum);
      }
    } else {
      Serial.println("DHT read failed");
    }
  }

  // ---------------------------------------------------------
  // AUTOMATIC STOP LOGIC (INDEPENDENT)
  // ---------------------------------------------------------

  // Check if Pump 1 finished its programmed run
  if (!stepperPUMP1.isRunning() && pumping1) {
    pumping1 = false;
    
    // Stop auxiliaries (Optional: remove these lines if you want Pump 2 to keep Drum/KV running)
    analogWrite(MOTOR_ENA, 0); 
    digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
    drum_speed = 0;
    digitalWrite(DRUM_IN1, LOW); digitalWrite(DRUM_IN2, LOW);
    analogWrite(DRUM_ENA, 0);
    loopingY = false;
    digitalWrite(fan_relay, LOW);
    digitalWrite(lamp_relay, LOW);
    digitalWrite(lamp2_relay, LOW);
    // -----------------------------------------------------------------------------------------
    
    stepperPUMP1.stop();
    stepperPUMP1.setCurrentPosition(stepperPUMP1.currentPosition());
    Serial.println("PUMP1 finished/stopped");
    
    Serial1.print("lbl_serial.txt=\"Pump1 Finished!\""); 
    Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  }

  // Check if Pump 2 finished its programmed run
  if (!stepperPUMP2.isRunning() && pumping2) {
    pumping2 = false;
    
    // Stop auxiliaries
    analogWrite(MOTOR_ENA, 0); 
    digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, LOW);
    drum_speed = 0;
    digitalWrite(DRUM_IN1, LOW); digitalWrite(DRUM_IN2, LOW);
    analogWrite(DRUM_ENA, 0);
    loopingY = false;
    digitalWrite(fan_relay, LOW);
    digitalWrite(lamp_relay, LOW);
    digitalWrite(lamp2_relay, LOW);
    // -----------------------------------------------------------------------------------------

    stepperPUMP2.stop();
    stepperPUMP2.setCurrentPosition(stepperPUMP2.currentPosition());
    Serial.println("PUMP2 finished/stopped");
    
    Serial1.print("lbl_serial.txt=\"Pump2 Finished!\"");
    Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  }
}

// ---------------------------------------------------------
// WIFI HELPER
// ---------------------------------------------------------
bool connectToWiFi(const String& ssid, const String& password) {
  WiFi.disconnect(); 
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 10000; 

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(500);
  }
  return WiFi.status() == WL_CONNECTED;
}

// ---------------------------------------------------------
// NEXTION COMMAND HANDLING
// ---------------------------------------------------------
void handleNextionCommand() {
  command = readNextionCommand();
  Serial.print("Received: ");
  Serial.println(command);

  if (command == "100") {
    manualX = false;
    stepperX.setCurrentPosition(stepperX.currentPosition());
    float currentXPosMM = stepperX.currentPosition() / stepsPerMM_X; 
    Serial1.print("nmb_x.val=");  
    Serial1.print((int)currentXPosMM); 
    Serial1.write(0xff); Serial1.write(0xff); Serial1.write(0xff);
  }
  else if (command == "105") manualX = true, manualXDir = 1;
  else if (command == "106") manualX = true, manualXDir = -1;
  else if (command == "200") {
    manualY = false;
    loopingY = false;
    stepperY.setCurrentPosition(stepperY.currentPosition());
    float currentYPosMM = stepperY.currentPosition() / stepsPerMM_Y;
    Serial1.print("nmb_hmg.val=");
    Serial1.print((int)currentYPosMM-200); 
    Serial1.write(0xff); Serial1.write(0xff); Serial1.write(0xff);
  }
  else if (command == "205") manualY = true, manualYDir = 1;
  else if (command == "206") manualY = true, manualYDir = -1;
  
  // STOP PUMP 1
  else if (command == "400") {
    manualPUMP1 = false;
    stepperPUMP1.stop();
    stepperPUMP1.setCurrentPosition(stepperPUMP1.currentPosition());
    Serial.println("PUMP1 stopped manually");
    pumping1 = false; // Fixed variable name
    Serial1.print("lbl_serial.txt=\"Pump1 is stopped!\""); 
    Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF); 
  }
  
  // STOP PUMP 2
  else if (command == "500") {
    manualPUMP2 = false;
    stepperPUMP2.stop();
    stepperPUMP2.setCurrentPosition(stepperPUMP2.currentPosition());
    Serial.println("PUMP2 stopped manually");
    pumping2 = false; // Fixed variable name
    Serial1.print("lbl_serial.txt=\"Pump2 is stopped!\""); 
    Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF); 

    float currentPump1PosMM = stepperPUMP1.currentPosition() / stepsPerMM_Pump1; 
    // Send updated position to Nextion (Note: Original code sent Pump1 pos here, assuming correct?)
    Serial1.print("sld_syr1.val=");
    Serial1.print((int)currentPump1PosMM*10); 
    Serial1.write(0xff); Serial1.write(0xff); Serial1.write(0xff);
  }

  else if (command == "405") manualPUMP1 = true, manualPump1Dir = 1;
  else if (command == "406") manualPUMP1 = true, manualPump1Dir = -1;
  else if (command == "505") manualPUMP2 = true, manualPump2Dir = 1;
  else if (command == "506") manualPUMP2 = true, manualPump2Dir = -1;

  else if (command.startsWith("101")) {
    int colonIndex = command.indexOf(":", 3);
    float x_mm = command.substring(3, colonIndex).toFloat();
    float x_speed = command.substring(colonIndex + 1).toFloat();
    x_speed=x_speed*(pprX/60);
    long target = lround(x_mm * stepsPerMM_X);
    stepperX.setMaxSpeed(x_speed);  
    stepperX.moveTo(target);
    Serial.print("Moving X to "); Serial.print(x_mm); Serial.print(" mm at speed "); Serial.println(x_speed);
  }
  else if (command == "102") {
    Serial.println("Homing X...");
    homingX = true;
    Serial1.print("lbl_serial.txt=\"\"");
    Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  }
  else if (command.startsWith("201")) {
    int colonIndex = command.indexOf(":", 3);
    float y_mm = command.substring(3, colonIndex).toFloat();
    float y_speed = command.substring(colonIndex + 1).toFloat();
    y_speed=y_speed*(pprY/60);
    long target = lround(y_mm * stepsPerMM_Y);
    stepperY.setMaxSpeed(y_speed); 
    stepperY.moveTo(target);
    Serial.print("Moving Y to "); Serial.print(y_mm); Serial.print(" mm at speed "); Serial.println(y_speed);
  }
  else if (command == "202") {
    Serial.println("Homing Y...");
    homingY = true;
  }
  else if (command == "drumcw") {
    drum_direction = 1;
    drum_speed = 130;  
    digitalWrite(DRUM_IN1, HIGH);
    digitalWrite(DRUM_IN2, LOW);
    analogWrite(DRUM_ENA, drum_speed);
    Serial.println("Drum rotating CW at speed 200 PWM");
  } 
  else if (command == "drumccw") {
    drum_direction = -1;
    drum_speed = 130;
    digitalWrite(DRUM_IN1, LOW);
    digitalWrite(DRUM_IN2, HIGH);
    analogWrite(DRUM_ENA, drum_speed);
    Serial.println("Drum rotating CCW at speed 200 PWM");
  } 
  else if (command.startsWith("203")) {
    int firstColon = command.indexOf(":", 3);
    int secondColon = command.indexOf(":", firstColon + 1);
    int thirdColon = command.indexOf(":", secondColon + 1);

    yLoopStart = command.substring(3, firstColon).toFloat();
    yLoopEnd = command.substring(firstColon + 1, secondColon).toFloat();
    yLoopCount = command.substring(secondColon + 1, thirdColon).toInt();
    float yLoopSpeed = command.substring(thirdColon + 1).toFloat();
    yLoopSpeed=yLoopSpeed*(pprY/60);

    yLoopCurrentCycle = 0;
    yGoingToEnd = true;
    loopingY = true;

    stepperY.setMaxSpeed(yLoopSpeed); 
    stepperY.moveTo(lround(yLoopEnd * stepsPerMM_Y));

    Serial.print("Starting Y loop: ");
    Serial.print("Start = "); Serial.print(yLoopStart);
    Serial.print(", End = "); Serial.print(yLoopEnd);
    Serial.print(", Cycles = "); Serial.print(yLoopCount);
    Serial.print(", Speed = "); Serial.println(yLoopSpeed);
  } else if (command == "204") {
    loopingY = false;
    Serial.println("Y loop canceled.");
  } else if (command == "300") {
    drum_speed = 0;
    analogWrite(DRUM_ENA, 0); 
    Serial.println("Drum stopped");
  } else if (command.startsWith("301")) {
    float rpm = abs(command.substring(3).toFloat());
    rpm = constrain(rpm, 0, MOTOR_MAX_RPM);
    drum_direction = 1;
    if (rpm < MOTOR_MIN_RPM) {
        drum_speed = 0;
    } else {
        drum_speed = map(rpm, MOTOR_MIN_RPM, MOTOR_MAX_RPM, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    }
    if (drum_speed == 0) {
        analogWrite(DRUM_ENA, 0);
    } else {
        digitalWrite(DRUM_IN1, HIGH);
        digitalWrite(DRUM_IN2, LOW);
        analogWrite(DRUM_ENA, drum_speed);
    }
    Serial.print("Drum CW at RPM: "); Serial.print(rpm); Serial.print(", PWM: "); Serial.println(drum_speed);
  }
  else if (command.startsWith("302")) {
    float rpm = abs(command.substring(3).toFloat());
    rpm = constrain(rpm, 0, MOTOR_MAX_RPM);
    drum_direction = -1;
    if (rpm < MOTOR_MIN_RPM) {
        drum_speed = 0;
    } else {
        drum_speed = map(rpm, MOTOR_MIN_RPM, MOTOR_MAX_RPM, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
    }
    if (drum_speed == 0) {
        analogWrite(DRUM_ENA, 0);
    } else {
        digitalWrite(DRUM_IN1, LOW);
        digitalWrite(DRUM_IN2, HIGH);
        analogWrite(DRUM_ENA, drum_speed);
    }
    Serial.print("Drum CCW at RPM: "); Serial.print(rpm); Serial.print(", PWM: "); Serial.println(drum_speed);
  }
  else if (command == "402") {
    homingPUMP1 = true;
  } else if (command == "502") {
    homingPUMP2 = true;
  }else if (command == "fanoff") {
    digitalWrite(fan_relay, LOW);
  }else if (command == "fanon") {
    digitalWrite(fan_relay, HIGH);
  }else if (command == "lampoff") {
    digitalWrite(lamp_relay, LOW);
  }else if (command == "lampon") {
    digitalWrite(lamp_relay, HIGH);
  }else if (command == "lamp2off") {
    digitalWrite(lamp2_relay, LOW);
  }else if (command == "lamp2on") {
    digitalWrite(lamp2_relay, HIGH);
  } 
  
  // --------------------------------------------------------------------------------
  // PUMP 1 COMMAND START
  // --------------------------------------------------------------------------------
  else if (command.startsWith("403:")) {
    int colon1 = command.indexOf(":", 4);
    int colon2 = command.indexOf(":", colon1 + 1);
    int colon3 = command.indexOf(":", colon2 + 1);
    int colon4 = command.indexOf(":", colon3 + 1);

    if (colon1 != -1 && colon2 != -1 && colon3 != -1 && colon4 == -1) {

      float diameter = command.substring(4, colon1).toFloat() / 10.0; 
      int unit = command.substring(colon1 + 1, colon2).toInt();
      float desired_volume = command.substring(colon2 + 1, colon3).toFloat() / 10.0;
      float desired_flowrate = command.substring(colon3 + 1).toFloat() / 100.0;

      Serial.print("P1 Diameter: "); Serial.println(diameter);
      Serial.print("Unit: "); Serial.println(unit);
      Serial.print("Vol: "); Serial.println(desired_volume);
      Serial.print("Flow: "); Serial.println(desired_flowrate);

      if (diameter <= 0) { Serial.println("Error: Dia > 0"); return; }

      float steps_per_mm = pprPUMP1 / pitchPUMP1;
      float area_mm2 = 3.1416 * pow(diameter / 2.0, 2);
      float volume_mm3 = 0;
      if (unit == 0 || unit == 1) { volume_mm3 = desired_volume * 1000.0; } 
      else if (unit == 2 || unit == 3) { volume_mm3 = desired_volume; } 
      else { Serial.println("Invalid unit."); return; }

      float distance_mm = volume_mm3 / area_mm2;
      float flowrate_mm3_per_s = 0;
      if (unit == 0) { flowrate_mm3_per_s = (desired_flowrate * 1000.0) / 60.0; } 
      else if (unit == 1) { flowrate_mm3_per_s = (desired_flowrate * 1000.0) / 3600.0; } 
      else if (unit == 2) { flowrate_mm3_per_s = desired_flowrate / 60.0; } 
      else if (unit == 3) { flowrate_mm3_per_s = desired_flowrate / 3600.0; }

      float speed_mm_per_s = flowrate_mm3_per_s / area_mm2;
      long steps_to_move = distance_mm * steps_per_mm;
      float steps_per_second = speed_mm_per_s * steps_per_mm;

      Serial.print("Steps: "); Serial.println(steps_to_move);
      Serial.print("Speed: "); Serial.println(steps_per_second);

      if (steps_per_second < 1.0) {
        Serial.println("Error: Speed too low.");
        Serial1.print("lbl_serial.txt=\"Calc error! Check values!\""); 
        Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF); 
        return;
      }
      Serial1.print("lbl_serial.txt=\"P1 Started!\""); 
      Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF); 
      
      stepperPUMP1.setMaxSpeed(steps_per_second);
      stepperPUMP1.move(steps_to_move);
      pumping1 = true; // Use P1 specific flag
    }
  }

  // --------------------------------------------------------------------------------
  // PUMP 2 COMMAND START
  // --------------------------------------------------------------------------------
  else if (command.startsWith("503:")) { // ADDED COLON FOR SAFETY
    int colon1 = command.indexOf(":", 4);
    int colon2 = command.indexOf(":", colon1 + 1);
    int colon3 = command.indexOf(":", colon2 + 1);
    int colon4 = command.indexOf(":", colon3 + 1);

    if (colon1 != -1 && colon2 != -1 && colon3 != -1 && colon4 == -1) {

      float diameter = command.substring(4, colon1).toFloat() / 10.0; 
      int unit = command.substring(colon1 + 1, colon2).toInt();
      float desired_volume = command.substring(colon2 + 1, colon3).toFloat() / 10.0; 
      float desired_flowrate = command.substring(colon3 + 1).toFloat() / 100.0; 

      Serial.print("P2 Diameter: "); Serial.println(diameter);
      Serial.print("Unit: "); Serial.println(unit);
      Serial.print("Vol: "); Serial.println(desired_volume);
      Serial.print("Flow: "); Serial.println(desired_flowrate);

      if (diameter <= 0) { Serial.println("Error: Dia > 0"); return; }

      float steps_per_mm = pprPUMP2 / pitchPUMP2;
      float area_mm2 = 3.1416 * pow(diameter / 2.0, 2);
      float volume_mm3 = 0;
      if (unit == 0 || unit == 1) { volume_mm3 = desired_volume * 1000.0; } 
      else if (unit == 2 || unit == 3) { volume_mm3 = desired_volume; } 
      else { Serial.println("Invalid unit."); return; }

      float distance_mm = volume_mm3 / area_mm2;
      float flowrate_mm3_per_s = 0;
      if (unit == 0) { flowrate_mm3_per_s = (desired_flowrate * 1000.0) / 60.0; } 
      else if (unit == 1) { flowrate_mm3_per_s = (desired_flowrate * 1000.0) / 3600.0; } 
      else if (unit == 2) { flowrate_mm3_per_s = desired_flowrate / 60.0; } 
      else if (unit == 3) { flowrate_mm3_per_s = desired_flowrate / 3600.0; }

      float speed_mm_per_s = flowrate_mm3_per_s / area_mm2;
      long steps_to_move = distance_mm * steps_per_mm;
      float steps_per_second = speed_mm_per_s * steps_per_mm;

      Serial.print("Steps: "); Serial.println(steps_to_move);
      Serial.print("Speed: "); Serial.println(steps_per_second);

      if (steps_per_second < 1.0) {
        Serial.println("Error: Speed too low.");
        Serial1.print("lbl_serial.txt=\"Calc error! Check values!\""); 
        Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF); 
        return;
      }
      Serial1.print("lbl_serial.txt=\"P2 Started!\""); 
      Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF); 
      
      stepperPUMP2.setMaxSpeed(steps_per_second);
      stepperPUMP2.move(steps_to_move);
      pumping2 = true; // Use P2 specific flag
    }
  }

  else if (command.startsWith("wifi!")) {
    int idx1 = command.indexOf('!', 5);
    int idx2 = command.indexOf('!', idx1 + 1);
    int idx3 = command.indexOf('!', idx2 + 1);
    
    if (idx1 != -1 && idx2 != -1 && idx3 != -1) {
      String ssid = command.substring(5, idx1);
      String password = command.substring(idx1 + 1, idx2);
      telegramToken = command.substring(idx2 + 1, idx3);
      telegramChatID = command.substring(idx3 + 1);

      Serial.print("Trying WiFi: "); Serial.println(ssid);

      if (connectToWiFi(ssid, password)) {
        Serial.println("WiFi connected.");
        Serial1.print("btn_wifi.val=1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("tsw btn_wifi,1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("btn_wifi.aph=127"); 
        Serial1.print("tsw b1,1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("b1.aph=127"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("val_wifi.val=1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("lbl_serial.txt=\"Wi-Fi Connected!\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);

        client.setInsecure();  
        if (bot != nullptr) { delete bot; bot = nullptr; }
        bot = new UniversalTelegramBot(telegramToken, client);

        if (bot->sendMessage(telegramChatID, "Optosense-Bot connected successfully!", "")) {
          Serial.println("Telegram message sent.");
          telegramConnected = true;
          Serial1.print("lbl_serial.txt=\"Telegram is connected!\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        } else {
          Serial.println("Telegram message failed.");
          telegramConnected = false;
          Serial1.print("lbl_serial.txt=\"Telegram is pending!\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        }
      } else {
        Serial.println("WiFi connection failed.");
        Serial1.print("btn_wifi.val=0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("tsw btn_wifi,1"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("btn_wifi.aph=127"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("val_wifi.val=0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
        Serial1.print("lbl_serial.txt=\"WiFi connection failed!\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
      }
    } else {
      Serial.println("Malformed wifi command.");
    }
  }
  else if (command == "wifioff") {
    WiFi.disconnect(true);
    telegramConnected = false;
    Serial.println("WiFi and Telegram bot disconnected.");
    Serial1.print("lbl_serial.txt=\"WiFi and Telegram disconnected!\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF); 
    Serial1.print("btn_wifi.val=0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
    Serial1.print("tsw btn_wifi,0"); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  }
  else if (command.startsWith("pprx")) {
    float newPpr = command.substring(4).toFloat();
    if (newPpr > 0) { pprX = newPpr; Serial.print("pprX: "); Serial.println(pprX); }
  }
  else if (command.startsWith("ppry")) {
    float newPpr = command.substring(4).toFloat();
    if (newPpr > 0) { pprY = newPpr; Serial.print("pprY: "); Serial.println(pprY); }
  }
  else if (command.startsWith("pprpump1")) {
    float newPpr = command.substring(8).toFloat();
    if (newPpr > 0) { pprPUMP1 = newPpr; Serial.print("pprPUMP1: "); Serial.println(pprPUMP1); }
  }
  else if (command.startsWith("pprpump2")) {
    float newPpr = command.substring(8).toFloat();
    if (newPpr > 0) { pprPUMP2 = newPpr; Serial.print("pprPUMP2: "); Serial.println(pprPUMP2); }
  }
  else if (command.startsWith("kv:")) {
    int value = command.substring(3).toInt(); 
    value=value*2.55;
    value = constrain(value, 0, 255);
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, value); 
    Serial.print("Voltage generator set to PWM value: "); Serial.println(value);
  }
  else if (command.startsWith("reset")) {
    command = "";
    x_speed = 1000; y_speed = 1000; pump1_speed = 6400; pump2_speed = 6400;
    drum_speed = 0; drum_direction = 1;
    homingX = false; homingY = false; homingPUMP1 = false; homingPUMP2 = false;
    manualX = false; manualY = false; manualPUMP1 = false; manualPUMP2 = false;
    manualXDir = 1; manualYDir = 1; manualPump1Dir = 1; manualPump2Dir = 1;

    loopingY = false; yLoopStart = 0; yLoopEnd = 0; yLoopCount = 0; yLoopCurrentCycle = 0; yGoingToEnd = true;

    stepperX.setAcceleration(10000); stepperX.setMaxSpeed(x_speed); stepperX.setCurrentPosition(0); stepperX.setSpeed(0);
    stepperY.setAcceleration(10000); stepperY.setMaxSpeed(y_speed); stepperY.setCurrentPosition(0); stepperY.setSpeed(0);
    stepperPUMP1.setAcceleration(10000); stepperPUMP1.setMaxSpeed(pump1_speed); stepperPUMP1.setCurrentPosition(0); stepperPUMP1.setSpeed(0);
    stepperPUMP2.setAcceleration(10000); stepperPUMP2.setMaxSpeed(pump2_speed); stepperPUMP2.setCurrentPosition(0); stepperPUMP2.setSpeed(0);

    digitalWrite(DRUM_IN1, LOW); digitalWrite(DRUM_IN2, LOW); analogWrite(DRUM_ENA, 0);
    digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW); analogWrite(MOTOR_ENA, 0); 
    pumping1 = false; pumping2 = false; // Reset flags

    Serial.print("Device is resetted");
  }
}

String readNextionCommand() {
  String result = "";
  uint8_t ffCount = 0;
  unsigned long lastByteTime = millis();
  while (millis() - lastByteTime < 5) {
    if (Serial1.available()) {
      uint8_t b = Serial1.read();
      lastByteTime = millis();
      if (b == 0xFF) {
        ffCount++;
        if (ffCount == 3) break;
      } else {
        result += (char)b;
        ffCount = 0;
      }
    }
  }
  return result;
}
