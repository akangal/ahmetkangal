/*   Hoverboard_Serial_Test
 *   Controls the speed, brake, and direction of a single hoverboard motor
 *   via commands sent through the serial port.
 *   Measures the speed of a hoverboard motor asynchronously
 *   using a custom ReadSpeed function.  Uses the SC speed pulse output of the
 *   RioRand 400W 6-60V PWM DC Brushless Electric Motor Speed Controller with Hall.
 *   Outputs the speed data to the serial port.
 *     
 *   created 2021
 *   Mad-EE  (Mad Electrical Engineer)
 *   www.mad-ee.com
 *   
 *   This example code is in the public domain.
 *   
 *   Platform:  Arduino UNO
 */

// Constants
const unsigned long SPEED_TIMEOUT = 500000;       // Time used to determine wheel is not spinning
const unsigned int UPDATE_TIME = 500;             // Time used to output serial data
const unsigned int BUFFER_SIZE = 16;              // Serial receive buffer size
const double BAUD_RATE = 115200;                  // Serial port baud rate
const double WHEEL_DIAMETER_IN = 6.5;            // Motor wheel diamater (inches)
const double WHEEL_CIRCUMFERENCE_IN = 22.25;     // Motor wheel circumference (inches)
const double WHEEL_DIAMETER_CM = 16.5;           // Motor wheel diamater (centimeters)
const double WHEEL_CIRCUMFERENCE_CM = 56.5;      // Motor wheel circumference (centimeters)

// Pin Declarations
const int PIN_DIR = 2;      // Motor direction signal
const int PIN_DIR2 = 6;      // Motor direction signal
const int PIN_BRAKE = 4;    // Motor brake signal (active low)
const int PIN_BRAKE2 = 7;    // Motor brake signal (active low)
const int PIN_PWM = 9;      // PWM motor speed control
const int PIN_PWM2 = 10;      // PWM motor speed control
const int PIN_SPEED = 12;   // SC Speed Pulse Output from RioRand board
const int PIN_SPEED2 = 3;   // SC Speed Pulse Output from RioRand board

// Variables used in ReadFromSerial function
String _command = "";       // Command received in Serial read command
int _data = 0;              // Data received in Serial read command

// Variables used in ReadSpeed function
double _freq;               // Frequency of the signal on the speed pin
double _rpm;                // Wheel speed in revolutions per minute
double _mph;                // Wheel speed in miles per hour
double _kph;                // Wheel speed in kilometers per hour

//2nd motor
double _freq2;  // Frequency of the signal on the speed pin (Motor 2)
double _rpm2;   // Wheel speed in revolutions per minute (Motor 2)
double _mph2;   // Wheel speed in miles per hour (Motor 2)
double _kph2;   // Wheel speed in kilometers per hour (Motor 2)


// This is ran only once at startup
void setup() 
{
    // Set pin directions
    pinMode(PIN_SPEED, INPUT);
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);

    //2nd motor
    pinMode(PIN_SPEED2, INPUT);
    pinMode(PIN_PWM2, OUTPUT);
    pinMode(PIN_BRAKE2, OUTPUT);
    pinMode(PIN_DIR2, OUTPUT);
    
    // Set initial pin states
    digitalWrite(PIN_BRAKE, false);
    digitalWrite(PIN_DIR, false);
    analogWrite(PIN_PWM, 20);

    //2nd motor
    // Set initial pin states
    digitalWrite(PIN_BRAKE2, false);
    digitalWrite(PIN_DIR2, false);
    analogWrite(PIN_PWM2, 20);
    
    // Initialize serial port
    Serial.begin(BAUD_RATE);
    Serial.println("---- Program Started ----");
}

// This is the main program loop that runs repeatedly
void loop() 
{
    // Read serial data and set dataReceived to true if command is ready to be processed
    bool dataReceived = ReadFromSerial();

    // Process the received command if available
    if (dataReceived == true)
        ProcessCommand(_command, _data);

    // Read the speed from input pin (sets _rpm and _mph)
    ReadSpeed();

    // Outputs the speed data to the serial port 
    WriteToSerial(); 
}

// Receives string data from the serial port
// Data should be in the format <command>,<data>
// Data should be terminated with a carriage return
// Function returns true if termination character received 
bool ReadFromSerial()
{    
    // Local variables   
    static String cmdBuffer;        // Stores the received command
    static String dataBuffer;       // Stores the received data
    static bool isCommand = true;   // Flag to store received bytes in command or data buffer
    byte recByte;                   // Byte received from the serial port
    
    // Check if any new data is available, if not exit
    if (Serial.available() == false)
      return false;
    
    // Read single byte from serial port
    recByte = Serial.read();
    
    // Check if byte is termination character (carriage return)
    // Check if byte is termination character (carriage return or newline)
    if (recByte == '\r' || recByte == '\n')

    {
        // Save buffers to global variables
        cmdBuffer.toUpperCase();
        _command = cmdBuffer;
        _data = dataBuffer.toInt();
      
        // Write what was received back to the serial port
        Serial.print("Received: "); 
        Serial.print(_command); 
        Serial.print(",");
        Serial.println(_data);
      
        // Clear local variables
        cmdBuffer = "";
        dataBuffer = "";
        isCommand = true;
      
        return true;
    }
    
    // Check if byte is a comma which separates the command from the data
    if ((char)recByte == ',')
    {
        isCommand = false;  // Next byte will be a data byte
        return false;
    }

    // Save data to one of the receive buffers
    if (isCommand)
        cmdBuffer += (char)recByte;
    else
        dataBuffer += (char)recByte;
    
    return false;
}

// Processes the command and data, sends result to serial port
void ProcessCommand(String command, int data)
{  
    // Process SPEED command
    if (command == "PWM")
    {
      Serial.print("Setting speed:  ");
      Serial.println(data);
      analogWrite(PIN_PWM, data);
    }
        
    // Process BRAKE command
    if (command == "BRAKE")
    {
      Serial.print("Setting brake:  ");
      Serial.println(data);
      digitalWrite(PIN_BRAKE, data);
    }

    // Process DIR command
    if (command == "DIR")
    {
      Serial.print("Setting direction:  ");
      Serial.println(data);
      digitalWrite(PIN_DIR, data);
    }
 //2nd motor

    // Process SPEED command
    if (command == "PWM2")
    {
      Serial.print("Setting speed:  ");
      Serial.println(data);
      analogWrite(PIN_PWM2, data);
    }
        
    // Process BRAKE command
    if (command == "BRAKE2")
    {
      Serial.print("Setting brake:  ");
      Serial.println(data);
      digitalWrite(PIN_BRAKE2, data);
    }

    // Process DIR command
    if (command == "DIR2")
    {
      Serial.print("Setting direction:  ");
      Serial.println(data);
      digitalWrite(PIN_DIR2, data);
    }
}

// Reads the speed from the input pin and calculates RPM and MPH
// Monitors the state of the input pin and measures the time (µs) between pin transitions
// Reads the speed from the input pins and calculates RPM and MPH for both motors
void ReadSpeed()
{
    static bool lastState1 = false, lastState2 = false;
    static unsigned long last_uS1, last_uS2;
    static unsigned long timeout_uS1, timeout_uS2;

    // Motor 1
    bool state1 = digitalRead(PIN_SPEED);
    if (state1 != lastState1)
    {
        unsigned long current_uS = micros();
        unsigned long elapsed_uS = current_uS - last_uS1;
        double period_uS = elapsed_uS * 2.0;
        _freq = (1 / period_uS) * 1E6;
        _rpm = _freq / 45 * 60;
        if (_rpm > 5000) _rpm = 0;
        _mph = (WHEEL_CIRCUMFERENCE_IN * _rpm * 60) / 63360;
        _kph = (WHEEL_CIRCUMFERENCE_CM * _rpm * 60) / 100000;
        last_uS1 = current_uS;
        timeout_uS1 = last_uS1 + SPEED_TIMEOUT;
        lastState1 = state1;
    }
    else if (micros() > timeout_uS1)
    {
        _freq = _rpm = _mph = _kph = 0;
        last_uS1 = micros();
    }

    // Motor 2
    bool state2 = digitalRead(PIN_SPEED2);
    if (state2 != lastState2)
    {
        unsigned long current_uS = micros();
        unsigned long elapsed_uS = current_uS - last_uS2;
        double period_uS = elapsed_uS * 2.0;
        _freq2 = (1 / period_uS) * 1E6;
        _rpm2 = _freq2 / 45 * 60;
        if (_rpm2 > 5000) _rpm2 = 0;
        _mph2 = (WHEEL_CIRCUMFERENCE_IN * _rpm2 * 60) / 63360;
        _kph2 = (WHEEL_CIRCUMFERENCE_CM * _rpm2 * 60) / 100000;
        last_uS2 = current_uS;
        timeout_uS2 = last_uS2 + SPEED_TIMEOUT;
        lastState2 = state2;
    }
    else if (micros() > timeout_uS2)
    {
        _freq2 = _rpm2 = _mph2 = _kph2 = 0;
        last_uS2 = micros();
    }
}


// Writes the RPM and MPH for both motors to the serial port
void WriteToSerial()
{
    static unsigned long updateTime;
    if (millis() > updateTime)
    {
        Serial.print((String)"Motor 1 - Freq:" + _freq + " RPM:" + _rpm + " MPH:" + _mph + " KPH:" + _kph + " ");
        Serial.println();
        Serial.print((String)"Motor 2 - Freq:" + _freq2 + " RPM:" + _rpm2 + " MPH:" + _mph2 + " KPH:" + _kph2 + " ");
        Serial.println();
        updateTime = millis() + UPDATE_TIME;
    }
}
