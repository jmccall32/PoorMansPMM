# include <EEPROM.h>

// Initiate measured values to a reasonable value
float Vstart = 12.7;
float Vhouse = 12.7;
float Vdiff = 0.0;

// Control setpoints
float Vcharge = 13.3; // Start cross-charging if start battery rises above this voltage
float Vstop = 13.0; // Stop cross-charging from start battery if it falls below this voltage
float Vdiff_start = 0.30; // Start cross-charging if (house - start) voltage exceeds this level
float Vdiff_stop = -0.04; // Start cross-charging if (house - start) voltage falls below this level
float Voff = 11.5; // Shut down controller if house battery falls below this voltage
unsigned long delayTime_ms = 15000; // Wait this long after voltage rises before starting cross-charging

// EEProm addresses for control setpoints
const unsigned int EEPromIsSetAddress = 0; // This is the location of an integer set to 1 if true, any other value if false (not using a bool because "clear" EEProm values can be either 0 or 255)
const unsigned int VchargeAddress = 4;
const unsigned int VstopAddress = 8;
const unsigned int VoffAddress = 12;
const unsigned int delayTimeAddress = 16;
const unsigned int Vdiff_startAddress = 20;
const unsigned int Vdiff_stopAddress = 24;
const unsigned int alphaAddress = 28;

// Setpoint limits
const float V_min = 10.0;
const float V_max = 15.0;
const float Vdiff_min = -3.0;
const float Vdiff_max = 3.0;
const float alpha_min = 0.0;
const float alpha_max = 1.0;
const unsigned long delayTime_min = 1000;
const unsigned long delayTime_max = 360000;

// Timeout when changing setpoints through serial terminal
const long dataEntryTimeout_ms = 10000;

// Voltage input settings
const float K_voltageScale = (100.0 / 18.0) + 1.0; // = R1/R2 + 1
const int startVoltagePin = A1;
const int houseVoltagePin = A0;
float alpha = 0.05; // alpha of discrete low-pass filter; for 1-second cycle time, this is *about* the same as the filter cutoff frequency

// Digital IO settings
const int boardPowerRelay = 13; // relay to keep board power on
const int crossChargingRelay = 12; // pilot relay for off-board cross charging solenoid
const int offIndicator = 4; // indicator LED for off state
const int onIndicator = 5; // indicator LED for on state
const int D121 = 6; // 12V digital input (with 12V reed relay) #1
const int D122 = 7; // 12V digital input (with 12V reed relay) #2

// State machine constants
const unsigned int STATE_BOOT = 0;
const unsigned int STATE_OFF = 1;
const unsigned int STATE_WAIT_CONNECT = 2;
const unsigned int STATE_ON = 3;
const unsigned int STATE_WAIT_DISCONNECT = 4;
// Pad all state names to equal length with leading spaces
const char* stateNames[] = 
{
  "           booting up",
  "                  off",
  "   waiting to connect",
  "       cross-charging",
  "waiting to disconnect"
  };

// set this true on build to cross-charge if start battery at lower voltage than house battery, even if house battery isn't being charged
const bool crossChargeToStart = true;

// State machine variables
int state = STATE_BOOT;
int nextState = STATE_BOOT;
unsigned long stateChangeTime = 0;
unsigned long timeInState = 0;
unsigned long cycleEndTime = 0;
bool flasherState = true; // used for flashing LEDs
bool readyToConnect = false; // criteria met to start cross-charging
bool readyToDisconnect = false; // criteria met to stop cross-charging

// Mode in which serial monitor puts out tab delimited columnar data instead of human-friendly screens
bool dataLoggingMode = false;
const long serialBaudRate = 9600;
const int cycleTime_ms = 1000;
 
void setup() 
{
  // Get the two main relays into the correct initial state before you do anything else
  pinMode(boardPowerRelay,OUTPUT);
  digitalWrite(boardPowerRelay,HIGH);
  pinMode(crossChargingRelay,OUTPUT);
  digitalWrite(crossChargingRelay,LOW);
  
  pinMode(offIndicator,OUTPUT);
  pinMode(onIndicator,OUTPUT);
  pinMode(D121,INPUT);
  pinMode(D122,INPUT);

  if(EEPROM.read(EEPromIsSetAddress) == 1)
  {
    Vcharge = loadSetpoint(VchargeAddress,Vcharge,V_min,V_max);
    Vstop = loadSetpoint(VstopAddress,Vstop,V_min,V_max);
    Voff = loadSetpoint(VoffAddress,Voff,V_min,V_max);
    Vdiff_start = loadSetpoint(Vdiff_startAddress,Vdiff_start,Vdiff_min,Vdiff_max);
    Vdiff_stop = loadSetpoint(Vdiff_stopAddress,Vdiff_stop,Vdiff_min,Vdiff_max);
    alpha = loadSetpoint(alphaAddress,alpha,alpha_min,alpha_max);
    delayTime_ms = loadSetpoint(delayTimeAddress,delayTime_ms,delayTime_min,delayTime_max);
  }
  else
  {
    EEPROM.put(EEPromIsSetAddress,1);
    EEPROM.put(VchargeAddress,Vcharge);
    EEPROM.put(VstopAddress,Vstop);
    EEPROM.put(VoffAddress,Voff);
    EEPROM.put(Vdiff_startAddress,Vdiff_start);
    EEPROM.put(Vdiff_stopAddress,Vdiff_stop);
    EEPROM.put(alphaAddress,alpha);
    EEPROM.put(delayTimeAddress,delayTime_ms);
  }

  Serial.begin(serialBaudRate);
  
  stateChangeTime = millis();
  cycleEndTime = millis();
}

void loop() 
{
  cycleEndTime += cycleTime_ms;
  if(state != nextState)
  {
    stateChangeTime = millis();
  }
  timeInState = millis() - stateChangeTime;
  state = nextState;
  Vstart = (alpha * readBatteryVoltage(startVoltagePin)) + ((1-alpha) * Vstart);
  Vhouse = (alpha * readBatteryVoltage(houseVoltagePin)) + ((1-alpha) * Vhouse);
  Vdiff = Vhouse - Vstart;

  if(Vhouse < Voff && state != STATE_BOOT)
  {
    Serial.println("House battery voltage critically low");
    delay(5000); // Wait to make sure it's not a short transient
    Vhouse = readBatteryVoltage(houseVoltagePin);
    if(Vhouse < Voff)
    {
      digitalWrite(boardPowerRelay,LOW);
      delay(1000); // Wait for debouncing purposes
    }
  }
  
  readyToConnect = Vstart > Vcharge || Vhouse > Vcharge;
  readyToDisconnect = Vstart < Vstop && Vhouse < Vstop;
  
  if(crossChargeToStart)
  {
    readyToConnect = readyToConnect || Vdiff > Vdiff_start;
    readyToDisconnect = readyToDisconnect && Vdiff < Vdiff_stop;
  }

  switch(state)
  {
    case STATE_BOOT:
      digitalWrite(crossChargingRelay,LOW);
      // alternately flash indicator LEDs as "lamp test" and to indicate bootup state
      digitalWrite(offIndicator,flasherState);
      digitalWrite(onIndicator,!flasherState);
      
      // Wait in this state to allow low pass filters to clear before acting on reported voltages
      if (timeInState > delayTime_ms)
      {
        nextState = STATE_OFF;
      }
      else
      {
        nextState = STATE_BOOT;
      }
      break;
    case STATE_OFF:
      digitalWrite(crossChargingRelay,LOW);
      digitalWrite(offIndicator,HIGH);
      digitalWrite(onIndicator,LOW);
      if(readyToConnect)
      {
        nextState = STATE_WAIT_CONNECT;
      }
      else
      {
        nextState = STATE_OFF;
      }
      break;
    case STATE_WAIT_CONNECT:
      digitalWrite(crossChargingRelay,LOW);
      digitalWrite(offIndicator,HIGH);
      digitalWrite(onIndicator,flasherState);
      if(readyToConnect)
      {
        if (timeInState >= delayTime_ms)
        {
          nextState = STATE_ON;
        }
        else
        {
          nextState = STATE_WAIT_CONNECT;
        }
      }
      else
      {
        nextState = STATE_OFF;
      }
      break;
    case STATE_ON:
      digitalWrite(crossChargingRelay,HIGH);
      digitalWrite(offIndicator,LOW);
      digitalWrite(onIndicator,HIGH);
      if(readyToDisconnect)
      {
        nextState = STATE_WAIT_DISCONNECT;
      }
      else
      {
        nextState = STATE_ON;
      }
      break;
    case STATE_WAIT_DISCONNECT:
      digitalWrite(crossChargingRelay,HIGH);
      digitalWrite(offIndicator,flasherState);
      digitalWrite(onIndicator,HIGH);
      if(readyToDisconnect)
      {
        if (timeInState >= delayTime_ms)
        {
          nextState = STATE_OFF;
        }
        else
        {
          nextState = STATE_WAIT_DISCONNECT;
        }
      }
      else
      {
        nextState = STATE_ON;
      }
      break;
  }
  
  if(dataLoggingMode)
  {
    if(Serial.available())
    {
      dataLoggingMode = false;
      clearSerialBuffer();
      clearScreen();
    }
    else
    {
      printDataLogRow();
    }
  }
  
  if(!dataLoggingMode)
  {
    printStatus();
  }
  
  flasherState = !flasherState;
  
  while(millis() < cycleEndTime)
  {
  }
}

void clearScreen()
{
  // Clears terminal screen if terminal responds to VT100-style commands
  // Works well with "real" terminal program (e.g. PuTTY or Tera Term) but not the Arduino IDE serial monitor
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}

void sendCursorHome()
{
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}


void printDataLogRow()
{
  Serial.print(cycleEndTime);
  Serial.print("	");
  Serial.print(Vstart);
  Serial.print("	");
  Serial.print(Vhouse);
  Serial.print("	");
  Serial.print(state);
  Serial.print("	");
  Serial.println(nextState);
}

void printStatus()
{
  sendCursorHome();
  Serial.print("Starting battery voltage      = ");
  Serial.println(Vstart);
  Serial.print("House battery voltage         = ");
  Serial.println(Vhouse);
  Serial.print("System state  = ");
  Serial.print(state);
  Serial.print(": ");
  Serial.println(stateNames[state]);
  Serial.print("Next state    = ");
  Serial.print(nextState);
  Serial.print(": ");
  Serial.println(stateNames[nextState]);
  Serial.print("Time in state (s)             = ");
  Serial.println(timeInState/1000);
  Serial.println();
  Serial.print("Charge start voltage          = ");
  Serial.println(Vcharge);
  Serial.print("Charge stop voltage           = ");
  Serial.println(Vstop);
  Serial.print("Low battery shutdown voltage  = ");
  Serial.println(Voff);
  Serial.print("Charge start diff voltage     = ");
  Serial.println(Vdiff_start);
  Serial.print("Charge stop diff voltage      = ");
  Serial.println(Vdiff_stop);
  Serial.print("Voltage filter alpha          = ");
  Serial.println(alpha);
  Serial.print("Charging delay time (ms)      = ");
  Serial.println(delayTime_ms);
  
  Serial.println();
  
  Serial.println("Press any key to change settings");

  if(Serial.available())
  {
    clearSerialBuffer();
    Serial.println();
    Serial.println("Enter 1 to change charge start voltage");
    Serial.println("Enter 2 to change charge stop voltage");
    Serial.println("Enter 3 to change low battery shutdown voltage");
    Serial.println("Enter 4 to change differential start voltage");
    Serial.println("Enter 5 to change differential stop voltage");
    Serial.println("Enter 6 to change voltage filter alpha");
    Serial.println("Enter 7 to change charging delay time");
    Serial.println("Enter 8 to restore default settings");
    Serial.println("Enter 9 to enter data logging mode");
    Serial.println("Enter 0 to cancel");

    int menuOption = readSerialNumberInput(0,0,9,false);

    switch(menuOption)
    {
      case 0:
        Serial.println("Cancelled");
        clearScreen();
        break;
      case 1:
        Serial.println("Enter new charge start voltage");
        Vcharge = readSerialNumberInput(Vcharge,V_min,V_max,true);
        EEPROM.put(VchargeAddress,Vcharge);
        clearScreen();
        break;
      case 2:
        Serial.println("Enter new charge stop voltage");
        Vstop = readSerialNumberInput(Vstop,V_min,V_max,true);
        EEPROM.put(VstopAddress,Vstop);
        break;
      case 3:
        Serial.println("Enter new low battery shutdown voltage");
        Voff = readSerialNumberInput(Voff,V_min,V_max,true);
        EEPROM.put(VoffAddress,Voff);
        clearScreen();
        break;
      case 4:
        Serial.println("Enter new differential start voltage");
        Vdiff_start = readSerialNumberInput(Vdiff_start,Vdiff_min,Vdiff_max,true);
        EEPROM.put(Vdiff_startAddress,Vdiff_start);
        clearScreen();
        break;
      case 5:
        Serial.println("Enter new differential stop voltage");
        Vdiff_stop = readSerialNumberInput(Vdiff_stop,Vdiff_min,Vdiff_max,true);
        EEPROM.put(Vdiff_stopAddress,Vdiff_stop);
        clearScreen();
        break;
      case 6:
        Serial.println("Enter new filter alpha");
        alpha = readSerialNumberInput(alpha,alpha_min,alpha_max,true);
        EEPROM.put(alphaAddress,alpha);
        clearScreen();
        break;
      case 7:
        Serial.println("Enter new charging delay time");
        delayTime_ms = readSerialNumberInput(delayTime_ms,delayTime_min,delayTime_max,true);
        EEPROM.put(delayTimeAddress,delayTime_ms);
        clearScreen();
        break;
      case 8:
        Serial.println("Enter 123456 to restore default settings");
        if (readSerialNumberInput(0,123456,123456,false) == 123456)
        {
          EEPROM.update(EEPromIsSetAddress,0);
          Serial.println("Default settings will be restored on next board reset");
          delay(2000);
        }
        clearScreen();
        break;
      case 9:
        dataLoggingMode = true;

        clearScreen();
        Serial.println("Entering data logging mode");
        Serial.println("Press any key to exit");
        Serial.println("time_ms	Vstart	Vhouse	state	nextState");
        break;
      default:
        Serial.println("Invalid entry");
        break;
    }
  }
}

float readSerialNumberInput(float oldValue, float minValue, float maxValue, bool printResult)
{
    static const int maxEntryLength = 32;
    int idx = 0;
    static const char enterKey = '\r';
    char thisChar;
    char thisString[maxEntryLength];
    bool entryComplete = false;
    float outputValue;
    unsigned long lastEntryTime = millis();
    String outputString;

    while(!entryComplete)
    {
      if((millis() - lastEntryTime) > dataEntryTimeout_ms || idx >= maxEntryLength)
      {
        entryComplete = true;
        clearSerialBuffer();
        return oldValue;
      }
      else if(Serial.available())
      {
        lastEntryTime = millis();
        thisChar = Serial.read();

        if(thisChar == enterKey)
        {
          entryComplete = true;
          clearSerialBuffer();
          thisString[idx] = '\0'; // terminate the string

          outputString = String(thisString);
          outputValue = outputString.toFloat();

          if(outputValue < minValue)
          {
            Serial.println();
            Serial.print("Value cannot be less than ");
            Serial.println(minValue);
            delay(2000);
            return oldValue;
          }
          else if(outputValue > maxValue)
          {
            Serial.println();
            Serial.print("Value cannot be greater than ");
            Serial.println(maxValue);
            delay(2000);
            return oldValue;
          }
          else
          {
            Serial.println();
            if(printResult)
            {
              Serial.print("Value updated to ");
              Serial.println(outputValue);
              delay(2000);
            }
            return outputValue;
          }
        }
        else
        {
          Serial.print(thisChar); // echo input
          thisString[idx] = thisChar;
          idx++;
        }
      }
    }
      
    
}

void clearSerialBuffer()
{
  while(Serial.available())
  {
    Serial.read();
  }
}

float readBatteryVoltage(int pin)
{
  static const float Vref = 5.0;
  
  // I'm making this a float to ensure that (rawCounts / maxCounts) doesn't do integer division.  Not sure if this is necessary.  I should go learn more about how C++ and Arduino handle mixed types...
  static const float maxCounts = pow(2.0,10.0) - 1.0;
  
  float batteryVoltage = K_voltageScale * Vref * (analogRead(pin) / maxCounts);
  return batteryVoltage;
}

float loadSetpoint(unsigned int address,float oldValue, float minValue, float maxValue)
{
  float newValue;
  
  EEPROM.get(address,newValue);

  if(newValue > maxValue || newValue < minValue || isnan(newValue))
  {
	  Serial.print("Out-of-range value at EEPROM address ");
	  Serial.print(address);
	  Serial.println(". Default value used instead.");
	  EEPROM.put(address,oldValue);
    return oldValue;
	
  }
  else
  {
    return newValue;
  }
}

unsigned long loadSetpoint(unsigned int address,unsigned long oldValue, unsigned long minValue, unsigned long maxValue)
{
  unsigned long newValue;
  
  EEPROM.get(address,newValue);

  if(newValue > maxValue || newValue < minValue)
  {
    Serial.print("Out-of-range value at EEPROM address ");
    Serial.print(address);
    Serial.println(". Default value used instead.");
    EEPROM.put(address,oldValue);
    return oldValue;
  }
  else
  {
    return newValue;
  }
}
