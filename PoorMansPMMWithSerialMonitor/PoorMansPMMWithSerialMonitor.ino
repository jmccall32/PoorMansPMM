# include <EEPROM.h>

// Initiate measured values to a reasonable value
float Vstart = 12.7;
float Vhouse = 12.7;

// Control setpoints
float Vcharge = 13.1;
float Vstop = 12.8;
float Voff = 11.5;
unsigned long delayTime_ms = 10000;

// EEProm addresses for control setpoints
const unsigned int EEPromIsSetAddress = 0; // This is the location of a byte (8-bit int) set to 1 if true, any other value if false (not using a bool because "clear" EEProm values can be either 0 or 255)
const unsigned int VchargeAddress = 4;
const unsigned int VstopAddress = 8;
const unsigned int VoffAddress = 12;
const unsigned int delayTimeAddress = 16;

// Setpoint limits
const float Vmin = 10.0;
const float Vmax = 15.0;
const unsigned int delayTime_min = 1000;
const unsigned int delayTime_max = 3600000;

// Timeout when changing setpoints through serial terminal
const int dataEntryTimeout_ms = 10000;

// Voltage input settings
const float K_voltageScale = (100.0 / 18.0) + 1.0; // = R1/R2 + 1
const int startVoltagePin = A1;
const int houseVoltagePin = A0;

// Digital IO settings
const int boardPowerPin = 12; 
const int crossChargingPin = 13; // has on-board indicator LED which is convenient
const int waitPin = 8; // indicator LED for wait state
const int onPin = 9; // indicator LED for on state
const int D121 = 6; // 12V digital input (with 12V reed relay) #1
const int D122 = 7; // 12V digital input (with 12V reed relay) #2

// State machine variables
const unsigned int STATE_OFF = 0;
const unsigned int STATE_WAIT = 1;
const unsigned int STATE_ON = 2;
const char* stateNames[] = {"off","waiting to charge","cross-charging"};
int state = STATE_OFF;
int nextState = STATE_OFF;
unsigned long stateChangeTime = 0;
unsigned long timeInState = 0;
 
void setup() 
{
  // Get the two digital outs into the correct initial state before you do anything else
  pinMode(boardPowerPin,OUTPUT);
  digitalWrite(boardPowerPin,HIGH);
  pinMode(crossChargingPin,OUTPUT);
  digitalWrite(crossChargingPin,LOW);
  
  pinMode(waitPin,OUTPUT);
  pinMode(onPin,OUTPUT);
  
  // turn on both indicator LEDs as "lamp test" and to indicate bootup state
  digitalWrite(waitPin,HIGH);
  digitalWrite(onPin,HIGH);
  
  pinMode(D121,INPUT);
  pinMode(D122,INPUT);

  if(EEPROM.read(EEPromIsSetAddress) == 1)
  {
	Vcharge = loadFloatSetpoint(VchargeAddress,Vcharge,Vmin,Vmax);
	Vstop = loadFloatSetpoint(VstopAddress,Vstop,Vmin,Vmax);
	Vstop = loadFloatSetpoint(VstopAddress,Vstop,Vmin,Vmax);
	delayTime_ms = loadUnsignedSetpoint(delayTimeAddress,delayTime_ms,delayTime_min,delayTime_max);
  }
  else
  {
    EEPROM.update(EEPromIsSetAddress,1);
    EEPROM.put(VchargeAddress,Vcharge);
    EEPROM.put(VstopAddress,Vstop);
    EEPROM.put(VoffAddress,Voff);
    EEPROM.put(delayTimeAddress,delayTime_ms);
  }

  Serial.begin(115200);
  
  // Give the board 10 seconds for filter caps to come to voltage
  delay(10000);
  
  stateChangeTime = millis();
}
 
void loop() 
{
  if(state != nextState)
  {
    stateChangeTime = millis();
  }
  timeInState = millis() - stateChangeTime;
  state = nextState;
  Vstart = readBatteryVoltage(startVoltagePin);
  Vhouse = readBatteryVoltage(houseVoltagePin);

  if(Vhouse < Voff)
  {
    Serial.println("House battery voltage critically low");
    delay(5000); // Wait to make sure it's not a short transient
    Vhouse = readBatteryVoltage(houseVoltagePin);
    if(Vhouse < Voff)
    {
      digitalWrite(boardPowerPin,LOW);
      delay(1000); // Wait for debouncing purposes
    }
  }

  switch(state)
  {
    case STATE_OFF:
      digitalWrite(crossChargingPin,LOW);
      digitalWrite(waitPin,LOW);
      digitalWrite(onPin,LOW);
      if(Vhouse > Vcharge || Vstart > Vcharge)
      {
        nextState = STATE_WAIT;
      }
      else
      {
        nextState = STATE_OFF;
      }
      break;
    case STATE_WAIT:
      digitalWrite(crossChargingPin,LOW);
      digitalWrite(waitPin,HIGH);
      digitalWrite(onPin,LOW);
      if(Vhouse > Vcharge || Vstart > Vcharge)
      {
        if (timeInState > delayTime_ms)
        {
          nextState = STATE_ON;
        }
        else
        {
          nextState = STATE_WAIT;
        }
      }
      else
      {
        nextState = STATE_OFF;
      }
      break;
    case STATE_ON:
      digitalWrite(crossChargingPin,HIGH);
      digitalWrite(waitPin,LOW);
      digitalWrite(onPin,HIGH);
      if(Vhouse < Vstop && Vstart < Vstop)
      {
        nextState = STATE_OFF;
      }
      else
      {
        nextState = STATE_ON;
      }
      break;
  }
  
  clearScreen();
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
	Serial.println("Enter 4 to change charging delay time");
	Serial.println("Enter 5 to restore default settings");
	Serial.println("Enter 0 to cancel");

    int menuOption = readSerialNumberInput(0,0,5,false);

    switch(menuOption)
    {
      case 0:
        Serial.println("Cancelled");
        break;
      case 1:
        Serial.println("Enter new charge start voltage");
        Vcharge = readSerialNumberInput(Vcharge,Vmin,Vmax,true);
        EEPROM.put(VchargeAddress,Vcharge);
        break;
      case 2:
        Serial.println("Enter new charge stop voltage");
        Vstop = readSerialNumberInput(Vstop,Vmin,Vmax,true);
        EEPROM.put(VstopAddress,Vstop);
        break;
      case 3:
        Serial.println("Enter new low battery shutdown voltage");
        Voff = readSerialNumberInput(Voff,Vmin,Vmax,true);
        EEPROM.put(VoffAddress,Voff);
        break;
      case 4:
        Serial.println("Enter new charging delay time");
        delayTime_ms = readSerialNumberInput(delayTime_ms,delayTime_min,delayTime_max,true);
        EEPROM.put(delayTimeAddress,delayTime_ms);
        break;
      case 5:
        Serial.println("Enter 123456 to restore default settings");
        if (readSerialNumberInput(0,123456,123456,false) == 123456)
        {
          EEPROM.update(EEPromIsSetAddress,0);
          Serial.println("Default settings will be restored on next board reset");
          delay(2000);
        }
        break;
      default:
        Serial.println("Invalid entry");
        break;
    }
  }
  
  delay(1000);
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

float loadFloatSetpoint(unsigned int address,float oldValue, float minValue, float maxValue)
{
  float newValue;
  
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

unsigned int loadUnsignedSetpoint(unsigned int address,unsigned int oldValue, unsigned int minValue, unsigned int maxValue)
{
  unsigned int newValue;
  
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
