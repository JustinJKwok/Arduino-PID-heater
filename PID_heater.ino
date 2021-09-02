//Make grounds common for thermistor circuit and heater circuit 
// ...BUT position them away from each other otherwise the pwm heater current interferes with ADC readings.
// ... ALSO use pwm pin furthest away from other pins in use, esp. ground
// ... after the above got rid of oscillations but still get about 1 spike per sampling period
// ... solved the spike issue by putting a 100 uf capacitor between the analogRead thermistor pin and GND

//PID temperature control for 12V cartridge heater and 100k thermistor. Displays to LCD screen.
#include <EEPROM.h>
//#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
#include <Ticker.h>
#include <DFRobot_RGBLCD.h>
#include <Wire.h>

#define R_SERIES 10000.0
#define THERMISTOR_PIN A3 //analog in
#define MOSFET_PIN 10 //pwm
#define NUM_COEFF 5
#define WINDOW_SIZE 30

// polynomial coefficients from fitting 4th order polynomial to 1/T vs ln(R) data
const double thermistorCoeffs[NUM_COEFF] = {0.0007991, 0.00021205, 0.00000029006, 0.000000040926, 0.00000000072227};

double windowTemps[WINDOW_SIZE];
double windowAvgTemp;
double windowSum;
double stableAvgTempErr = 0.6; // the +/- error that is acceptable for the movingAvgTemp (w.r.t. setTemp) to be considered stable.
double stableAbsTempErr = 1.2; // the +/- error that is acceptable for all Temps in the window
double unsafeTemp = 40.0;
double minTemp = 20.0;
double maxTemp = 150.0;
double setTemp = 25.0;
double latestTemp = setTemp; //either get an instant read at start or disable pid until we have an avg reading
double thermistorADC = 0.0;
int sampleNum = 0; //for ADC readings, rename?
double computePIDInterval = 1000; //milliseconds
const int samplingInterval = 5; //milliseconds
const int numSamplesToAvg = 200;
double updateLCDInterval = 1000;
bool hasReportedStable = true;

// timed functions
void readThermistor();
void updateLCDTemp();
Ticker timerReadThermistor(readThermistor, samplingInterval);
Ticker timerUpdateLCDTemp(updateLCDTemp, updateLCDInterval);

// pid gain vars
double pwmOut = 0.0;
double KpDefault = 1.5;
double KiDefault = 0.015;
double KdDefault = 10;
double Kp;
double Ki;
double Kd;
int KpEEAddr = 0;
int KiEEAddr = 4;
int KdEEAddr = 8;

PID heaterPID(&latestTemp, &pwmOut, &setTemp, Kp, Ki, Kd, DIRECT);

DFRobot_RGBLCD LCD(16,2);
int r = 30;//30;
int g = 128;//90;
int b = 0;//10;
int colorStable[] = {40, 128, 0};
int colorUnstable[] = {128, 63, 0};
int colorHot[] = {128, 0, 0};
int colorCool[] = {13, 63, 128};
byte customOne[8] = {0b00001, 0b01001, 0b11001, 0b01001, 0b01001, 0b11101, 0b00001, 0b11111};
byte customTwo[8] = {0b00001, 0b11001, 0b00101, 0b01001, 0b10001, 0b11101, 0b00001, 0b11111};

// see Serial Input Basics by Robin2: https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
const int maxChars = 32; //max number of chars when reading from serial into cstring. anything larger rewrites last char of cstring (?)
const int maxTokens = 10; //max number of "words" or "tokens" separated by spaces (e.g. "mv samx 1" is 3 words or 3 tokens). print error if exceeded
char incomingChars[maxChars]; //incoming chars until newline terminator
char incomingParams[maxTokens][maxChars]; //incoming chars parsed into cstrings
char tempChars[maxChars];
bool newSerialData = false; // whether there is new data parsed from serial that needs to be processed
int numTokens = 0; //number of tokens or params

char charACK[] = "ACK"; //"\x06";
char charNAK[] = "NAK";
char charSUCC[] = "SUCC";
char charFAIL[] = "FAIL";
char charWARN[] = "WARN";

bool isDebugging = false;

void setup() {
  // put your setup code here, to run once:
    analogReference(EXTERNAL);
    timerReadThermistor.start();
    timerUpdateLCDTemp.start();

    LCD.init();
    LCD.setRGB(0, 0, 0);    
    LCD.customSymbol(1, customOne);
    LCD.customSymbol(2, customTwo);
    redrawLCD(); //clear, print fixed text, print set point
    //LCD.setRGB(colorCool[0],colorCool[1],colorCool[2]);
    
    latestTemp = convertADCTemp(analogRead(THERMISTOR_PIN));
    updateLCDTemp();

    //initialize moving avg readings
    for (int i = 0; i < WINDOW_SIZE; ++i)
    {
        windowTemps[i] = latestTemp;
    }
    windowAvgTemp = latestTemp;
    windowSum = windowAvgTemp * (double)WINDOW_SIZE;

    EEPROM.get(KpEEAddr, Kp);
    EEPROM.get(KiEEAddr, Ki);
    EEPROM.get(KdEEAddr, Kd);
    
    heaterPID.SetSampleTime(computePIDInterval);
    //heaterPID.SetMode(AUTOMATIC);
    heaterPID.SetMode(MANUAL);
    
    Serial.begin(115200);
}

void loop() {
    timerReadThermistor.update();
    timerUpdateLCDTemp.update();

    if (latestTemp > (setTemp + 10.0))
    {
        heaterPID.SetTunings(Kp,Ki,0.0);
    }
    else
    {
        heaterPID.SetTunings(Kp,Ki,Kd);
    }

    readNextSerial();
    if (newSerialData == true)
    {
        if (isDebugging)
        {
            Serial.print(">>");
            Serial.println(incomingChars);
        }

        strcpy(tempChars, incomingChars);
        parseTokens();
        goToCommand();
    }
    
    heaterPID.Compute();
    analogWrite(MOSFET_PIN, pwmOut);

    newSerialData = false;
    //reset the parsed tokens
    for (int i = 0; i < numTokens; ++i)
    {
        incomingParams[i][0] = '\0';
    }
    //reset the token counter
    numTokens = 0;
}

void readNextSerial()
{
    //read serial buffer until newline, read into incomingChars
    static bool hasReadHeader = false;
    static int ndx = 0;
    char nextChar;
    char footer = '\n';
    char header = '>';

    while (Serial.available() > 0 && newSerialData == false)
    {
        nextChar = Serial.read();

        if (hasReadHeader == true)
        {
            //keep adding into cstring until newline
            if (nextChar != footer)
            {
                incomingChars[ndx] = nextChar;
                ++ndx;
                if (ndx >= maxChars)
                {
                    ndx = maxChars - 1;
                }
            }
            else
            {
                //if reading newline then terminate cstring and indicate there is new data to process
                incomingChars[ndx] = '\0';
                ndx = 0;
                newSerialData = true;
                hasReadHeader = false;
            }
        }
        else if (nextChar == header)
        {
            hasReadHeader = true;
        }
    }
}

void parseTokens()
{
    char* token;
    numTokens = 0;

    token = strtok(tempChars, " ");

    while (token != NULL)
    {
        //save into cstring array and read next portion
        strcpy(incomingParams[numTokens], token);
        ++numTokens;

        if (numTokens >= maxTokens)
        {
            numTokens = maxTokens - 1;
        }

        token = strtok(NULL, " ");
    }
}

void readThermistor()
{   
    //int reading = analogRead(THERMISTOR_PIN);
    //Serial.print("CURRENTADC:"); Serial.print(reading); Serial.print(", ");
    //Serial.println();
    
    thermistorADC += analogRead(THERMISTOR_PIN);
    ++sampleNum;
    
    //if analog read thermistor adc is near max 1023, then thermistor is likely disconnected and you should turn off PID and set pwmout 0
    //if temp ever exceed the max allowable temp then force pwmout to 0 (def for manual, but what about PID?) blink the blaclight (check if blink can be called each loop)
    //place this after computer but before analogwrite
    if (sampleNum == numSamplesToAvg)
    {
        thermistorADC = thermistorADC / numSamplesToAvg;
        latestTemp = convertADCTemp(thermistorADC);

        if (latestTemp > maxTemp)
        {
            if (isDebugging)
            {
                printWARN(); //move outside debug flag if ready to parse unprompted WARN
                Serial.println("Max temperature exceeded. Turning off PID and heating");
            }
            heaterPID.SetMode(MANUAL);
            pwmOut = 0.0;
            LCD.blinkLED();
        }
        else if (latestTemp < convertADCTemp(1015))
        {
            if (isDebugging)
            {
                printWARN(); //move outside debug flag if ready to parse unprompted WARN
                Serial.println("Problem with thermistor circuit. Turning off PID and heating.");
            }
            heaterPID.SetMode(MANUAL);
            pwmOut = 0.0;
            LCD.blinkLED();
            
        }
        else
        {
            LCD.noBlinkLED(); 
        }
        
        updateMovingAvg();
        updateLCDColor();
        updateLCDSetTemp();

        sampleNum = 0;
        
        if (isDebugging)
        {
            Serial.print("T:"); Serial.print(latestTemp); Serial.print(", ");
            Serial.print("S:"); Serial.print(setTemp); Serial.print(", ");
            Serial.print("PWM%:"); Serial.print(pwmOut/2.55); Serial.print(", ");
            Serial.print("WindowT:"); Serial.print(windowAvgTemp); Serial.print(", ");
            //Serial.print("ADC:"); Serial.print(thermistorADC); Serial.print(", ");
            Serial.println();
        }
        thermistorADC = 0.0;        
    }
}

double convertADCTemp(double val)
{
    double lnR = log(R_SERIES * val / (1023.0 - val));

    //Serial.print("Thermistor resistance: ");
    //Serial.println(R_SERIES * val / (1023.0 - val));
    
    double temp = 0.0;
    for (int i = 0; i < NUM_COEFF; ++i)
    {
         temp = temp + thermistorCoeffs[i] * pow(lnR, i);
    }

    temp = 1 / temp - 273.15;
    return temp;
}

void updateLCDTemp()
{
    char tempstr[6];
    dtostrf(latestTemp, 5, 1, tempstr);
    LCD.setCursor(8,0);
    LCD.print(tempstr);
}

void updateLCDSetTemp()
{
    char tempstr[6];
    dtostrf(setTemp, 5, 1, tempstr);
    LCD.setCursor(8,1);
    LCD.print(tempstr);
}

void redrawLCD()
{
    LCD.clear();
    
    LCD.setCursor(0,0);
    LCD.write(1);
    LCD.setCursor(2,0);
    LCD.print("Temp:");
    LCD.setCursor(2,1);
    LCD.print("Set :");
    LCD.setCursor(14,0);
    LCD.print((char)223);
    LCD.print('C');
    LCD.setCursor(14,1);
    LCD.print((char)223);
    LCD.print('C');

    updateLCDSetTemp();
}

void updateMovingAvg()
{
    static int ndx = 0;
    windowSum = windowSum - windowTemps[ndx];
    windowTemps[ndx] = latestTemp;
    windowSum = windowSum + latestTemp;

    ndx = ++ndx;
    ndx = ndx % WINDOW_SIZE;

    windowAvgTemp = windowSum / WINDOW_SIZE;   
}

void updateLCDColor()
{
    if (heaterPID.GetMode() == AUTOMATIC)
    {
        if ( (windowAvgTemp >= setTemp - stableAvgTempErr) && (windowAvgTemp <= setTemp + stableAvgTempErr) && allWindowTempsWithinErr() )
        {
            LCD.setRGB(colorStable[0], colorStable[1], colorStable[2]);

            //move this out to a more relevant location
            if (!hasReportedStable)
            {
                printSUCC();
                Serial.print("Temperature has stabilized at setpoint of ");
                Serial.print(setTemp);
                Serial.print(", actual ");
                Serial.println(latestTemp);
                hasReportedStable = true;
            }
        }
        else
        {            
            LCD.setRGB(colorUnstable[0], colorUnstable[1], colorUnstable[2]);

            //move this out to a more relevant location
            if (hasReportedStable && isDebugging)
            {
                printWARN();
                Serial.println("Temperature deviating from set point");
            }
        }
    }
    else
    {
        if (latestTemp >= unsafeTemp)
        {
            LCD.setRGB(colorHot[0], colorHot[1], colorHot[2]);
        }
        else
        {
            LCD.setRGB(colorCool[0], colorCool[1], colorCool[2]);
        }
    }
}

bool allWindowTempsWithinErr()
{
    for (int i = 0; i < WINDOW_SIZE; ++i)
    {
        if ( (windowTemps[i] <= setTemp - stableAbsTempErr) || (windowTemps[i] >= setTemp + stableAbsTempErr) )
        {
            return false;
        }   
    }
    return true;
}

bool isValidDoubleInRange(char* str, double& num, double lb, double ub)
{
    char* endptr;
    num = strtod(str, &endptr);
    if (endptr == str || *endptr != '\0')
    {
        printNAK();
        Serial.print("Parameter ");
        Serial.print(incomingParams[2]);
        Serial.println(" is not a number");

        return false;
    }
    else
    {
        //bounds are inclusive (num = lb or ub is ok)
        if (num < lb || num > ub)
        {
            printNAK();
            Serial.print("Parameter ");
            Serial.print(incomingParams[2]);
            Serial.println(" is out of range");

            return false;
        }
        else
        {
          return true;
        }
    }
}

void setPIDOn()
{
    if (numTokens == 1)
    {
        printlnACK();
        heaterPID.SetMode(AUTOMATIC);
        printSUCC();
        Serial.println("PID control turned ON");

    }
    else
    {
        printNAK();
        Serial.println("Invalid number of parameters");

    }
}

void setPIDOff()
{
    if (numTokens == 1)
    {
        printlnACK();
        heaterPID.SetMode(MANUAL);
        pwmOut = 0.0;

        printSUCC();
        Serial.println("PID control turned OFF. MOSFET PWM set to zero.");

    }
    else
    {
        printNAK();
        Serial.println("Invalid number of parameters");

    }
}

void goToSetCommand()
{
    if (numTokens != 3)
    {
        printNAK();
        Serial.println("Invalid number of parameters");
        return;
    }
    
    //Difference between Ts and T: 
    //Ts can be set if pid is on or off. T can only be set with pid is on
    //Ts will report SUCC immediately when setpoint is changed regardless of actual temp
    //T will report SUCC only when the actual T has stabilitized at set point
    if (strcmp(incomingParams[1], "Ts") == 0)
    {
        double temp;
        if (!isValidDoubleInRange(incomingParams[2], temp, minTemp, maxTemp))
        {
            return;
        }
        
        printlnACK();    
        setTemp = temp;
        //put other flags like notstable or hasreported here   
        
        printSUCC();
        Serial.print("Setting temperature setpoint to ");
        Serial.println(temp); 
    }
    else if (strcmp(incomingParams[1], "T") == 0)
    {
        double temp;
        if (!isValidDoubleInRange(incomingParams[2], temp, minTemp, maxTemp))
        {
            return;
        }
        printlnACK();  

        if (heaterPID.GetMode() == MANUAL)
        {
            printFAIL();
            Serial.println("Cannot set T when PID is off. Use Ts instead to force set the setpoint.");
            return;
        }
         
        setTemp = temp;
        //put other flags like notstable or hasreported here   
        //printlnACK();   
        //Serial.print(incomingChars);
        //Serial.print(", Setting temperature setpoint to ");
        //Serial.print(temp); 
        //Serial.println(". Pending SUCC/FAIL if temperature reaches setpoint");
        hasReportedStable = false;
        //print SUCC and hasReportedStable=true once temperature is stable at setpoint
    }
    else if (strcmp(incomingParams[1], "pwm") == 0)
    {
        double pwm;
        if (!isValidDoubleInRange(incomingParams[2], pwm, 0.0, 255.0))
        {
            return;
        }

        printlnACK();
        
        if (heaterPID.GetMode() == AUTOMATIC)
        {
            printFAIL();
            Serial.println("Cannot set MOSFET PWM while PID is on");
            return;
        }

        pwmOut = pwm;  
        printSUCC();
        Serial.print("Setting MOSFET PWM to ");
        Serial.println(pwm);
    }
    else if (strcmp(incomingParams[1], "Kp") == 0)
    {
        double gain;
        if (!isValidDoubleInRange(incomingParams[2], gain, 0.0, 1000.0))
        {
            return;
        }
        printlnACK();
        Kp = gain;       
        printSUCC();
        Serial.print("Setting proportional gain Kp to: ");
        Serial.println(gain); 
    }
    else if (strcmp(incomingParams[1], "Ki") == 0)
    {
        double gain;
        if (!isValidDoubleInRange(incomingParams[2], gain, 0.0, 1000.0))
        {
            return;
        }
        printlnACK();
        Ki = gain;     
        printSUCC();
        Serial.print("Setting integral gain Ki to: ");
        Serial.println(gain); 
    }
    else if (strcmp(incomingParams[1], "Kd") == 0)
    {
        double gain;
        if (!isValidDoubleInRange(incomingParams[2], gain, 0.0, 1000.0))
        {
            return;
        }
        printlnACK();
        Kd = gain;  
        printSUCC();
        Serial.print("Setting derivative gain Kd to: ");
        Serial.println(gain);
    }
    else
    {
        printNAK();
        Serial.print("Parameter \"");
        Serial.print(incomingParams[1]);
        Serial.println("\" does not exist or cannot be set.");
    }
}

void goToPrintCommand()
{
    if (numTokens != 2)
    {
        printNAK();
        Serial.println("Invalid number of parameters");
        return;
    }

    if (strcmp(incomingParams[1], "T") == 0)
    {  
        printlnACK();
        printSUCC();
        Serial.print("Current temperature is = ");
        Serial.println(latestTemp); 
    }
    else if (strcmp(incomingParams[1], "Ts") == 0)
    {  
        printlnACK();
        printSUCC();
        Serial.print("Temperature setpoint is = ");
        Serial.println(setTemp); 
    }
    else if (strcmp(incomingParams[1], "Tmin") == 0)
    {  
        printlnACK();
        printSUCC();
        Serial.print("Minimum temperature is = ");
        Serial.println(minTemp); 
    }
    else if (strcmp(incomingParams[1], "Tmax") == 0)
    {  
        printlnACK();
        printSUCC();
        Serial.print("Maximum temperature is = ");
        Serial.println(maxTemp); 
    }
    else if (strcmp(incomingParams[1], "pid") == 0)
    {  
        printlnACK();
        printSUCC(); 
        Serial.print("PID is currently = ");
        if (heaterPID.GetMode() == AUTOMATIC)
        {
            Serial.println("ON");
        }
        else
        {
            Serial.println("OFF");
        }
    }
    else if (strcmp(incomingParams[1], "pwm") == 0)
    {
        printlnACK();
        printSUCC();
        Serial.print("MOSFET PWM is = ");
        Serial.println(pwmOut); 
    }
    else if (strcmp(incomingParams[1], "Kp") == 0)
    {   
        printlnACK();
        printSUCC();
        Serial.print("Proportional gain Kp is = ");
        Serial.println(Kp); 
    }
    else if (strcmp(incomingParams[1], "Ki") == 0)
    {
        printlnACK();
        printSUCC();
        Serial.print("Integral gain Ki is = ");
        Serial.println(Ki); 
    }
    else if (strcmp(incomingParams[1], "Kd") == 0)
    {
        printlnACK();
        printSUCC();
        Serial.print("Derivative gain Kd is = ");
        Serial.println(Kd); 
    }
    else if (strcmp(incomingParams[1], "Kall") == 0)
    {
        printlnACK();
        printlnSUCC();
        //don't allow from pyserial unless can parse all 3 lines
        Serial.print("Proportional gain Kp is: ");
        Serial.println(Kp);
        Serial.print("Integral gain Ki is: ");
        Serial.println(Ki); 
        Serial.print("Derivative gain Kd is: ");
        Serial.println(Kd); 
    }
    else
    {
        printNAK();
        Serial.print("Parameter \"");
        Serial.print(incomingParams[1]);
        Serial.println("\" does not exist or cannot be printed.");
    }
}

void saveGains()
{
    if (numTokens != 2)
    {
        if (isDebugging)   
        {
            Serial.println("Invalid number of parameters");
        }
        return;
    }

    if (strcmp(incomingParams[1], "Kp") == 0)
    {   
        EEPROM.put(KpEEAddr, Kp);

        if (isDebugging)   
        {
            Serial.print("Saving proportional gain Kp = ");
            Serial.print(Kp); 
            Serial.println(" to EEPROM"); 
        }
    }
    else if (strcmp(incomingParams[1], "Ki") == 0)
    {
        EEPROM.put(KiEEAddr, Ki);
        
        if (isDebugging)
        {   
            Serial.print("Saving integral gain Ki = ");
            Serial.print(Ki); 
            Serial.println(" to EEPROM"); 
        }
    }
    else if (strcmp(incomingParams[1], "Kd") == 0)
    {
        EEPROM.put(KdEEAddr, Kd);

        if (isDebugging)   
        {
            Serial.print("Saving derivative gain Kd = ");
            Serial.print(Kd); 
            Serial.println(" to EEPROM");  
        }
    }
    else if (strcmp(incomingParams[1], "Kall") == 0)
    {
        EEPROM.put(KpEEAddr, Kp);

        if (isDebugging)   
        {
            Serial.print("Saving proportional gain Kp = ");
            Serial.print(Kp); 
            Serial.println(" to EEPROM");
            EEPROM.put(KiEEAddr, Ki);
            Serial.print("Saving integral gain Ki = ");
            Serial.print(Ki); 
            Serial.println(" to EEPROM");
            EEPROM.put(KdEEAddr, Kd);
            Serial.print("Saving derivative gain Kd = ");
            Serial.print(Kd); 
            Serial.println(" to EEPROM"); 
        }  
    }
    else
    {
        if (isDebugging)   
        {
            Serial.print("Parameter \"");
            Serial.print(incomingParams[1]);
            Serial.println("\" does not exist or cannot be saved.");
        }
    }
}

//setting, loading, saving, defaulting gains is only intended for manual control, not from pyserial
//therefore, ACK NAK SUCC FAIL is not implemented for most of the gain related functions
void loadGains()
{
    if (numTokens != 2)
    {
        if (isDebugging)   
        {
            Serial.println("Invalid number of parameters");
        }
        return;
    }

    if (strcmp(incomingParams[1], "Kp") == 0)
    {   
        EEPROM.get(KpEEAddr, Kp);

        if (isDebugging)   
        {
            Serial.print("Loading proportional gain Kp = ");
            Serial.print(Kp); 
            Serial.println(" from EEPROM"); 
        }
    }
    else if (strcmp(incomingParams[1], "Ki") == 0)
    {
        EEPROM.get(KiEEAddr, Ki);

        if (isDebugging)   
        {
            Serial.print("Loading integral gain Ki = ");
            Serial.print(Ki); 
            Serial.println(" from EEPROM"); 
        }
    }
    else if (strcmp(incomingParams[1], "Kd") == 0)
    {
        EEPROM.get(KdEEAddr, Kd);

        if (isDebugging)   
        {
            Serial.print("Loading derivative gain Kd = ");
            Serial.print(Kd); 
            Serial.println(" from EEPROM");  
        }
    }
    else if (strcmp(incomingParams[1], "Kall") == 0)
    {
        EEPROM.get(KpEEAddr, Kp);

        if (isDebugging)   
        {
            Serial.print("Loading proportional gain Kp = ");
            Serial.print(Kp); 
            Serial.println(" from EEPROM");
            EEPROM.get(KiEEAddr, Ki);
            Serial.print("Loading integral gain Ki = ");
            Serial.print(Ki); 
            Serial.println(" from EEPROM");
            EEPROM.get(KdEEAddr, Kd);
            Serial.print("Loading derivative gain Kd = ");
            Serial.print(Kd); 
            Serial.println(" from EEPROM");   
        }
    }
    else
    {
        if (isDebugging)   
        {
            Serial.print("Parameter \"");
            Serial.print(incomingParams[1]);
            Serial.println("\" does not exist or cannot be loaded.");
        }
    }
}

void resetGainsToDefault()
{
    if (numTokens != 2)
    {
        if (isDebugging)   
        {
            Serial.println("Invalid number of parameters");
        }
        return;
    }

    if (strcmp(incomingParams[1], "Kp") == 0)
    {   
        Kp = KpDefault;

        if (isDebugging)
        {
            Serial.print("Resetting proportional gain Kp to default =  ");
            Serial.println(Kp); 
        }
    }
    else if (strcmp(incomingParams[1], "Ki") == 0)
    {
        Ki = KiDefault;

        if (isDebugging)   
        {
            Serial.print("Resetting integral gain Ki to default =  ");
            Serial.println(Ki); 
        }
    }
    else if (strcmp(incomingParams[1], "Kd") == 0)
    {
        Kd = KdDefault;

        if (isDebugging)   
        {
            Serial.print("Resetting derivative gain Kd to default =  ");
            Serial.println(Kd);   
        }
    }
    else if (strcmp(incomingParams[1], "Kall") == 0)
    {
        Kp = KpDefault;
        Ki = KiDefault;
        Kd = KdDefault;

        if (isDebugging)  
        {
            Serial.print("Resetting proportional gain Kp to default =  ");
            Serial.println(Kp);
            Serial.print("Resetting integral gain Ki to default =  ");
            Serial.println(Ki);
            Serial.print("Resetting derivative gain Kd to default =  ");
            Serial.println(Kd);     
        }
    }
    else
    {
        if (isDebugging)
        {
            Serial.print("Parameter \"");
            Serial.print(incomingParams[1]);
            Serial.println("\" does not exist or cannot be reset to default.");
        }
    }
}

void setDebugOn()
{
    if (numTokens == 1)
    {
        isDebugging = true;
        Serial.println("Debug messages turned ON");
    }
    else
    {
        if (isDebugging)  
        {
            Serial.println("Invalid number of parameters");
        }
    }
}

void setDebugOff()
{
    if (numTokens == 1)
    {
        isDebugging = false;
        //should not be able to print below if false
        //Serial.println("Debug messages turned OFF");
    }
    else
    {
        if (isDebugging)  
        {
            Serial.println("Invalid number of parameters");
        }
    }
}

void goToCommand()
{
    //Run function based on first parsed token
    //(cannot use switch/case for cstring comparisons, it is actually better with the if-else ladder in this particular situation)
    if (strcmp(incomingParams[0], "pidon") == 0)
    {   
        setPIDOn();
    }
    else if (strcmp(incomingParams[0], "pidoff") == 0)
    {   
        setPIDOff();
    }
    else if (strcmp(incomingParams[0], "set") == 0)
    {   
        goToSetCommand();
    }
    else if (strcmp(incomingParams[0], "pr") == 0)
    {
        goToPrintCommand();
    }
    else if (strcmp(incomingParams[0], "save") == 0)
    {
        saveGains();
    }
    else if (strcmp(incomingParams[0], "load") == 0)
    {
        loadGains();
    }
    else if (strcmp(incomingParams[0], "default") == 0)
    {
        resetGainsToDefault();
    }
    else if (strcmp(incomingParams[0], "debugon") == 0)
    {
        setDebugOn();
    }
    else if (strcmp(incomingParams[0], "debugoff") == 0)
    {
        setDebugOff();
    }
    else if (strcmp(incomingParams[0], "whoru") == 0)
    {
        printWhoAmI();
    }
    else if (strcmp(incomingParams[0], "help") == 0)
    {
        printHelp();
    }
    else if (strcmp(incomingParams[0], "info") == 0)
    {
        printInfo();
    }
    else if (strcmp(incomingParams[0], "") == 0)
    {
        //Serial.println("Blank token");
        //this happens if only header and footer are sent
    }
    else
    {
        //no command exists
        printNAK();
        Serial.print("No command \"");
        Serial.print(incomingParams[0]);
        Serial.println("\" exists. Type \"help\' for more info.");
    }
}

void printWhoAmI()
{
    Serial.println(F("This is an Arduino controller for a heating stage"));
}

void printHelp()
{
   
}

void printInfo()
{
   
}

//printing control char utility functions
void printACK()
{
    Serial.print(charACK);
    Serial.print(": ");
}

void printlnACK()
{
    Serial.print(charACK);
    Serial.print(": ");
    Serial.println(incomingChars);
}

void printNAK()
{
    Serial.print(charNAK);
    Serial.print(": ");
}

void printlnNAK()
{
    Serial.println(charNAK);
}

void printSUCC()
{
    Serial.print(charSUCC);
    Serial.print(": ");
}


void printlnSUCC()
{
    Serial.println(charSUCC);
}

void printFAIL()
{
    Serial.print(charFAIL);
    Serial.print(": ");
}

void printlnFAIL()
{
    Serial.println(charFAIL);
}

void printWARN()
{
    Serial.print(charWARN);
    Serial.print(": ");
}

void printlnWARN()
{
    Serial.println(charWARN);
}
