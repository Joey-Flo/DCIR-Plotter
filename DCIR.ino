// Define PWM parameters
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
const int gateDriver = 18;
const int gateChannel = 1;
const int gateResolution = 10;
const int gateFreq = 40000;
float gateDutyCycle;
float SOCVOLTAGE;
int counter, counterSOC;
int diffVal, modeSelect;
int adc0, adc1, adc2, adc3, adc0sum, adc1sum, adc3sum, CurrentLimit, shuntDifferential;
double volts0, volts1, volts2, volts3, VoltageLevel;
double Voltage, PreviousCurrentVal, PreviousSupplyVoltage, DCIR, VOC, VL;
double CURRENTVAL;
unsigned long previousMillis;
int Timevalue;
bool SOCloop = false;
String serialString = "";
char serialData[20];  // Array to hold the raw incoming characters
float parsedData[10];
int floatIndex = 0; 
int RGBtimerHigh = 0;
int RGBvalHigh = 127.5; //RGB timing variables for high freq
int RGBsetHigh = 3;
float Intensity1HIGH = 0;
float Intensity2HIGH = 0; //RGB intensity for high frequency 
float Intensity3HIGH = 0;
const int colorCFR = 4;
const int colorCFG = 5;  // Color Freq R/G/B
const int colorCFB = 6;
int Mode = 0;
float progStart = 0;
bool VALIDTEST = true;
// Adjustable SOC Plot Parameters:
float MINVOLTAGE = 2.65; //in volts (meant to be lower than actual)
float MAXDISCHARGE = 0.85; // in Amps
float interval = 10000; // in ms (1-min recommended)
int OPENCIRCUITDELAY = 100; //in ms

// Adjustable DCIR as funciton of Current values:
float CURRENTSTEPSIZE = 0.05;// in Amps
int CURRENTSAMPLES = 10;// # of steps


// Constant Current Tune
double SHUNTOFFSET = 0.985; //(Accounts for shunt resistor tolerance)
float SHUNTVALUE = 0.1;


void RGBsetup(){ //Set's up RGB timing variables
  RGBtimerHigh = RGBtimerHigh + 1;
    if ((RGBtimerHigh % 1) == 0){
    RGBvalHigh = RGBvalHigh + 8;
  }
  if (RGBvalHigh >= 256){
    RGBsetHigh = RGBsetHigh + 1;
    RGBvalHigh = 0;
  }
  if (RGBsetHigh == 6){
    RGBsetHigh = 0;
  }
}


void RGBdriverHIGH(){ //Set's intensity of RGB vals for high freq light strip
  if (RGBsetHigh == 0){
    Intensity1HIGH = 255;
    Intensity2HIGH = RGBvalHigh;
    Intensity3HIGH = 0;
  }
  if (RGBsetHigh == 1){
    Intensity1HIGH = 255 - RGBvalHigh;
    Intensity2HIGH = 255;
    Intensity3HIGH = 0;
  }
  if (RGBsetHigh == 2){
    Intensity1HIGH = 0;
    Intensity2HIGH = 255;
    Intensity3HIGH = RGBvalHigh;
  }
  if (RGBsetHigh == 3){
    Intensity1HIGH = 0;
    Intensity2HIGH = 255 - RGBvalHigh;
    Intensity3HIGH = 255;
  }
  if (RGBsetHigh == 4){
    Intensity1HIGH = RGBvalHigh;
    Intensity2HIGH = 0;
    Intensity3HIGH = 255;
  }
  if (RGBsetHigh == 5){
    Intensity1HIGH = 255;
    Intensity2HIGH = 0;
    Intensity3HIGH = 255 - RGBvalHigh;
  }
}

void BatteryDischarge() {
    // Turn off the LED or indicator
    ledcWrite(colorCFB, 0);
    bool SOCloop = true;  // Main loop control for State of Charge monitoring
    float OCvolts = 0.0;  // Variable to store open-circuit voltage
    unsigned long startMillis = 0;  // Variable to store the start time for 10-second check

    while (SOCloop) {
        if ((ads.computeVolts(ads.readADC_SingleEnded(3)) >= SOCVOLTAGE) and (digitalRead(34) == 0)) {
            float initialOCVOLTAGE = ads.computeVolts(ads.readADC_SingleEnded(3));
            currentMonitor(MAXDISCHARGE);
            delay(300000);
            ledcWrite(gateChannel, 0);
            delay(OPENCIRCUITDELAY);
            float initialVOLTAGE = ads.computeVolts(ads.readADC_SingleEnded(3));
            delay(300000);
            float finalVOLTAGE = ads.computeVolts(ads.readADC_SingleEnded(3));
            float OCDifferential = finalVOLTAGE - initialVOLTAGE;
            Serial.println("");
            Serial.println(OCDifferential,4);
            bool SOCsubloop = true;  // Sub-loop for secondary checks
            startMillis = millis() - 10000; // Record the start time
            while (SOCsubloop) {
                // if (millis() - startMillis >= 10000) {
                //     ledcWrite(gateChannel, 0);
                //     delay(OPENCIRCUITDELAY);
                //     OCvolts = ads.computeVolts(ads.readADC_SingleEnded(3));
                //     startMillis = millis();
                // }
                // currentMonitor(MAXDISCHARGE);
                // delay(100);  // Short delay to stabilize the loop
                // if (OCvolts <= SOCVOLTAGE) {
                //     SOCloop = false;  // Exit the main SOC loop
                //     SOCsubloop = false; // Exit the sub-loop
                // }
            }
        }
        else{
          SOCloop = false;  // Exit the main SOC loop
        }
    }

    // Log data to the Serial monitor
    Serial.println("");
    Serial.print(1); Serial.print(";"); Serial.print(1); Serial.print(";"); Serial.println(0);

    // Perform a flashing sequence on pin 23
    for (int i = 0; i < 2; i++) {
        digitalWrite(23, HIGH);
        delay(50);
        digitalWrite(23, LOW);
        delay(50);
    }

    // Turn the LED or indicator back on
    ledcWrite(colorCFB, 255);
}


// Polts DCIR as a function of SOC
void SOCplot() {
  Serial.println("");
  Timevalue = 0;
  counterSOC = 0;
  bool SOCloop = true;
  while (SOCloop == true){
  if (digitalRead(34) == HIGH){
    SOCloop = false;
    break;
  }
  unsigned long currentMillis = millis();
    for (int i = 0; i < 20 ; i++){
      adc3sum = adc3sum + ads.readADC_SingleEnded(3);
    }
      VoltageLevel = (adc3sum / 20);
      adc3sum = 0;
    if (MINVOLTAGE < ads.computeVolts(VoltageLevel)){
      currentMonitor(MAXDISCHARGE);
      if (counterSOC == 0){
        previousMillis = currentMillis;
        ComputeDCIR();
        ledcWrite(gateChannel, 0);
        if (Mode == 1){
          ledcWrite(colorCFR, 255);
          ledcWrite(colorCFB, 255);
        }
        if (Mode == 2){
          ledcWrite(colorCFG, 255);
          ledcWrite(colorCFR, 255);
        }
        delay(100);
        for (int i = 0; i < 20 ; i++){
          adc3sum = adc3sum + ads.readADC_SingleEnded(3);
        }
        VoltageLevel = (adc3sum / 20);
        adc3sum = 0;
        Serial.println("");
        Serial.print(0); Serial.print(";"); Serial.print(ads.computeVolts(VoltageLevel),4); Serial.print(";"); Serial.println(DCIR * 1000);
        progStart = millis();
      }
      if (currentMillis - previousMillis >= interval) {
        Timevalue++;
        previousMillis = currentMillis;  // Update the time  // Increment the value
        ComputeDCIR();
        ledcWrite(gateChannel, 0);
        if (Mode == 1){
          ledcWrite(colorCFR, 255);
          ledcWrite(colorCFB, 255);
        }
        if (Mode == 2){
          ledcWrite(colorCFG, 255);
          ledcWrite(colorCFR, 255);
        }
        delay(100);
        for (int i = 0; i < 20 ; i++){
          adc3sum = adc3sum + ads.readADC_SingleEnded(3);
        }
        VoltageLevel = (adc3sum / 20);
        adc3sum = 0;
        Serial.println("");
        Serial.print(Timevalue); Serial.print(";"); Serial.print(ads.computeVolts(VoltageLevel),4); Serial.print(";"); Serial.println(DCIR * 1000);
        }
      counterSOC = counterSOC + 1;
      }
    else{
      SOCloop = false;
      break;
    }
  }
  ledcWrite(gateChannel, 0);
  ledcWrite(colorCFR, 255);
  ledcWrite(colorCFG, 255);
  ledcWrite(colorCFB, 255);
  delay(100);
  for (int i = 0; i < 20 ; i++){
  adc3sum = adc3sum + ads.readADC_SingleEnded(3);
  }
  VoltageLevel = (adc3sum / 20);
  adc3sum = 0;
  ComputeDCIR();
  Serial.println("");
  Serial.print((millis() - progStart) / interval); Serial.print(";"); Serial.print(ads.computeVolts(VoltageLevel),4); Serial.print(";"); Serial.println(DCIR * 1000);
  delay(100);
  Serial.println("");
  Serial.print(Timevalue + 1); Serial.print(";"); Serial.print(1); Serial.print(";"); Serial.println(0);
  digitalWrite(23,HIGH);
  delay(50);
  digitalWrite(23,LOW);
  delay(50);
  digitalWrite(23,HIGH);
  delay(50);
  digitalWrite(23,LOW);
}


// Polts DCIR as a function of Discharge Current
void CurrentPlot(){
  progStart = millis();
  Serial.println("");
  VALIDTEST = true;
  for (int i = 1; i < CURRENTSAMPLES + 1; i++){
    if (digitalRead(34) == HIGH){
      VALIDTEST = false;
    }
    if ((MINVOLTAGE < ads.computeVolts(ads.readADC_SingleEnded(3))) and (VALIDTEST == true)){
      if (digitalRead(34) == HIGH){
        VALIDTEST = false;
      }
      currentMonitor(CURRENTSTEPSIZE * i);
      delay(100);
      ComputeDCIR();
      Serial.print(i * CURRENTSTEPSIZE); Serial.print(";"); Serial.println(DCIR * 1000);
      Serial.println("");
    }
    else{
      Serial.print(Timevalue + 1); Serial.print(";"); Serial.print(1); Serial.print(";"); Serial.println(0);
      break;
    }
  }
  digitalWrite(23,HIGH);
  delay(50);
  digitalWrite(23,LOW);
  delay(50);
  digitalWrite(23,HIGH);
  delay(50);
  digitalWrite(23,LOW);
}


// Creates a constant current of input value
void currentMonitor(float CURRENTVAL) {
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  volts0 = ads.computeVolts(adc0);
  volts1 = ads.computeVolts(adc1);
  volts2 = ads.computeVolts(adc2);
  volts3 = ads.computeVolts(adc3);
  if (PreviousCurrentVal != CURRENTVAL) {
    CurrentLimit = ((CURRENTVAL * SHUNTVALUE) / 6.144) * 32767.5;
    for (int i = 0; i < 20 ; i++){
      adc0sum = adc0sum + ads.readADC_SingleEnded(0);
      adc1sum = adc1sum + ads.readADC_SingleEnded(1);
    }
    shuntDifferential = ((adc0sum - adc1sum) / 20) * SHUNTOFFSET;
    adc0sum = 0;
    adc1sum = 0;
  }
  if ((volts3 > (PreviousSupplyVoltage + 0.02)) or (volts3 < (PreviousSupplyVoltage - 0.02))){
    for (int i = 0; i < 20 ; i++){
      adc0sum = adc0sum + ads.readADC_SingleEnded(0);
      adc1sum = adc1sum + ads.readADC_SingleEnded(1);
    }
    shuntDifferential = ((adc0sum - adc1sum) / 20) * SHUNTOFFSET;
    adc0sum = 0;
    adc1sum = 0;
  }
  DutyCycleConversion(CURRENTVAL);
  if (digitalRead(34) == HIGH){
    SOCloop = false;
    VALIDTEST = false;
  }
  ledcWrite(gateChannel, gateDutyCycle);
  int RGBscale = (gateDutyCycle / 1024) * 255;
  if (Mode == 1){
    ledcWrite(colorCFR, (255 - RGBscale));
    ledcWrite(colorCFB, (255 - RGBscale));
  }
  if (Mode == 2){
    ledcWrite(colorCFG, (255 - RGBscale));
    ledcWrite(colorCFR, (255 - RGBscale));
  }
  PreviousCurrentVal = CURRENTVAL;
  PreviousSupplyVoltage = volts3;
}


// Calculates DCIR
void ComputeDCIR(){
    delay(30);
    if (digitalRead(34) == HIGH){
      SOCloop = false;
      VALIDTEST = false;
    }
    for (int i = 0; i < 20 ; i++){
    adc3sum = adc3sum + ads.readADC_SingleEnded(3);
    }
    VL = (adc3sum / 20);
    adc3sum = 0;
    for (int i = 0; i < 10 ; i++){
      adc0sum = adc0sum + ads.readADC_SingleEnded(0);
      adc1sum = adc1sum + ads.readADC_SingleEnded(1);
    }
    shuntDifferential = ((adc0sum - adc1sum) / 10) * SHUNTOFFSET;
    adc0sum = 0;
    adc1sum = 0;
    ledcWrite(gateChannel, 0);
    if (Mode == 1){
      ledcWrite(colorCFR, 255);
      ledcWrite(colorCFB, 255);
    }
    if (Mode == 2){
      ledcWrite(colorCFG, 255);
      ledcWrite(colorCFR, 255);
    }
    delay(OPENCIRCUITDELAY);
    if (digitalRead(34) == HIGH){
      SOCloop = false;
      VALIDTEST = false;
    }
    for (int i = 0; i < 20 ; i++){
      adc3sum = adc3sum + ads.readADC_SingleEnded(3);
    }
    VOC = (adc3sum / 20);
    adc3sum = 0;
    DCIR = abs(ads.computeVolts(VOC - VL) / (ads.computeVolts(shuntDifferential) * 10));
}


// Tunes Duty Cycle to desired value
void DutyCycleConversion(float CURRENTVAL){
if ((CurrentLimit > (shuntDifferential + 1)) or (CurrentLimit < (shuntDifferential - 1))){
  unsigned long startTime = millis();
  gateDutyCycle = ((CURRENTVAL * SHUNTOFFSET) / volts3) * 1024;
  while (CurrentLimit != shuntDifferential){
    if (digitalRead(34) == HIGH){
      VALIDTEST = false;
      SOCloop = false;
      break;
    }
    unsigned long currentTime = millis();
    if (currentTime - startTime >= 10000) {
      break;
    }
    if (CurrentLimit > shuntDifferential){
      float differential = abs(shuntDifferential - CurrentLimit);
      if (differential > CurrentLimit * 0.4){
        gateDutyCycle = gateDutyCycle + 5;
      }
      if ((differential > CurrentLimit * 0.07) and (differential < CurrentLimit * 0.4)){
        gateDutyCycle = gateDutyCycle + 3;
      }
      if ((differential > 0) and (differential < CurrentLimit * 0.07)){
        gateDutyCycle = gateDutyCycle + 1;
      }
      if (gateDutyCycle > 1024){
        gateDutyCycle = 1024;
      }
  }
    if (CurrentLimit < shuntDifferential){
      float differential = abs(shuntDifferential - CurrentLimit);
      if (differential > CurrentLimit * 0.4){
        gateDutyCycle = gateDutyCycle - 5;
      }
      if ((differential > CurrentLimit * 0.07) and (differential < CurrentLimit * 0.4)){
        gateDutyCycle = gateDutyCycle - 3;
      }
      if ((differential > 0) and (differential < CurrentLimit * 0.07)){
        gateDutyCycle = gateDutyCycle - 1;
      }
      if (gateDutyCycle < 0){
        gateDutyCycle = 0;
      }
  }
    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    volts0 = ads.computeVolts(adc0);
    volts1 = ads.computeVolts(adc1);
    CurrentLimit = ((CURRENTVAL * SHUNTVALUE) / 6.144) * 32767.5;
    shuntDifferential = (adc0 - adc1) * SHUNTOFFSET;
    if (ads.computeVolts(ads.readADC_SingleEnded(3)) < MINVOLTAGE){
      SOCloop = false;
      break;
    }
    ledcWrite(gateChannel, gateDutyCycle);
    int RGBscale = (gateDutyCycle / 1024) * 255;
    if (Mode == 1){
      ledcWrite(colorCFR, (255 - RGBscale));
      ledcWrite(colorCFB, (255 - RGBscale));
    }
    if (Mode == 2){
      ledcWrite(colorCFG, (255 - RGBscale));
      ledcWrite(colorCFR, (255 - RGBscale));
    }
    delay(10);
  }
}
}


void setup() {
  Serial.begin(19200);
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
    if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
  ledcSetup(gateChannel, gateFreq, gateResolution);
  ledcAttachPin(gateDriver, gateChannel);
  ledcSetup(colorCFR, 5000, 8);
  ledcAttachPin(5, colorCFR);
  ledcSetup(colorCFG, 5000, 8);   
  ledcAttachPin(17, colorCFG);
  ledcSetup(colorCFB, 5000, 8);
  ledcAttachPin(16, colorCFB);
  pinMode(23, OUTPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  ledcWrite(colorCFR,255);
  ledcWrite(colorCFG,255);
  ledcWrite(colorCFB,255);
  for (int i = 0; i < 85; i++){
    delay(5);
    ledcWrite(colorCFR, 255 - i * 3);
  }
  for (int i = 0; i < 85; i++){
    delay(5);
    ledcWrite(colorCFR, i * 3);
  }
  for (int i = 0; i < 85; i++){
    delay(5);
    ledcWrite(colorCFG, 255 - i * 3);
  }
  for (int i = 0; i < 85; i++){
    delay(5);
    ledcWrite(colorCFG, i * 3);
  }
  for (int i = 0; i < 85; i++){
    delay(5);
    ledcWrite(colorCFB, 255 - i * 3);
  }

delay(100);
digitalWrite(23,HIGH);
delay(100);
digitalWrite(23,LOW);
delay(100);
}



void loop() {
  ledcWrite(gateChannel, 0);
  Serial.println(ads.computeVolts(ads.readADC_SingleEnded(3)));
  RGBsetup();
  RGBdriverHIGH();
  int Datacounter = 0;
  while (Serial.available() > 0) {
    char incomingChar = (char)Serial.read();
    serialData[Datacounter] = incomingChar;
    Datacounter++;
    if (Datacounter >= 20) {
      break;
    }
  }
  delay(40);
  serialString = String(serialData);
  int startIdx = 0, endIdx;
  while ((endIdx = serialString.indexOf(';', startIdx)) != -1) {
    parsedData[floatIndex++] = serialString.substring(startIdx, endIdx).toFloat();
    startIdx = endIdx + 1;
  }
  if (startIdx < serialString.length()) {
    parsedData[floatIndex++] = serialString.substring(startIdx).toFloat();
  }
  floatIndex = 0;
  if (parsedData[2] != 0){
    delay(2000);
  }
  if (parsedData[0] == 3){
    ledcWrite(colorCFR, 255);
    ledcWrite(colorCFG, 255);
    ledcWrite(colorCFB, 255);
    SOCVOLTAGE = parsedData[1];
    MAXDISCHARGE = parsedData[2];
    BatteryDischarge();
  }
  if (parsedData[0] == 2){
    Mode = 2;
    ledcWrite(colorCFR, 255);
    ledcWrite(colorCFG, 255);
    ledcWrite(colorCFB, 255);
    CURRENTSTEPSIZE = parsedData[1];
    CURRENTSAMPLES = parsedData[2];
    MINVOLTAGE = parsedData[3];
    CurrentPlot();
  }
  if (parsedData[0] == 1){
    Mode = 1;
    ledcWrite(colorCFR, 255);
    ledcWrite(colorCFG, 255);
    ledcWrite(colorCFB, 255);
    MINVOLTAGE = parsedData[1];
    MAXDISCHARGE = parsedData[2];
    interval = parsedData[3];
    SOCplot();
  }

  for (int i = 0; i < sizeof(parsedData); i++) {
    parsedData[i] = 0;
}
ledcWrite(colorCFR, (255 - Intensity1HIGH));
ledcWrite(colorCFG, (255 - Intensity2HIGH));
ledcWrite(colorCFB, (255 - Intensity3HIGH));
Mode = 0;
}

