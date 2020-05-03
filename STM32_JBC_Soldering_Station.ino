#include <EEPROM.h>
#include <Wire.h>
#include <TFT_eSPI.h> // Graphics and font library for ILI9341 driver chip
#include <SPI.h>

#define HEAT_PIN PB9
#define TC_PIN PA3
#define THERMISTOR_PIN PA1
#define HEAT_LED PA8
#define UP_BTN_PIN PA9       //momentary switch
#define DOWN_BTN_PIN PA10     //momentary switch
#define INTERRUPT_PIN PA2
//#define INTERRUPT_PIN PA10
#define R5_RESISTANCE 49700.0  //single supply board gain
#define R6_RESISTANCE 249.0
#define VCC 3.3
#define bLED PB6
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

//Power Computation
int power = 0;
int counter = 0;
int halfCycleOn = 0;
int oldPower;
int tipTempIsDisplayLastRound = 0;
int pwrPot = PA0;
int encoderPinA = PB13;   // right
int encoderPinB = PB14;   // left
static boolean rotating = false;// debounce management
boolean A_set = false;
boolean B_set = false;
int button = PB15;

int tipTempIs = 0;
int tipTempIsDisplay = 0;   // Separate Display variables
int tipTempIsDisplayRound = 0;
int tipTempSet = 330;       //default tip temperature setting
int tipTempSetDisplay = 0;  // for asynchronous updates
//const int thermistorB = 4250; // B-constant for muRata NXRT15WF104FA1B030 thermistor
bool heat = false;
int mainsCycles = 0;  //This cannot be unsigned
unsigned long buttonMillis = 0; // When last button press was registered
unsigned long lastmillis = 0;
unsigned long calLastMillis = 0;
unsigned long uplastmillis = 0;
unsigned long displayLastmillis = 0;
int reading;           // the current reading from the input pin
int previousHome = LOW;    // the previous reading from the input pin
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers
int state = HIGH;      // the current state of the output pin
int tipAddress = 0;
int calAddress = 5;
unsigned long tempAdj;

float pwrAdj;
int current;         // Current state of the button
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
byte previousSet = HIGH;
unsigned long firstTime; // how long since the button was first pressed
bool setScreen;
bool calScreen;
int opAmp;


float getAmbientTemperature() {
  // Calculates °C from RTD voltage divider
  double Temp = log(10000.0 * ((4095.0 / analogRead(THERMISTOR_PIN) - 1)));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  return Temp - 273.15; // calibration factor
}

float runningAverage(float M) {  //Rounding for op-amp
#define LENGTH_M 10              //This can be increased/decreased depending on op-amp stability. Default 20
  static int values[LENGTH_M];
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;
  sum -= values[index];
  values[index] = M;
  sum += values[index];
  index++;
  index = index % LENGTH_M;
  if (count < LENGTH_M) count++;
  return sum / count;
}

float runningAverageDisplay(float D) {  //Rounding for display temp
#define LENGTH_D 10                       //How much averaging 
  static int values[LENGTH_D];
  static byte index = 0;
  static float sum = 0;
  static byte count = 0;
  sum -= values[index];
  values[index] = D;
  sum += values[index];
  index++;
  index = index % LENGTH_D;
  if (count < LENGTH_D) count++;
  return sum / count;
}

void zeroCrossingInterrupt() {  //Heater ISR
  mainsCycles++;
  counter++;

  if (mainsCycles >= 0) { // At 0 turn off heater
    digitalWrite(HEAT_PIN, LOW);//use arduino function (slower)
    //GPIOB->ODR &= ~(1 << 9);      //use direct port manipulation for ISR (stm32 PB9 LOW)
  }
  if (mainsCycles > 6) { // Default 6, make sure op-amp voltage is stable before reading thermocouple(50ms)
    tipTempIs = round(runningAverage(((analogRead(TC_PIN) * (VCC / 4095.0)) / (1 + R5_RESISTANCE / R6_RESISTANCE)) * tempAdj + getAmbientTemperature()));
    //Calculate seperate rounding for the display now
    tipTempIsDisplay = tipTempIs;                                         
    tipTempIsDisplayRound = round(runningAverageDisplay(tipTempIsDisplay));

    if (tipTempIs < tipTempSet) { // If heat is missing turn on heater
      digitalWrite(HEAT_PIN, HIGH);  //use arduino function (slower)
      //GPIOB->ODR |= (1 << 9);       //use direct port manipulation for ISR (stm32 PB9 HIGH)
      mainsCycles = round(sqrt(tipTempSet - tipTempIs) * pwrAdj);  //use pot to adjust multiplier, if set to 0 will still wait until next main cycle
    }
    else { // If no heat is missing
      mainsCycles = -4;  //default -4  This could be removed
    }
  }
  if (digitalRead(HEAT_PIN) == HIGH) {
    halfCycleOn++;
  }
  if (counter >= 60) {  //take a sample every half second
    counter = 0;
    power = ((halfCycleOn / 45.0) * 100.0); //give percentage of power being used, because of op-amp measurment time, about 75% of power is available
    halfCycleOn = 0;
  }
}

void defaultTipSet() {

  current = digitalRead(button); //could make this check at an interval

  if (current == LOW && previousSet == HIGH && (millis() - firstTime) > 200) {    // if the button state changes to pressed, remember the start time
    firstTime = millis();
  }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  if (millis_held > 100) {  //debouncing
    if (current == HIGH && previousSet == LOW) {       // check if the button was released since last checked
      if (secs_held >= 3  && setScreen == false) {     // Button held for 1-3 seconds
        (setScreen = true);
      }
    }
  }
  if (setScreen == true && secs_held < 1 )  {
    EEPROM.put(tipAddress, tipTempSet);
    firstTime = millis();
    if (millis() - lastmillis >= 1000) { 
      lastmillis = millis();
      (calScreen = true);
      (setScreen = false);
    }
  }
  if (calScreen == true && (digitalRead(button) == LOW)) {
    EEPROM.put(calAddress, tempAdj);
    if (millis() - calLastMillis >= 1000) { // update the tip display every secondfirstTime = millis();
      calLastMillis = millis();
      tft.drawString("       ", 3, 145, 4);  //make sure the cal is removed from screen
      (calScreen = false);
    }
  }
  if (setScreen == true) {
    if (millis() - lastmillis >= 1000) { // update the tip display every second
      lastmillis = millis();
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.drawString("tip", 3, 5, 4);
    }
    else if (millis() - lastmillis >= 250) {     
      tft.drawString("       ", 3, 5, 4);
    }
  }
  previousSet = current;

  if (calScreen == true) {
    if (millis() - lastmillis >= 1000) { // update the tip display every second
      lastmillis = millis();
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.drawString("cal", 3, 145, 4);
    }
    else if (millis() - lastmillis >= 250) {
      tft.drawString("       ", 3, 145, 4);
      tempAdj = constrain(tempAdj, 20000, 80000);  //does this work here?
    }
  }
}

void momNo() { //momentary normally open switches

  if (!digitalRead(UP_BTN_PIN) && tipTempSet < 400 && millis() > buttonMillis + 10) {
    tipTempSet++;
    buttonMillis = millis();
  }
  if (!digitalRead(DOWN_BTN_PIN) && tipTempSet > 0 && millis() > buttonMillis + 10) {
    tipTempSet--;
    buttonMillis = millis();
  }
}

void rotaryPush() {

  reading = digitalRead(button);

  if (reading == HIGH && previousHome == LOW && millis() - time > debounce) {
    if (state == HIGH) {
      state = LOW;
      tipTempSet = EEPROM.get(tipAddress, tipTempSet);
    }
    else {
      state = HIGH;
      tipTempSet = 150;   //Toggle between these temps with rotary pushbutton for normal/rest state
    }
    time = millis();
  }
  previousHome = reading;
}

void doEncoderA() {   // Interrupt on A changing state
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  if ( digitalRead(encoderPinA) != A_set ) { // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
      if (calScreen == false) {
        tipTempSet = tipTempSet + 5;
      }
      else {
        tempAdj = tempAdj + 100;
      }
    rotating = false;  // no more debouncing until loop() hits again
  }
}

void doEncoderB() {   // Interrupt on B changing state, same as A above
  if ( rotating ) delay (1);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      if (calScreen == false) {
        tipTempSet = tipTempSet - 5;
      }
      else {
        tempAdj = tempAdj - 100;
      }
    rotating = false;
  }
}

void heaterLed() {                                                //LED or display heat on indicator
  if (digitalRead(HEAT_PIN) == HIGH) {                            //This could just be turned on and off when the heatpin is
    digitalWrite(HEAT_LED, HIGH);
  }
  else {
    digitalWrite(HEAT_LED, LOW);
  }
}

void setup()
{
  pinMode(bLED, OUTPUT);    //using GPIO for backlighting the display for the time being
  digitalWrite(bLED, HIGH); //doesn't seem to be much voltage drop
  tft.init();
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.drawString("Set:         `c", 70, 5, 4);
  tft.drawString("Tip:                   `c", 0, 100, 4);
  tft.drawString("Pwr: ", 70, 190, 4);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  attachInterrupt(PB13, doEncoderA, CHANGE);
  attachInterrupt(PB14, doEncoderB, CHANGE);
  attachInterrupt(INTERRUPT_PIN, zeroCrossingInterrupt , RISING);

  pinMode(TC_PIN, INPUT);
  pinMode(pwrPot, INPUT);
  pinMode (button, INPUT_PULLUP);         //Rotary Encoder
  pinMode(UP_BTN_PIN, INPUT_PULLUP);     //momentary switch
  pinMode(DOWN_BTN_PIN, INPUT_PULLUP);   //momentary switch
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(HEAT_LED, OUTPUT);
  analogReadResolution(12);  //Run the ADC at its full resolution
  tipTempSet = EEPROM.get(tipAddress, tipTempSet);
  tempAdj = EEPROM.get(calAddress, tempAdj);
  //Serial.begin(9600);
}

void loop() {

  rotating = true;  // reset the debouncer for the rotary encoder

  if ((abs(tipTempIsDisplayRound - tipTempIsDisplayLastRound) >= 1) && (millis() - displayLastmillis >= 500)) {  //update the display at most every half second
    displayLastmillis = millis();
    tipTempIsDisplayLastRound = tipTempIsDisplayRound;
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("        ", 85, 75, 7);
    tft.drawNumber(tipTempIsDisplayRound, 85, 75, 7);
    //Serial.println(tipTempIsDisplayRound);
    pwrAdj = (analogRead(pwrPot) / -2400.0) ; //this will now only read when the temp is changing
  }

  if (abs(tipTempSet - tipTempSetDisplay) >= 1) // Is it time to update the display?
  {
    tipTempSetDisplay = tipTempSet;
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawNumber(tipTempSetDisplay, 160, 7, 4);
    //
    tipTempSet = constrain(tipTempSet, 100, 400);
  }

  if (setScreen == true) {
    //    momNo();
    defaultTipSet();
    heaterLed();
  }
  else {
    //momNo();       //uncomment to enable pushputtons
    defaultTipSet();
    rotaryPush();
    heaterLed();
  }

  if (millis() - uplastmillis >= 300) { // Display power update every half second
    uplastmillis = millis();
    if (power != oldPower) {
      oldPower = power;
      if (power >= 100) {
        power = 99;
      }
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.drawString("                       ", 165, 190, 4);//overwrite to get rid of old numbers
      tft.drawNumber(power, 175, 190, 4);
      if (power > 9) {
        tft.drawString("%", 235, 190, 4);//overwrite to get rid of old numbers
      }
      else {
        tft.drawString("%", 205, 190, 4);//overwrite to get rid of old numbers
      }
    }
  }
}
