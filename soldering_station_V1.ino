#include <EEPROM.h>               //Used to save setpoint when power-off
#include <avr/sleep.h>          // for sleeping during ADC sampling
#include <U8glib.h>             // https://github.com/olikraus/u8glib
#include "max6675.h"

//Type of encoder
#define ROTARY_TYPE    1        // 0: 2 increments/step; 1: 4 increments/step (default)

// Pins
#define SENSOR_PIN    A0        // temperature sensor (thermistor)
#define ROTARY_2_PIN   2        // rotary encoder 2
#define ROTARY_1_PIN   3        // rotary encoder 1
#define BUZZER_PIN     4        // PIN D4 buzzer
#define BUTTON_PIN     5        // rotary encoder switch
#define THERMO_SO      6        // THERMOCOUPLE pins
#define THERMO_CS      7
#define THERMO_CLK     8
#define CONTROL_PIN    11       // pin D11 relay
#define SWITCH_PIN     12       // tilt switch

// Default temperature values
#define TEMP_MIN 0
#define TEMP_MAX 450
#define TEMP_DEFAULT  150
#define TEMP_STEP      1       // rotary encoder temp change steps

// Default timer values (0 = disabled)
#define TIME2OFF      10        // time to shut off heater in minutes

// Control values
#define TIME2SETTLE   950       // time in microseconds to allow OpAmp output to settle
#define SMOOTHIE      0.05      // OpAmp output smooth factor (1=no smoothing; 0.05 default)
#define PID_ENABLE    true      // enable PID control
#define BEEP_ENABLE   true      // enable/disable buzzer
#define MAINSCREEN    1         // type of main screen (0: big numbers; 1: more info)

// EEPROM identifier
#define EEPROM_IDENT   0xE76C   // to identify if EEPROM was written by this program

// Default values that can be changed by the user and stored in the EEPROM
uint16_t  DefaultTemp = TEMP_DEFAULT;
uint8_t   time2off    = TIME2OFF;
uint8_t   MainScrType = MAINSCREEN;
bool      PIDenable   = PID_ENABLE;
bool      beepEnable  = BEEP_ENABLE;

// Menu items
const char *SetupItems[]       = { "Setup Menu", "Temp Settings",
                                   "Timer Settings", "Control Type", "Main Screen",
                                   "Buzzer", "Information", "Turn OFF", "Return" };
const char *TempItems[]        = { "Temp Settings", "Default Temp", "Return" };
const char *TimerItems[]       = { "Timer Settings","Off Timer", "Return" };
const char *ControlTypeItems[] = { "Control Type", "Direct", "PID" };
const char *MainScreenItems[]  = { "Main Screen", "Big Numbers", "More Infos" };
const char *StoreItems[]       = { "Store Settings ?", "No", "Yes" };
const char *SureItems[]        = { "Are you sure ?", "No", "Yes" };
const char *BuzzerItems[]      = { "Buzzer", "Disable", "Enable" };
const char *DefaultTempItems[] = { "Default Temp", "deg C" };
const char *OffTimerItems[]    = { "Off Timer", "Minutes" };

// Variables for pin change interrupt
volatile uint8_t  a0, b0, c0, d0;
volatile bool     ab0;
volatile int      count, countMin, countMax, countStep;
volatile bool     handleMoved;

// State variables
//TODO: handle this flags and states properly
volatile enum State {heating, controlling, shut_down} state;
bool      inOffMode   = false;
bool      isWorky     = true;
bool      beepIfWorky = true;
bool showPopup = false;
bool IdleScreen = false;
bool systemoff = false;

// Timing variables
unsigned long sleepmillis;
unsigned long elapsedSeconds;
unsigned long remainingSeconds;
const unsigned long warningInterval = 30;
uint32_t  buttonmillis;
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 5;
unsigned long lastReadTime = 0;         // last temperature read (for thermocouple)
const unsigned long readInterval = 250; // time between readings in milliseconds

// Variables for temperature control
uint16_t  SetTemp, ShowTemp, gap, Step;
double    Input, Output, Setpoint, RawTemp, CurrentTemp, ChipTemp;

float real_temp;           //We will store here the real temp 
float SetpointDiff = 15;   //In degrees C
float elapsedTime, now_time, prev_time;        //Variables for time control
float refresh_rate = 200;                   //PID loop time in ms
float now_pid_error, prev_pid_error;


///////////////////PID constants///////////////////////
// // Define the aggressive and conservative PID tuning parameters
// double aggKp=11, aggKi=0.5, aggKd=1;
// double consKp=11, consKi=3, consKd=5;
float kp=3.5;         //Mine was 2.5
float ki=0.06;         //Mine was 0.06
float kd=0.8;         //Mine was 0.8
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

// Setup u8g object: uncomment according to the OLED used
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);
// U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_FAST|U8G_I2C_OPT_NO_ACK);
MAX6675 thermocouple(THERMO_CLK, THERMO_CS, THERMO_SO);


void setup() {
  Serial.begin(250000);
  analogReference(EXTERNAL);
  
  // Retrieve last stored set temperature from EEPROM
  getEEPROM();
  
  //pin modes
  pinMode(SWITCH_PIN,   INPUT);
  pinMode(CONTROL_PIN,  OUTPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  pinMode(ROTARY_1_PIN, INPUT_PULLUP);
  pinMode(ROTARY_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN,   INPUT_PULLUP);
  digitalWrite(CONTROL_PIN, HIGH);       // Initialize RELAY pin as high (Off)
  digitalWrite(BUZZER_PIN, LOW);         // must be LOW when buzzer not in use

  TCCR2B = TCCR2B & B11111000 | B00000111;    // D11 PWM is now 30.64 Hz

  // // setup ADC
  // ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);  // set ADC prescaler to 128
  // ADCSRA |= bit (ADIE);                 // enable ADC interrupt
  sei();                                // enable global interrupts
  
  // setup pin change interrupt for rotary encoder
  PCMSK2 = bit (PCINT19);  // Enable pin change interrupt for D2 (PCINT18) and D3 (PCINT19)
  PCICR  = bit (PCIE2);    // Enable pin change interrupt for PCINT[23:16] group
  PCIFR  = bit (PCIF2);    // Clear interrupt flag for PCINT[23:16]
  // set initial rotary encoder values
  a0 = PIND >> 2 & 1; b0 = PIND >> 3 & 1; ab0 = (a0 == b0);
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);

  //Display:
  if      ( u8g.getMode() == U8G_MODE_R3G3B2 )   u8g.setColorIndex(255);
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) u8g.setColorIndex(3);
  else if ( u8g.getMode() == U8G_MODE_BW )       u8g.setColorIndex(1);
  else if ( u8g.getMode() == U8G_MODE_HICOLOR )  u8g.setHiColorByRGB(255,255,255);

  // read and set current iron temperature
  SetTemp  = DefaultTemp;
  read_temperature();
  
  // if ((CurrentTemp + 20) < DefaultTemp) digitalWrite(CONTROL_PIN, LOW);
  if ((CurrentTemp + SetpointDiff) < DefaultTemp) state = heating;
  
  // reset sleep timer
  sleepmillis = millis();

  // beep on power up even if buzzer is disabled
  startup_beep();
  // delay(500); //for max6675
}

void loop() {
  ROTARYCheck();      // check rotary encoder (temp/boost setting, enter setup menu)
  
  SLEEPCheck();       // check and activate/deactivate sleep modes

  read_temperature();

  switch(state)	{
		case heating: //first state
      systemoff = false;
      ramp_up();
      break;

    case controlling: //main state
      systemoff = false;
      PID_control();
      if(real_temp < (Setpoint - SetpointDiff)){
        state = heating;
      }
      break;
    
    case shut_down: // this state is just used for when the system is off. (so its almost useless)
      cool_down();
      break;

    //TODO: manage the isworky etc states here
  }
  // check_movement();   // read vibration switch
  
  if      (inOffMode)   Setpoint = 0;
  else                  Setpoint = SetTemp;
  gap = abs(Setpoint - CurrentTemp);

  // MainScreen();       // updates the main page on the OLED
  if      (showPopup)  PopupScreen(remainingSeconds);  // Show countdown if popup is active
  else if (IdleScreen)  OffScreen();                    // Show idle screen
  else                 MainScreen();                   // Show the regular screen
  // Serial.println(sleepmillis);
  // Serial.println(handleMoved);
  // delay(100);
  // if(state == heating) Serial.println("RAMP UP");
  // else if(state == shut_down) Serial.println("SHUT DOWN");
  // else Serial.println("CONTROLLING");
  // delay(100);
  // Serial.println(real_temp);
}

// Encoder
void ROTARYCheck() {
  // set working temperature according to rotary encoder value
  SetTemp = getRotary();
  
  // check rotary encoder switch
  uint8_t c = digitalRead(BUTTON_PIN);
  if ( !c && c0 ) {
    beep();
    buttonmillis = millis();
    while( (!digitalRead(BUTTON_PIN)) && ((millis() - buttonmillis) < 500));
    //long press saves current temp
    if ((millis() - buttonmillis) >= 500){
      beep();
      DefaultTemp = SetTemp;
      updateEEPROM();
    }
    //short press opens menu
    else {
      handleMoved = true;
      SetupScreen();
    }
  }
  c0 = c;
}
void setRotary(int rmin, int rmax, int rstep, int rvalue) {
  countMin  = rmin << ROTARY_TYPE;
  countMax  = rmax << ROTARY_TYPE;
  countStep = rstep;
  count     = rvalue << ROTARY_TYPE;  
}
int getRotary() {
  return (count >> ROTARY_TYPE);
}

bool countdownBeeped = false;
void SLEEPCheck() {
  if (handleMoved) {
    if ((CurrentTemp + 20) < SetTemp) // if temp is well below setpoint
      digitalWrite(CONTROL_PIN, LOW); // then start the heater right now
    beepIfWorky = true;               // beep again when working temperature is reached
    handleMoved = false;              // reset handleMoved flag
    inOffMode   = false;              // reset off flag
    sleepmillis = millis();           // reset sleep timer
    countdownBeeped = false;          // reset beep flag
    showPopup  = false;               // Reset popup flag if handle was moved
    if (systemoff == false) IdleScreen = false;        // Reset Idle flag if handle was moved
  }

  // // check time passed since the handle was moved
  elapsedSeconds   = (millis() - sleepmillis) / 1000;
  remainingSeconds = (time2off*60) - elapsedSeconds;
  
  // If less than a minute is left, activate the popup screen
  if ((remainingSeconds >= 1)  && (remainingSeconds <= warningInterval) && (!inOffMode && (time2off > 0)) && (IdleScreen == false)) {
    showPopup = true;               // Enable the popup
    PopupScreen(remainingSeconds);  //countdown popup screen
    //beep twice
    if (!countdownBeeped) {
      beep(); delay(100); beep();
      countdownBeeped = true;       // Set flag to prevent further beeps
    }
  }

  // if shutdown time has passed
  if ((remainingSeconds < 1) && (!inOffMode) && (time2off > 0) && (!systemoff)){
    inOffMode = true;
    shutdown_beep();
    showPopup = false;  // Disable the popup when the system turns off
    IdleScreen = true;   // start idle sreen
    systemoff = true;    //turn off system
    if (state == heating || state == controlling) {
      state = shut_down;
    }
    else if (state == shut_down) {
      state = heating;
    }
  }
}


void read_temperature(){
  unsigned long thermoTime = millis();
   if (thermoTime - lastReadTime >= readInterval) {
    lastReadTime = thermoTime; // update the time of the last reading

    real_temp = thermocouple.readCelsius();
    // delay(250); //for max6675

  }
  
  ShowTemp = real_temp;
  CurrentTemp = real_temp;

  // Stabilize displayed temperature when around setpoint
  if ((ShowTemp != Setpoint) || (abs(ShowTemp - CurrentTemp) > 5)) ShowTemp = CurrentTemp;
  if (abs(ShowTemp - Setpoint) <= 1) ShowTemp = Setpoint;

  // Set state variable if temperature is in working range; beep if working temperature was just reached
  gap = abs(SetTemp - CurrentTemp);
  if (gap < 5) {
    // if (!isWorky && beepIfWorky) beep();
    isWorky = true;
    beepIfWorky = false;
  }
  else {
    isWorky = false;
  }
}

//change this one for pin change interrupt on pin 12 (for the tilt switch)
void check_movement(){
  uint8_t d = digitalRead(SWITCH_PIN);        // check handle vibration switch
  if (d != d0) {handleMoved = true; d0 = d;}  // set flag if handle was moved
}



//Fucntion for ramping up the temperature
void ramp_up(void){  
  //Rising temperature to (Setpoint - SetpointDiff)
  elapsedTime = millis() - prev_time; 
  if(elapsedTime > refresh_rate){  
    read_temperature();
    
    if(real_temp < (Setpoint - SetpointDiff)){
      digitalWrite(CONTROL_PIN, LOW);                //Turn On SSR
      // Output = 255;
      // PID_control();
      // handleMoved = true;
    }
    else
    {
      digitalWrite(CONTROL_PIN, HIGH);                 //Turn Off SSR
      state = controlling;                         //Already hot so we go to PID control
    }
    
    // display.clearDisplay();  
    // display.setCursor(0,0);           
    // display.print("Set: "); 
    // display.println(Setpoint,1);
    
    // display.print((char)247); 
    // display.print("C: "); 
    // display.println(real_temp,1);     
    // display.print("Ramp Up");   
    // display.display();//Finally display the created image
    // Serial.println(real_temp);                //For debug only
    prev_time = millis();
  }
}//End of ramp_up loop

//Main PID compute and execute function
void PID_control(void){
  // if      (inOffMode)   Setpoint = 0;
  // else                  Setpoint = SetTemp;

  elapsedTime = millis() - prev_time;   
  if(elapsedTime > refresh_rate){    
    //1. We get the temperature and calculate the error
    read_temperature(); //update real_temp
    now_pid_error = Setpoint - real_temp;

    if (PIDenable) {
      // Input = CurrentTemp;
      // if (gap < 30) ctrl.SetTunings(consKp, consKi, consKd);
      // else ctrl.SetTunings(aggKp, aggKi, aggKd); 
      // ctrl.Compute();

      //2. We calculate PID values
      PID_p = kp * now_pid_error;
      PID_d = kd*((now_pid_error - prev_pid_error)/refresh_rate);
      //2.2 Decide if we apply I or not. Only when error is very small
      if(-3 < now_pid_error && now_pid_error < 3){
        PID_i = PID_i + (ki * now_pid_error);}
      else{PID_i = 0;}

      //3. Calculate and map total PID value
      Output = PID_p + PID_i + PID_d;  
      Output = map(Output, 0, 150, 0, 255);

      //4. Set limits for PID values
      if(Output < 0){Output = 0;}
      if(Output > 255) {Output = 255; }
      // Output = constrain(Output, 0, 255); //try this instead here?

      //5. Write PWM signal to the SSR
      analogWrite(CONTROL_PIN, 255-Output);
    }
    else {
      // turn on heater if current temperature is below setpoint
      if ((CurrentTemp + 10) < Setpoint) Output = 0; else Output = 255;
      digitalWrite(CONTROL_PIN, Output);     // set heater PWM
    }
    
    //7. Save values for next loop
    prev_time = millis();                       //Store time for next loop
    prev_pid_error = now_pid_error;             //Store error for next loop
    //Serial.println(elapsedTime);                //For debug only
  }  
}//End PID_control loop

//Function for turning off everything and monitoring the coolidn down process
void cool_down(void){
  digitalWrite(CONTROL_PIN, HIGH);    //SSR is OFF with HIGH pulse!
  elapsedTime = millis() - prev_time;   
  if(elapsedTime > refresh_rate){  
    // display.clearDisplay();
            
    // display.setCursor(0,0);  
    // display.print("Set: "); 
    // display.println(Setpoint,1);

    // display.print("Off");       
    // display.display();//Finally display the created image
    prev_time = millis();
  }
}//End cool_down loop


// draws the main screen
//change this with an if (if popup show popup, if system off show system off, else show main screen)
void MainScreen() {
  u8g.firstPage();
  do {
    // draw setpoint temperature
    u8g.setFont(u8g_font_9x15);
    u8g.setFontPosTop();
    u8g.drawStr( 0, 0,  "SET:");
    u8g.setPrintPos(40,0);
    u8g.print(Setpoint, 0);

    // draw status of heater
    u8g.setPrintPos(83,0);
    if (ShowTemp > 500)    u8g.print(F("ERROR"));
    else if (inOffMode)    u8g.print(F("  OFF"));
    else if (isWorky)      u8g.print(F("WORKY"));
    else if (Setpoint < real_temp) u8g.print(F(" COOL"));
    else if (Output < 150) u8g.print(F(" HEAT"));
    else                   u8g.print(F(" HOLD"));

    // rest depending on main screen type
    if (MainScrType) {
      // draw control type
      u8g.setFont(u8g_font_9x15);
      u8g.setFontPosTop(); 
      if (PIDenable) {
        u8g.drawStr( 0, 17,  "PID:");
        // draw pwm percentage      
        u8g.setPrintPos(0,32);
        if (state == heating) u8g.print("Full ");
        else u8g.print(map(Output, 0, 255, 0, 100)); u8g.print(F("%"));
      }
      else {
        u8g.drawStr( 0, 17,  "DIRECT:");
        u8g.setPrintPos(0,32); //map(Output, 255, 0, 0, 1); //u8g.print(map(Output, 255, 0, 0, 1));
        if(Output == 255) u8g.print(F(" OFF"));
        else u8g.print(F(" ON"));
      }

      //print state
      u8g.setPrintPos( 0,52); 
      if(state == heating) u8g.print("RAMP UP");
      else if(state == shut_down) u8g.print("SHUT DOWN");
      else u8g.print("CONTROLLING");
          
      // draw current temperature
      u8g.setFont(u8g_font_freedoomr25n);
      u8g.setFontPosTop();
      u8g.setPrintPos(67,22);
      if (ShowTemp > 500) u8g.print(F("000")); else u8g.print(ShowTemp);
    }
    else {
      // draw current temperature in big figures
      u8g.setFont(u8g_font_fub42n);
      u8g.setFontPosTop();
      u8g.setPrintPos(15,20);
      if (ShowTemp > 500) u8g.print(F("000")); else u8g.print(ShowTemp);
    }
  } while(u8g.nextPage());
}
void PopupScreen(int remainingSeconds) {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_9x15);
    u8g.setFontPosTop();
    u8g.setPrintPos(30,10);
    u8g.print("OFF in:");
    u8g.setPrintPos(50, 30);
    u8g.print(remainingSeconds);
    u8g.print("s");
  } while(u8g.nextPage());
}
void OffScreen() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_9x15);
    u8g.setFontPosTop();

    u8g.setPrintPos(35,10);
    u8g.print("System");
    u8g.setPrintPos(50,30);
    u8g.print("Off");

  } while(u8g.nextPage());
}

void SetupScreen() {
  // digitalWrite(CONTROL_PIN, HIGH);      // shut off heater
  // beep();
  uint16_t SaveSetTemp = SetTemp;
  uint8_t selection = 0;
  bool repeat = true;
  
  while (repeat) {
    PID_control(); //TODO: debug this
    selection = MenuScreen(SetupItems, sizeof(SetupItems), selection);
    switch (selection) {
      case 0:   TempScreen(); break;
      case 1:   TimerScreen(); repeat = false; break;
      case 2:   PIDenable = MenuScreen(ControlTypeItems, sizeof(ControlTypeItems), PIDenable); break;
      case 3:   MainScrType = MenuScreen(MainScreenItems, sizeof(MainScreenItems), MainScrType); repeat = false; break;
      case 4:   beepEnable = MenuScreen(BuzzerItems, sizeof(BuzzerItems), beepEnable); break;
      case 5:   InfoScreen(); break;
      case 6:   TurnOff(); repeat = false; break;
      default:  repeat = false; break;
    }
  }  
  updateEEPROM();
  handleMoved = true;
  SetTemp = SaveSetTemp;
  setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
}
void TurnOff(){
  state = shut_down; // shut heater off
  shutdown_beep();
  IdleScreen = true;
  systemoff = true;
  inOffMode = true;
}
// temperature settings screen
void TempScreen() {
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat) {
    selection = MenuScreen(TempItems, sizeof(TempItems), selection);
    switch (selection) {
      case 0:   setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
                DefaultTemp = InputScreen(DefaultTempItems); break;
      default:  repeat = false; break;
    }
  }
}
// timer settings screen
void TimerScreen() {
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat) {
    selection = MenuScreen(TimerItems, sizeof(TimerItems), selection);
    switch (selection) {
      case 0:   setRotary(0, 60, 1, time2off);
                time2off = InputScreen(OffTimerItems); break;
      default:  repeat = false; break;
    }
  }
}
// menu screen
uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected) {
  uint8_t lastselected = selected;
  int8_t  arrow = 0;
  if (selected) arrow = 1;
  numberOfItems >>= 1;
  setRotary(0, numberOfItems - 2, 1, selected);
  bool    lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g.firstPage();
      do {
        u8g.setFont(u8g_font_9x15);
        u8g.setFontPosTop();
        u8g.drawStr( 0, 0,  Items[0]);
        u8g.drawStr( 0, 16 * (arrow + 1), ">");
        for (uint8_t i=0; i<3; i++) {
          uint8_t drawnumber = selected + i + 1 - arrow;
          if (drawnumber < numberOfItems)
            u8g.drawStr( 12, 16 * (i + 1), Items[selected + i + 1 - arrow]);
        }
      } while(u8g.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {delay(10); lastbutton = false;}
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return selected;
}
// input value screen
uint16_t InputScreen(const char *Items[]) {
  uint16_t  value;
  bool      lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    value = getRotary();
    u8g.firstPage();
      do {
        u8g.setFont(u8g_font_9x15);
        u8g.setFontPosTop();
        u8g.drawStr( 0, 0,  Items[0]);
        u8g.setPrintPos(0, 32); u8g.print(">"); u8g.setPrintPos(10, 32);        
        if (value == 0)  u8g.print(F("Deactivated"));
        else            {u8g.print(value);u8g.print(" ");u8g.print(Items[1]);}
      } while(u8g.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {delay(10); lastbutton = false;}
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
  return value;
}
// information display screen
void InfoScreen() {
  bool lastbutton = (!digitalRead(BUTTON_PIN));

  do {
    u8g.firstPage();
      do {
        u8g.setFont(u8g_font_9x15);
        u8g.setFontPosTop();
        u8g.setPrintPos(0,  0); u8g.print(F("Info screen xd"));
      } while(u8g.nextPage());
    if (lastbutton && digitalRead(BUTTON_PIN)) {delay(10); lastbutton = false;}
  } while (digitalRead(BUTTON_PIN) || lastbutton);

  beep();
}


// EEPROM
void updateEEPROM() {
  EEPROM.update( 2, DefaultTemp >> 8);
  EEPROM.update( 3, DefaultTemp & 0xFF);
  EEPROM.update( 4, time2off);
  EEPROM.update( 5, MainScrType);
  EEPROM.update( 6, PIDenable);
  EEPROM.update( 7, beepEnable);
}
void getEEPROM() {
  uint16_t identifier = (EEPROM.read(0) << 8) | EEPROM.read(1);
  if (identifier == EEPROM_IDENT) {
    DefaultTemp = (EEPROM.read(2) << 8) | EEPROM.read(3);
    time2off    =  EEPROM.read(4);
    MainScrType =  EEPROM.read(5);
    PIDenable   =  EEPROM.read(6);
    beepEnable  =  EEPROM.read(7);
  }
  else {
    EEPROM.update(0, EEPROM_IDENT >> 8); EEPROM.update(1, EEPROM_IDENT & 0xFF);
    updateEEPROM();
  }
}


// beep on the buzzer
void beep(){
  if (beepEnable) {
    for (uint8_t i=0; i<255; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(125);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(125);
    }
  }
}
void startup_beep(){
  // Turn the buzzer on for fixed periods to create simple beeps
  tone(BUZZER_PIN, 1000);  // Play a 1 kHz tone
  delay(125);             // Play for 125 milliseconds
  noTone(BUZZER_PIN);      // Turn the buzzer off
  tone(BUZZER_PIN, 1500);  // Play a 2 kHz tone (will sound the same for an active buzzer)
  delay(125);             
  noTone(BUZZER_PIN);      
  tone(BUZZER_PIN, 2000);  
  delay(125);             
  noTone(BUZZER_PIN);      
  delay(125);
}
void shutdown_beep(){
  tone(BUZZER_PIN, 2000);  
  delay(125);            
  noTone(BUZZER_PIN);      
  tone(BUZZER_PIN, 1500);  
  delay(125);             
  noTone(BUZZER_PIN);      
  tone(BUZZER_PIN, 1000);  
  delay(125);             
  noTone(BUZZER_PIN);      
  delay(125);
}

// ADC interrupt service routine
// EMPTY_INTERRUPT (ADC_vect);             // nothing to be done here

ISR(PCINT2_vect) {
  uint8_t a = PIND >> 3 & 1;  // Read the state of pin D3 (PCINT19)
  uint8_t b = PIND >> 2 & 1;  // Read the state of pin D2 (PCINT18)

  if (a != a0) {              // A changed
    a0 = a;
    if (b != b0) {            // B changed
      b0 = b;
      
      int increment = 0;
      unsigned long currentTime = micros();
      if (a == b) {
        // Counter-clockwise //encoder is wired weirdly
        increment = -countStep;
        if ((currentTime - _lastIncReadTime) < _pauseLength) {
          increment *= _fastIncrement;  //fast increment multiplier
        }
        _lastIncReadTime = currentTime;
      }
      else {
        // Clockwise
        increment = countStep;
        if ((currentTime - _lastDecReadTime) < _pauseLength) {
          increment *= _fastIncrement;
        }
        _lastDecReadTime = currentTime;
      }

      // Update count
      count = constrain(count + increment, countMin, countMax);

      // Check for rotary type and apply the same logic if needed
      if (ROTARY_TYPE && ((a == b) != ab0)) {
        count = constrain(count + increment, countMin, countMax);
      }

      ab0 = (a == b);
      handleMoved = true;
      IdleScreen  = false;
      systemoff   = false;
      if (state == shut_down){
        state = heating;
      }
    }
  }
}