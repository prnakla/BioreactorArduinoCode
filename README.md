# BioreactorArduinoCode
//Code for electromechanical bioreactor stimulation 

// GUI
// #include <AFMotor.h>
#include "Nextion.h"

// #define DHTPIN 4?? define pins for mechanical and electrical components? how

// Initialize sensors or motors
// take rx and tx digital pins out before uploading arduino code

// DECLARE NEXTION OBJECTS - Example (page id = 0, component id = 1, component name = 'b0')
  NexSlider h0 = NexSlider(1, 1, "h0");
  NexSlider h1 = NexSlider(1, 2, "h1");
  NexSlider h2 = NexSlider(1, 3, "h2");
  NexSlider h3 = NexSlider(1, 4, "h3");
  NexSlider h4 = NexSlider(1, 5, "h4");
  NexSlider h5 = NexSlider(1, 1, "h5");
  NexSlider h6 = NexSlider(1, 2, "h6");
  NexSlider h7 = NexSlider(1, 3, "h7");
  NexSlider h8 = NexSlider(1, 4, "h8");
  NexSlider h9 = NexSlider(1, 5, "h9");

NexText t7 = NexText(1, 14, "t7");
NexText t8 = NexText(1, 15, "t8");
NexText t9 = NexText(1, 16, "t9");
NexText t10 = NexText(1, 17, "t10");
NexText t11 = NexText(1, 18, "t11");
NexText t12 = NexText(1, 14, "t12");
NexText t13 = NexText(1, 15, "t13");
NexText t14 = NexText(1, 16, "t14");
NexText t15 = NexText(1, 17, "t15");
NexText t16 = NexText(1, 18, "t16");
//NexButton electrial = NexButton(0, 1, "b0");
//NexButton mechanical = NexButton(0, 2, "b1");
//NexButton bstart = NexButton(0, 5, "b2");
//NexButton bstop = NexButton(0, 6, "b3");

NexButton b0 = NexButton(0, 1, "b0");
NexButton b1 = NexButton(0, 2, "b1");
NexButton b2 = NexButton(0, 5, "b2");
NexButton b3 = NexButton(0, 6, "b3");

// Mention once for each page, copy pasted, check if that's an issue when trouble shooting
NexButton b4 = NexButton (1, 13, "b4");
//NexButton menu = NexButton (2, 13, "b4");
//NexButton menu = NexButton (3, 2, "b4");
//NexButton menu = NexButton (4, 2, "b4");


// Register a button object to the touch event list.  
NexTouch *nex_listen_list[] = {
  &b0,
  &b1,
  &b2,
  &b3,
  &b4,
  &h0,
  &h1,
  &h2,
  &h3,
  &h4,
  &h5,
  &h6,
  &h7,
  &h8,
  &h9,
  NULL
};


// CALLBACK FUNCTIONS

void b0PopCallback(void *ptr) {
  //go to page 1 electrical
}

void b1PopCallback(void *ptr) {
  //go to page 2 mechanical
}

void b2PopCallback(void *ptr) {
  //register the time at press of button (since when the arduino code was compiled) 
  unsigned long startTime = millis();
  
  //go to page 3 start
  //code to pull values from page 1 and 2, and start the motors + voltage pulses accordingly
}

void b3PopCallback(void *ptr) {
  //go to page 4 stop
  //code to terminate all functions, motors, voltage, etc. 
}

void b4PopCallback(void *ptr) {
  //go to page 0 menu 
}

// code for numbers changing with slider- make sure limits are correct
// any other code for the pins?? idts... 
// When slider is released, slider text changes (can i make this so it changes when touched?)

void h0PopCallback(void *ptr) {
  uint32_t number = 0;
  char a[10] = {0};
  // change text with the current slider value
  h0.getValue(&number);
  utoa(number, a, 10);
  t7.setText(a);
  // change slider value accordingly
  //example: change LED brightness with "analogWrite(led2, number);"
}

void h1PopCallback(void *ptr) {
  uint32_t number = 0;
  char b[10] = {0};
  h1.getValue(&number);
  utoa(number, b, 10);
  t8.setText(b);
}

void h2PopCallback(void *ptr) {
  uint32_t number = 0;
  char c[10] = {0};
  h2.getValue(&number);
  utoa(number, c, 10);
  t9.setText(c);
}

void h3PopCallback(void *ptr) {
  uint32_t number = 0;
  char d[10] = {0};
  h3.getValue(&number);
  utoa(number, d, 10);
  t10.setText(d);
}

void h4PopCallback(void *ptr) {
  uint32_t number = 0;
  char e[10] = {0};
  h4.getValue(&number);
  utoa(number, e, 10);
  t11.setText(e);
}

void h5PopCallback(void *ptr) {
  uint32_t number = 0;
  char f[10] = {0};
  h5.getValue(&number);
  utoa(number, f, 10);
  t12.setText(f);
}

void h6PopCallback(void *ptr) {
  uint32_t number = 0;
  char g[10] = {0};
  h6.getValue(&number);
  utoa(number, g, 10);
  t13.setText(g);
}

void h7PopCallback(void *ptr) {
  uint32_t number = 0;
  char h[10] = {0};
  h7.getValue(&number);
  utoa(number, h, 10);
  t14.setText(h);
}

void h8PopCallback(void *ptr) {
  uint32_t number = 0;
  char i[10] = {0};
  h8.getValue(&number);
  utoa(number, i, 10);
  t15.setText(i);
}

void h9PopCallback(void *ptr) {
  uint32_t number = 0;
  char j[10] = {0};
  h9.getValue(&number);
  utoa(number, j, 10);
  t16.setText(j);
}

#include <Adafruit_MotorShield.h>
#include <math.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2), and #2 (M3 and M4)
Adafruit_StepperMotor *motor1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *motor2 = AFMS.getStepper(200, 2);

// Initialize Time Variables
  unsigned long hrs = 0;
  unsigned long mins = 0;
  unsigned long secs = 0;
  unsigned long ms = 0;
  unsigned long numSecsPerMin = 0;
  unsigned long numMinsPerHr = 0;
  unsigned long Hrs = 0;
  
// Initialize Electrical Stimulation Variables
  int elecStimState = LOW;
  unsigned long previousMillisElec = 0;
  int potMotorIter = 0;
  bool potMotor_True = false;
  double refVoltage = 1.25;
  double voltage = 0;
  int redElectrodePos = HIGH;
  int elec_Hz = 0;
  unsigned long elecRuntime = 0;
  unsigned long elecDuration = 0;
  unsigned long elecTrain = 0;
  unsigned long buckHalfPeriod = 0;
  int minOhm = 500;
  bool elecStim_True =false; 
 
// Initialize Mechanical Stimulation Variables
  int mechStimState = LOW;
  unsigned long previousMillisMech = 0;
  unsigned long mech_HalfPeriod = 0;
  int numSteps = 0;
  int strain = 0;
  unsigned long mechRuntime = 0;
  unsigned long mechDuration = 0;
  unsigned long mechTrain = 0;
  double mech_Hz = 0;
  bool mechStim_True = false;
  
// Initialize Electrical Stimulation PWM Output Variables
  int pin_6 = 6;
  int pin_8 = 8;
  int buck_pin = 10;
  int pin_12 = 12;
  int pin_13 = 13;

// Function declarations
  bool setPotMotorTrue(double voltage, int potMotorIter);
  void potMotor(double voltage);
  bool setElecStim_True(double voltage, int elec_Hz, unsigned long elecRuntime, unsigned long hrs, unsigned long elecDuration, unsigned long mins, unsigned long elecTrain, unsigned long secs);
  unsigned long checkElapsed_Time(unsigned long ms, unsigned long previousMillis);
  int elecStimStart(int redElectrodePos);
  void elecStimStop();
  bool setMechStim_True(int strain, double mech_Hz, unsigned long mechRuntime, unsigned long hrs, unsigned long mechDuration, unsigned long mins, unsigned long mechTrain, unsigned long secs);
  void mechStimStart(int numSteps);
  void mechStimStop(int numSteps); 
  unsigned long numHrs(unsigned long ms);
  unsigned long numMins(unsigned long ms);
  unsigned long numSecs(unsigned long ms);
unsigned long checkElapsedTimeElec(unsigned long ms, unsigned long previousMillisElec, unsigned long buckHalfPeriod);
unsigned long checkElapsedTimeMech(unsigned long ms, unsigned long previousMillisMech, unsigned long mech_HalfPeriod);

/////// End initial state parameters

void setup() {

  // You might need to change NexConfig.h file in your ITEADLIB_Arduino_Nextion folder
  // Set the baudrate which is for debug and communicate with Nextion screen
  nexInit();

  // Register the pop event callback function of the components
  b0.attachPop(b0PopCallback);
  b1.attachPop(b1PopCallback);
  b2.attachPop(b2PopCallback);
  b3.attachPop(b3PopCallback);
  b4.attachPop(b4PopCallback);
  h0.attachPop(h0PopCallback);
  h1.attachPop(h1PopCallback);
  h2.attachPop(h2PopCallback);
  h3.attachPop(h3PopCallback);
  h4.attachPop(h4PopCallback);
  h5.attachPop(h5PopCallback);
  h6.attachPop(h6PopCallback);
  h7.attachPop(h7PopCallback);
  h8.attachPop(h8PopCallback);
  h9.attachPop(h9PopCallback);
  
  Serial.begin(9600);

  while (!Serial);
  Serial.println("Stepper test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  motor1->setSpeed(10);  // 10 rpm (test)
  motor2->setSpeed(600);  // 10 rpm

  pinMode(pin_6, OUTPUT);
  pinMode(pin_8, OUTPUT);
  pinMode(buck_pin, OUTPUT);
  pinMode(pin_12, OUTPUT);
  pinMode(pin_13, OUTPUT);

  ///////Set up GUI here *******************
}
// End setup()
/////////////////////////


void loop() {

  /*
   * When a pop or push event occured every time,
   * the corresponding component[right page id and component id] in touch event list will be asked.
   */
  nexLoop(nex_listen_list);

  ms = startTime;
  elecDuration = h0;
  elecTrain = h1;
  elec_Hz = h2;
  voltage = h3;
  elecRuntime = h4;
  mechDuration = h5;
  mechTrain = h6;
  mech_Hz = h7;
  strain = h8;
  mechRuntime = h9;

    // Keep track of elapsed time....this is harder than it appears. If the GUI is running, time is elapsing. We should probably make the press of Start Button
    // as time zero, and base everything else off of that.

    unsigned long ms = millis(); // <<---- Make this equal the start button time in millis()
    unsigned long hrs = numHrs(ms);  // Then this would be (millis() - (start button time))/3600
    unsigned long mins = numMins(ms);  // Etc. 
    unsigned long secs = numSecs(ms); // Etc. 

    ////CALL GUI function and RETURN GUI parameters here *****************
    /// We need double voltage, int strain, double mech_Hz, int elec_Hz, int mechRuntime, int elecRuntime, int mechDuration, int elecDuration, int mechTrain, int elecTrain
  
    // Check to see if voltage stimulation is set then turn potentiometer to reach desired voltage

    bool potMotorTrue = setPotMotorTrue(voltage, potMotorIter);
  
    if (voltage > refVoltage && potMotorTrue) {
      potMotor(voltage);
      potMotorIter++;
    }

   // Electrical Stimulation Loop

   // First we need to see if electrical stimulation is desired. As long as 1.25 < voltage <= 50 and 0 < elec_Hz <= 5  and start button pushed
  // There will be at least 1 sec of stimulation up to elecTrain, elecDuration, and elecRunTime limits. These times are measure from the
  // start of the program.
  
    bool elecStim_True = setElecStim_True(voltage, elec_Hz, elecRuntime, hrs, elecDuration, mins, elecTrain, secs);  

    if (elecStim_True) {
       unsigned long buckHalfPeriod = 500/elec_Hz;
       unsigned long previousMillisElec = checkElapsedTimeElec(ms, previousMillisElec, buckHalfPeriod);
       
       if (previousMillisElec >= buckHalfPeriod && elecStimState == LOW) {
            elecStimStart(redElectrodePos);
            elecStimState = HIGH;
       }
       else {
            elecStimState = LOW;
            elecStimStop();
       }
  }
  
  

              
  // Mechanical Stimulation Loop
  //
  //
  //
  // First we need to see if mechanical stimulation is desired. As long as 0 < strain <= 20 and 0 < mech_Hz <= 5  and start button pushed
  // There will be at least 1 sec of stimulation up to mechTrain, mechDuration, and mechRunTime limits. These times are measure from the
  // start of the program.

  
    bool mechStim_True = setMechStim_True(strain, mech_Hz, mechRuntime, hrs, mechDuration, mins, mechTrain, secs);

    if (mechStim_True) {
      mech_HalfPeriod = 500/mech_Hz;                                              
      unsigned long previousMilisMech = checkElapsedTimeMech(ms, previousMillisMech, mech_HalfPeriod);                                          
      int numSteps = strain; //10% strain = 1mm. The stepper motor has 200 steps per revolution and a 2mm pitch on the screw.
                                                       //In other words, strain resolution is about 0.1%, but can be adjusted more or less yet, if necessary 
    
     if (previousMillisMech >= mech_HalfPeriod && mechStimState == LOW) {
            mechStimStart(numSteps);
            mechStimState = HIGH;
     }
     else {
          mechStimStop(numSteps);
          mechStimState = LOW;
     }    
  }
} 
//end void loop()
///////////////////
///////////////////


//////////// Functions follow


// These functions keep track of runtime in hours:mins:secs

unsigned long numHrs(unsigned long ms) {
  unsigned long Hrs = ((millis()-ms)/1000)/3600;
  return Hrs;
}

unsigned long numMins(unsigned long ms) {
  unsigned long numMinsTotal = ((millis()-ms)/1000)/60;
  
  while (numMinsTotal >= 60) {
    numMinsTotal = numMinsTotal - 60;
  }
    return numMinsTotal;
}

unsigned long numSecs(unsigned long ms) {
unsigned long numSecsTotal = (millis()-ms)/1000;
  
  while (numSecsTotal >= 60) {
    numSecsTotal = numSecsTotal - 60;
  }
    return numSecsTotal;
}


// Electrical Stimulation Functions Follow
//
//
//


//T his function determines whether the voltage pot needs to be set and whether it has already
// been set
bool setPotMotorTrue(double voltage, int potMotorIter) {
  if (voltage > 1.25 && potMotorIter < 1) {
    return true;
  }
  else {
    return false;
  }
  
}


// This function calculates the necessary resistance value of the potentiometer to produce the desired stimulation voltage,
// then determines the number of corresponding steps to turn the potentiometer to reach that resistance
// and finally moves the motor coupled to the potentiometer the number of corresponding steps
//
void potMotor(double voltage) {
  
  float R2 = 22000*((voltage/1.25) - 1);
  float numStepsInit = (R2 - minOhm)/312;
  int numStepsPot = round(numStepsInit);

  motor1->step(numStepsPot, FORWARD, SINGLE); 
  motor1->release();
}

// This function checks to see whether electrical stimulation is desired and helps to keep track of the various time parameters associated with it
//
bool setElecStim_True(double voltage, int elec_Hz, unsigned long elecRuntime, unsigned long hrs, unsigned long elecDuration, unsigned long mins, unsigned long elecTrain, unsigned long secs) {

  if (voltage > 1.25 && elec_Hz > 0 && hrs < elecRuntime && mins < elecDuration && secs < elecTrain) {
    return true;
  }
  else {
    return false;
  }
}

// This function starts the electrical stimulation and changes the finite state machine as to which electrode
// Lead is hot. Changing polarity like this helps to minimize corrosion of electrodes in media bath.
//
int elecStimStart(int redElectrodePos) {

  // This switches electrode polarity so as to mitigate corrosion
  if (redElectrodePos = HIGH) {
    redElectrodePos = LOW;
    analogWrite(buck_pin, 255); //This turns the DC-DC Buck Converter On
    analogWrite(pin_12, 255);  //This turns the corresponding switches on to activate the red electrode as positive voltage
    analogWrite(pin_6, 255);  //This turns the corresponding switches on to activate the red electrode as positive voltage
    analogWrite(pin_8, 0);  //This turns the corresponding switches off to activate the black electrode as ground
    analogWrite(pin_13, 0); //This turns the corresponding switches off to activate the black electrode as ground
  }

  else {
    redElectrodePos = HIGH;
    analogWrite(buck_pin, 255); //This turns the DC-DC Buck Converter On
    analogWrite(pin_12, 0); //This turns the corresponding switches on to activate the red electrode as ground
    analogWrite(pin_6, 0); //This turns the corresponding switches on to activate the red electrode as ground
    analogWrite(pin_8, 255); //This turns the corresponding switches off to activate the black electrode as positive voltage
    analogWrite(pin_13, 255); //This turns the corresponding switches off to activate the black electrode as positive voltage
  }   
}


// This function simply stops electrical stimulation, permanently or temporarily depending on the setElecStim_True function above
//
void elecStimStop() {
  analogWrite(buck_pin, 0); //This turns the DC-DC Buck Converter Off. If it is off, it won't provide voltage out
  analogWrite(pin_8, 0);  //This sets the black electrode as ground...ie, off
  analogWrite(pin_12,0);  //This sets the red electrode as ground...ie, off
}


// This function determines the amount of time elapsed during an electrical stimulation event.
// That is, it checks to see if the current time is less than the half period of the desired electrical stimulation
// And returns it so that other code can determine whether to turn on or off the stimulation.
//
unsigned long checkElapsedTimeElec(unsigned long ms, unsigned long previousMillisElec, unsigned long buckHalfPeriod) {

  if (ms - previousMillisElec >= buckHalfPeriod) {
      previousMillisElec = ms;
      return previousMillisElec;  
  }
}

/// Mechanical Stimulation Functions Follow
////
///


// Like its corresponding electrical stimulation function above, this function determines whether mechanical stimulation is desired
//
bool setMechStim_True(int strain, double mech_Hz, unsigned long mechRuntime, unsigned long hrs, unsigned long mechDuration, unsigned long mins, unsigned long mechTrain, unsigned long secs) {

  if (strain > 0 && mech_Hz > 0 && hrs < mechRuntime && mins < mechDuration && secs < mechTrain) {
    return true;
  }
  else {
    return false;
  }
}

// This function determines how long it's been since stimulation has started or stopped, to determine how long
// the mechanical stimulation should last based upon input parameters
//
unsigned long checkElapsedTimeMech(unsigned long ms, unsigned long previousMillisMech, unsigned long mech_HalfPeriod) {

  if (ms - previousMillisMech >= mech_HalfPeriod) {
      previousMillisMech = ms;
      return previousMillisMech;  
  }
}

// This function moves the motor to stretch the tissue
//
void mechStimStart(int numSteps) {
  motor2->step(numSteps, FORWARD, DOUBLE);
  motor2->release();
}

// This function moves the motor to relax stretched tissue
//
void mechStimStop(int numSteps) {
  motor2->step(numSteps, BACKWARD, DOUBLE);
  motor2->release();
}

