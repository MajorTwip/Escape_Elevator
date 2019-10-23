/*  Example sketch for the PCF8574 for the purposes of showing how to use the interrupt-pin.

    Attach the positive lead of an LED to PIN7 on the PCF8574 and the negative lead to GND,
    a wire from GPIO2 (Nodemcu D4) to PIN3 that will be used to trigger the interrupt,
    and the INT-pin to GPIO14 (Nodemcu D5) on the ESP8266.

    If all goes well you should see the small blue LED on the ESP-module lighting up and the
    LED connected to the PCF going off, and vice versa. */

#include <pcf8574_esp.h>
#include <Wire.h>
#include <AccelStepper.h>

//Configurations

//Values 
#define LONGPRESSTIME 1000 

#define MOT_ELE_MAXSPEED 8000
#define MOT_ELE_ACCEL 1500
#define MOT_ELE_MAX 3800

//Pins onboard
#define MOT_ELE_DIR D0
#define MOT_ELE_STEP 13
#define MOT_MIR_OPEN 12
#define MOT_MIR_CLOSE 14

//Pins PCF8574
#define CODE_LEVEL 0
#define CODE_CABLE 1
#define ENDSWITCH 2
#define BTN_RED 3
#define BTN_GREEN 7
#define LOCKS 6

PCF857x pcf8574(0x20, &Wire);
AccelStepper mot_ele(AccelStepper::DRIVER,MOT_ELE_STEP,MOT_ELE_DIR);



//helper BUTTONS 
bool CL_isPushed(){
  return pcf8574.read(CODE_LEVEL);
}

bool CC_isPushed(){
  return pcf8574.read(CODE_CABLE);
}

bool ES_isPushed(){
  return !pcf8574.read(ENDSWITCH);
}

bool RED_isPushed(){
  return !pcf8574.read(BTN_RED);
}

long RED_pushedsince;
bool RED_isHeld(){
  if(RED_isPushed()){
    if(RED_pushedsince == 0) RED_pushedsince = millis();
    if(RED_pushedsince + LONGPRESSTIME < millis()) return true;
  }else{
    RED_pushedsince = 0;
  }
  return false;
}

bool GREEN_isPushed(){
  return !pcf8574.read(BTN_GREEN);
}

long GREEN_pushedsince;
bool GREEN_isHeld(){
  if(GREEN_isPushed()){
    if(GREEN_pushedsince == 0) GREEN_pushedsince = millis();
    if(GREEN_pushedsince + LONGPRESSTIME < millis()) return true;
  }else{
    GREEN_pushedsince = 0;
  }
  return false;
}

void closeMir(){
  digitalWrite(MOT_MIR_OPEN,LOW);
  delay(100);
  digitalWrite(MOT_MIR_CLOSE,HIGH);
}


void openMir(){  
  digitalWrite(MOT_MIR_CLOSE,LOW);
  delay(100);
  digitalWrite(MOT_MIR_OPEN,HIGH);
}

//list of states
enum state {INIT,
            INIT_ELE,
            END_INIT_ELE,
            INIT_MIR,
            STOP,
            RESET,
            READY,
            STARTING,
            PLAY_NOSOLVED,
            PLAY_CCSOLVED_OPENING,
            PLAY_CCSOLVED,
            PLAY_LCSOLVED_OPENING,
            PLAY_LCSOLVED
            };

enum state actualState = INIT;

bool firstRun = true;
int initial_homing=0;

void prog(){
  switch(actualState){
    case INIT:
      if(firstRun){
        Serial.println("Begin with initialization");
        firstRun = false;
      }
      actualState=INIT_ELE; firstRun = true;
      break;

      
    case INIT_ELE:
      if(firstRun){
        Serial.println("Begin with zeroing ElevatorDoor");
        firstRun=false;

        //  Set Max Speed and Acceleration of each Steppers at startup for homing
        mot_ele.setMaxSpeed(1000.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
        mot_ele.setAcceleration(400.0);  // Set Acceleration of Stepper
      }

      // Start Homing procedure of Stepper Motor at startup


      mot_ele.moveTo(initial_homing);  // Set the position to move to
      initial_homing--;  // Decrease by 1 for next move if needed
      delay(5);

      
      if(ES_isPushed()||RED_isPushed()){
        Serial.println("ElevatorDoor reached zero");
        mot_ele.setCurrentPosition(0);
        mot_ele.setMaxSpeed(MOT_ELE_MAXSPEED);
        mot_ele.setAcceleration(MOT_ELE_ACCEL);
        initial_homing=0;
        actualState=END_INIT_ELE;
        firstRun = true;
      }
      break;


      
    case END_INIT_ELE:
      if(firstRun){
        Serial.println("Opening Elevator");
        firstRun = false;
        mot_ele.moveTo(MOT_ELE_MAX);
      }
      if(mot_ele.distanceToGo()==0){
        Serial.println("Elevator opened");
        actualState = INIT_MIR;
        firstRun=true;
      }
      break;

      
    case INIT_MIR:
      openMir();
      delay(200);
      actualState = STOP;
      break;

    case STOP:
      if(firstRun){
        Serial.println("Stopping sequence startet, opening all doors");
        firstRun = false;
        openMir();
      }
      mot_ele.moveTo(MOT_ELE_MAX);
      if(GREEN_isPushed()){
        actualState = RESET;
        firstRun = true;
      }
      break;

    case RESET:
            closeMir();
      if(firstRun){
        Serial.println("Resetting Game");
        firstRun = false;
        mot_ele.moveTo(MOT_ELE_MAX);
        closeMir();
        delay(2000);
      }
      if(RED_isPushed()){
         mot_ele.stop();
         actualState = STOP;
         firstRun=true;
      }
        actualState = READY;
      break;
     
    case READY:
      if(firstRun){
        Serial.println("Game ready");
        firstRun = false;
      }
      if(GREEN_isPushed()){
        actualState = STARTING;
        firstRun = true;
      }
      if(RED_isPushed()){
        mot_ele.stop();
        actualState = STOP;
        firstRun=true;
      }
      break;

    case STARTING:
      if(firstRun){
        Serial.println("Game starting, closing door");
        firstRun = false;
        mot_ele.moveTo(0);
        initial_homing=0;
      }
      if(mot_ele.distanceToGo()==0){
        if(ES_isPushed() || GREEN_isPushed()){
          actualState = PLAY_NOSOLVED;
          firstRun = true;
          mot_ele.setCurrentPosition(0);
          delay(2000);
        }
        else{
          mot_ele.moveTo(initial_homing);
          initial_homing--;
          delay(5);
        }
      }
      
      if(RED_isPushed()){
        mot_ele.stop();
        actualState = STOP;
        firstRun=true;
      }
      break;

    case PLAY_NOSOLVED:
      if(firstRun){
        Serial.println("Game running");
        firstRun = false;
      }
      if(GREEN_isPushed() || CC_isPushed()){
        actualState = PLAY_CCSOLVED_OPENING;
        firstRun = true;
      }
      if(RED_isPushed()){
        mot_ele.stop();
        actualState = STOP;
        firstRun=true;
      }
      break;

    case PLAY_CCSOLVED_OPENING:
        Serial.println("Cables solved, opening mirror");
        firstRun = false;
        openMir();
        delay(2000);
      
        actualState = PLAY_CCSOLVED;

      break;

    case PLAY_CCSOLVED:
      if(firstRun){
        Serial.println("Cables soved, mirror opened");
        firstRun = false;
      }
      if(GREEN_isPushed()||CL_isPushed()){
        actualState = PLAY_LCSOLVED_OPENING;
        firstRun = true;
      }
      if(RED_isPushed()){
        mot_ele.stop();
        actualState = STOP;
        firstRun=true;
      }
      break;

    case PLAY_LCSOLVED_OPENING:
      if(firstRun){
        Serial.println("Levelcode Solved, game finished, opening door");
        firstRun = false;
        mot_ele.moveTo(MOT_ELE_MAX);
      }
      if(mot_ele.distanceToGo()==0){
        actualState = STOP;
        firstRun = true;
      }
      if(RED_isPushed()){
        mot_ele.stop();
        actualState = STOP;
        firstRun=true;
      }
      break;
    default:
      break;
  }
}
            


void setup() {
  Serial.begin(9600);
  Serial.println("Firing up...");

  Wire.begin();
  pcf8574.begin();

  pinMode(MOT_MIR_OPEN,OUTPUT);
  pinMode(MOT_MIR_CLOSE,OUTPUT);
  digitalWrite(MOT_MIR_OPEN,LOW);
  digitalWrite(MOT_MIR_CLOSE,LOW);
}

void loop() {
  prog();
  mot_ele.run();
  


}



