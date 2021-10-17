
#include <EEPROM.h>  

int pinI1=5;      // PIN D5 MOTOR UP
int pinI2=7;     // PIN D7 MOTOR DOWN
int speedpinA=6;  // PIN D6 PWM

//const int relay_pin=4;

int pwm=150;
volatile int steps = 0;   // Pulses from  Hall Effect sensors


const int pinH1=2;      // PIN D2 HALL sensor signal

const int remote_input = 3;     // PIN D3 Remote signal 
const int init_button = 4;      // PIN D4 Door init button
const int LONG_PRESS_TIME = 2000;

// Variables will change:
int lastState = HIGH;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
bool isPressing = false;
bool isLongDetected = false;

int RemotebuttonState = 0; 
//int InitbuttonState = 0;  

int door_status = 0;          //Door open/closed position 0-closed 1-open 2-interrupted
int door_position = 0;
int prev_door_pos = 0;
bool active = 0;              //Active command check
int door_open_pos = 0;      //Hall memory of open position
const int door_closed_pos = 0;      //Closed position default
int last_dir = 0;             // Last direction of movement 0-up 1-down  

  
volatile unsigned long  prevTimer = 0;

bool initialized = 0;
bool alarm = 0;
int alarm_count=0;


void setup() {
  // put your setup code here, to run once:
  pinMode(pinI1,OUTPUT); 
  pinMode(pinI2,OUTPUT); 
  pinMode(speedpinA,OUTPUT); 
  
  digitalWrite(pinI1,HIGH); 
  digitalWrite(pinI2,HIGH);

  delay(2000);


//  pinMode(relay_pin,OUTPUT);
 
 

  Serial.begin(9600);

  pinMode(remote_input,INPUT);
  pinMode(init_button,INPUT_PULLUP);

  pinMode(pinH1,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinH1), countSteps, CHANGE); // HALL sensor interrupt init


//EEPROM.write(0, 100);
door_open_pos = EEPROM.read(0);
// door_closed_pos = EEPROM.read(1);
  


/* INIT SEQUENCE */

Serial.print("Door open position set: ");
Serial.println(door_open_pos);
Serial.print("Door closed position set: ");
Serial.println(door_closed_pos);
Serial.print("Door initialized: ");
Serial.println(initialized);


delay(1000);
//digitalWrite(relay_pin, HIGH);

  
  }

void loop() {

  
  RemotebuttonState = digitalRead(remote_input);                // wait for remote signal or button press
  currentState = digitalRead(init_button);                // wait for remote signal or button press

  
  if (RemotebuttonState == HIGH && initialized==1)              // if remote signal goes HIGH & door is initialized
  {
  Serial.println("Remote sigal");
  Serial.println (active);
  
    if (active == 0)                                      // if NO command is active
      { 
      switch (door_status)                         
        {
        case 0:                                           // case  door stautus is closed
        Serial.println("Door opening");  
        active=1;                                         // set active flag
        last_dir = 0;                                     // Set last dir of movement UP
        open();                                           // open COMMAND
        break;
        
        case 1:                                           // case  door stautus is open
        Serial.println("Door closing");  
        active=1;  
        last_dir = 1;                                     
        close();                                       
        break;

        case 2:                                           // case  door stautus is stopped inbetween
        Serial.println("Door resume");  
        active=1;  
        if (last_dir == 1){                                     
          
          close(); 
        }
        else{
          
          open(); 
        }
        
        break;
        }      
      }
   }
    
  else if (currentState == LOW && initialized == 0){    // Init button long press routine
          
          Serial.println("Door initialization");
          active=1;
          initialize_door();
  }        
          
          
          
          
//          Serial.println("A press is detected");
//
//    pressedTime = millis();
//    isPressing = true;
//    isLongDetected = false;
//    }
//  else if (currentState == HIGH){ 
//    isPressing = false;
//    }
//
//  
//  if(isPressing == true && isLongDetected == false) {
//    long pressDuration = millis() - pressedTime;
//
//    if( pressDuration > LONG_PRESS_TIME ) {
//      Serial.println("A long press is detected");
//      isLongDetected = true;
//      
      

      
    

  
}
    


void open()       // Drive door UP
{
  prevTimer = millis();
  door_status=2;
  analogWrite(speedpinA,pwm); 
  digitalWrite(pinI2,LOW); 
  digitalWrite(pinI1,HIGH);
  
  while(active == 1){
  
   if(millis() - prevTimer > 100){                    // Update the Position every 1/10 seconds
        
        updatePosition();
        prevTimer = millis();
      
       
        if (door_position >= door_open_pos){
          stop();
          //Serial.println("Door opened");
          //Serial.print ("Door Position UP: ");
          //Serial.println (door_position);

          door_status=1;
          active = 0;

          }

        if (door_position >= door_open_pos-300){
          analogWrite(speedpinA,30);
          }
       
        if (door_position == prev_door_pos){
          stop();
          alarm=1;
          Serial.println ("ALARM");
          active = 0;

        }  
    
   }
 }
}



void close()    // Drive door DOWN

{
  prevTimer = millis();
  door_status=2;
  analogWrite(speedpinA,pwm);  
  digitalWrite(pinI2,HIGH); 
  digitalWrite(pinI1,LOW);
  
  while(active == 1){



  if(millis() - prevTimer > 100){                   
        updatePosition();                            
        prevTimer = millis();
        
        if (door_position <= door_closed_pos){
          stop();
          //Serial.println("Door closed");
          //Serial.print ("Door Position DOWN: ");
          //Serial.println (door_position);
          door_status=0;
          active = 0;
          }
        if (door_position <= door_closed_pos+300){
          analogWrite(speedpinA,50);
          }
        if (door_position == prev_door_pos){
          stop();
          alarm=1;
          Serial.println ("ALARM");
          active = 0;
        }
  }
 }
}
 
 void stop()
{
  analogWrite(speedpinA,0);
  digitalWrite(pinI2,HIGH); 
  digitalWrite(pinI1,HIGH);
  
} 

void countSteps() 
{
  steps++;
  
}


void updatePosition()
{
  if(last_dir == 0){
    //Serial.print ("door position: ");
    //Serial.println (door_position);
    prev_door_pos = door_position;
    door_position = door_position + steps;
    steps = 0;
  } else {
    //Serial.print ("door position: ");
    //Serial.println (door_position);
    prev_door_pos = door_position;
    door_position = door_position - steps;
    steps = 0;
  }}


void initialize_door()          // Door initialization
{
  prevTimer = millis();
  door_status=2;
  last_dir=1;
  door_position = 10000;

  
  while(active == 1){
  analogWrite(speedpinA,50);  
  digitalWrite(pinI2,HIGH); 
  digitalWrite(pinI1,LOW);


  if(millis() - prevTimer > 80){                   
        updatePosition();                            
        prevTimer = millis();
        
        if (door_position == prev_door_pos){
          stop();
          Serial.println ("DETECTED CLOSED");
          active = 0;
          door_position = 0;
        }
  }
 }
  delay(3000);
  active = 1;
  last_dir=0;
  prevTimer = millis();
  
  while(active == 1){
  analogWrite(speedpinA,50);  
  digitalWrite(pinI2,LOW); 
  digitalWrite(pinI1,HIGH);


  if(millis() - prevTimer > 80){                   
        updatePosition();                            
        prevTimer = millis();
        
        if (door_position == prev_door_pos){
          stop();
          Serial.println ("DETECTED OPEN");
          active = 0;
          door_position=door_position-100;      //Compensate for belt slack
          door_open_pos = door_position;
          EEPROM.write(0, door_position);
          //Serial.print("Door open position detected: ");
          //Serial.println(door_position);
        }
  }
 }

  delay(3000);
  active = 1;
  last_dir=1; 
  close();

switch (alarm) {
 
case 0:
Serial.println ("Init sucessfull");
Serial.print("Door open position set: ");
Serial.println(door_open_pos);
initialized=1;
active=0;
break;

case 1:
Serial.println ("Init failed");
initialized=0;

break;



}
  
  
}
