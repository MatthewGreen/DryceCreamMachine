/*
Alan Jenson, Yu Li & Matthew Green
INFO 4320 - Final Project
Dryce Cream Machine
Due Date: May  11 2013

Some Code by Dr. Francois Guimbretiere, Tom Igone and from Bildr.org (http://bildr.org/2011/06/easydriver/)
Modified May 2013
by Alan Jenson, Yu Li & Matthew Green (Cornell University)

*/

#include <Servo.h> 

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%START OF VARIABLE DECLARATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//################Generic Variables#################
const int ledPin = 13;
//################EOL Generic Variables##############

//################Serial Communication Variables##############
//Slot A
int van_A;
int choc_A;
int top_A;

//Slot B
int van_B;
int choc_B;
int top_B;

//Slot C
int van_C;
int choc_C;
int top_C;

//Slot D
int van_D;
int choc_D;
int top_D;

//Slot E
int van_E;
int choc_E;
int top_E;

//Slot Flags
boolean a_Flag;
boolean b_Flag;
boolean c_Flag;
boolean d_Flag;
boolean e_Flag;
//################EOL Serial Communication Variables##############

//################Reed Switches Pins & Variables##################
//Turntable Motor
const int turntable_motorPin = 11; //PWM
const int turntable_power = 57; //Used for the PWM to the power the motor
//Reed Switch Pins
const int reedA = A0; //Using Analog Pins But Are Digital
const int reedB = A1;
const int reedC = A2;
const int reedD = A3;
const int reedE = A4;
// State of the LED
boolean LED_on = false;
// State of the button
byte switch_reedA = LOW;
byte switch_reedB = LOW;
byte switch_reedC = LOW;
byte switch_reedD = LOW;
byte switch_reedE = LOW;
// Previous state of the button
byte old_reedA = LOW;
byte old_reedB = LOW;
byte old_reedC = LOW;
byte old_reedD = LOW;
byte old_reedE = LOW;
//################EOL Reed Switches Pins & Variables##############

//################Ice Dispenser Pins & Variables##################
const int ice_dirPin = 4;
const int ice_stepPin = 5; //PWM
const int ice_degrees = 1110;
//################EOL Ice Dispenser Pins & Variables##############

//################Topping Dispenser Pins & Variables##################
const int topping_dirPin = 8;
const int topping_stepPin = 3; //PWM
const int topping_degrees = 180;
const int topping_inverse_degrees = -180;
//################EOL Topping Dispenser Pins & Variables##############

//################Stirring Mechanism Pins & Variables##################
const int stir_dirPin = 7;
const int stir_stepPin = 6; //PWM
const int stir_motorPin = 9;
const int stir_degrees = 1500;
const int stir_inverse_degrees = -1500;
//################EOL Stirring Mechanism Pins & Variables##############

//################Liquid Dispenser Mechanism Pins & Variables##################
const int servo_vanillaPin = 10;
const int servo_chocoPin = 12;
const int servo_value_angle = 180;
const int servo_value_angleClose = 0;
Servo vanillaServo;
Servo chocolateServo;
//################EOL Liquid Dispenser Mechanism Pins & Variables##################

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%END OF VARIABLE DECLARATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

int lastSensor = 0;
int delayTime = 3000;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%START OF FUNCTION DECLARATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//~~~This function is used tp read from the C# GUI to know what stations need to be turned on or off.
//The input comes in as a series of 1,0,1 etc of which there are 15 ending with a \n.
boolean readFromGUI(){
  boolean _read = false;
 if (Serial.available() > 0){
    //Read The Serial Port And Parse The Values To The Holding Variables
    van_A = Serial.parseInt();
    choc_A = Serial.parseInt();
    top_A = Serial.parseInt();
    van_B = Serial.parseInt();
    choc_B = Serial.parseInt();
    top_B = Serial.parseInt();
    van_C = Serial.parseInt();
    choc_C = Serial.parseInt();
    top_C = Serial.parseInt();
    van_D = Serial.parseInt();
    choc_D = Serial.parseInt();
    top_D = Serial.parseInt();
    van_E = Serial.parseInt();
    choc_E = Serial.parseInt();
    top_E = Serial.parseInt();
    
    //Set flags to know whether a slot is on or not
    if(van_A == 1 || choc_A == 1 || top_A == 1){a_Flag = true;}
    if(van_B == 1 || choc_B == 1 || top_B == 1){b_Flag = true;}
    if(van_C == 1 || choc_C == 1 || top_C == 1){c_Flag = true;}
    if(van_D == 1 || choc_D == 1 || top_D == 1){d_Flag = true;}
    if(van_E == 1 || choc_E == 1 || top_E == 1){e_Flag = true;}
    
    //Check For The End Of The Line & Acknowledge By Blinking The LED
//    if (Serial.read() == '\n'){
//      digitalWrite(ledPin, HIGH);
//      delay(500);
//      digitalWrite(ledPin, LOW);
//      delay(500);
//      digitalWrite(ledPin, HIGH);
//      delay(500);
//      digitalWrite(ledPin, LOW);
//    }
    _read = true;
  }
  return _read;
}

//~~~This Function Is Used At The Very End To Let The GUI Know That The Operations Have Been Completed
void writeToGUI(){
  //Set flags to false
  a_Flag = false;
  b_Flag = false;
  c_Flag = false;
  d_Flag = false;
  e_Flag = false;
  
  while (Serial.available() > 0){
    Serial.println(van_A);
    Serial.println(choc_A);
    Serial.println(top_A);
    Serial.flush();
  }
}

//~~~From Bildr.org (http://bildr.org/2011/06/easydriver/) for use with the Stepper motors for the stirring mechanism and the ice and topping dispensers.
void rotateDispDeg(float deg, float speed, int dir_pin, int step_pin){ 
  //step_pin must be a PWM pin
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg > 0)? HIGH:LOW;
  digitalWrite(dir_pin,dir); 

  int steps = abs(deg)*(1/0.225);
  float usDelay = (1/speed) * 70;

  for(int i=0; i < steps; i++){ 
    digitalWrite(step_pin, HIGH); 
    delayMicroseconds(usDelay); 

    digitalWrite(step_pin, LOW); 
    delayMicroseconds(usDelay); 
  }

}

//~~~@Author Alan Jenson || Used to dispense toppings
boolean dispense(int dirPin, int stepPin){
  //use the dispenser corresponding to dirPin and stepPin
  rotateDispDeg(290,.1,dirPin,stepPin);
  delay(200);
  for(int i=0;i<5;i++){
    rotateDispDeg(2,.1,dirPin,stepPin);
    delay(200);
  }
  rotateDispDeg(290,.1,dirPin,stepPin);
  delay(200);
  for(int i=0;i<5;i++){
    rotateDispDeg(2,.1,dirPin,stepPin);
    delay(200);
  }
  return false;
}

//~~~@Author Alan Jenson || Used to dispense ice
boolean dispenseIce(int dirPin, int stepPin){
  //use the dispenser corresponding to dirPin and stepPin
  rotateDispDeg(100,.1,dirPin,stepPin);
  
  return false;
}

//~~~From Dr. Francois Guimbretiere. Used to read the state change of the reed switches. 
byte read_button(byte pin, byte ref_value){
  // observed the state of the button
  byte current_button = digitalRead(pin);
  // There is a LOW -> HIGH transition
  // or a HIGH -> LOW transition
  if (((ref_value == LOW)
    && (current_button == HIGH))
    || ((ref_value == HIGH)
    && (current_button == LOW)))
  {
    // wait for a while (5ms)
    delay(5);
    // update the state of the button
    current_button = digitalRead(pin);
  }
  return(current_button);
}

//~~~Used to shift the turntable one slot to the right. Example from slot A to slot B.
boolean shiftTurntable(){
  analogWrite(turntable_motorPin, turntable_power);
  digitalWrite(ledPin, HIGH);
  
  // Read the state of the button
  switch_reedA = read_button(reedA, old_reedA);
  switch_reedB = read_button(reedB, old_reedB);
  switch_reedC = read_button(reedC, old_reedC);
  switch_reedD = read_button(reedD, old_reedD);
  switch_reedE = read_button(reedE, old_reedE);
  
  if ((old_reedA == LOW)
    && (switch_reedA == HIGH)
    && (lastSensor != 1)
    )
  {
    // LOW to HIGH transition:
    // we toggle the LED state
    digitalWrite(ledPin, LOW);
    analogWrite(turntable_motorPin, LOW);
    //Serial.println("Switch A");
    lastSensor = 1;
    delay(delayTime);
    return true;
    //analogWrite(turntable_motorPin, turntable_power);

  }else if(
  (old_reedB == LOW)
    && (switch_reedB == HIGH)
    && (lastSensor != 2)
    )
  {
    // LOW to HIGH transition:
    // we toggle the LED state
    digitalWrite(ledPin, LOW);
    analogWrite(turntable_motorPin, LOW);
    //Serial.println("Switch B");
    lastSensor = 2;
    delay(delayTime);
    return true;
    //analogWrite(turntable_motorPin, turntable_power);
    
  }else if(
  (old_reedC == LOW)
    && (switch_reedC == HIGH)
    && (lastSensor != 3)
    )
  {
    // LOW to HIGH transition:
    // we toggle the LED state
   digitalWrite(ledPin, LOW);
   analogWrite(turntable_motorPin, LOW);
   //Serial.println("Switch C");
   lastSensor = 3;
   delay(delayTime);
   return true;
   //analogWrite(turntable_motorPin, turntable_power);
  }else if(
  (old_reedD == LOW)
    && (switch_reedD == HIGH)
    && (lastSensor != 4)
    )
  {
    // LOW to HIGH transition:
    // we toggle the LED state
    digitalWrite(ledPin, LOW);
    analogWrite(turntable_motorPin, LOW);
    //Serial.println("Switch D");
    lastSensor = 4;
    delay(delayTime);
    return true;
    //(turntable_motorPin, turntable_power);
  }else if(
  (old_reedE == LOW)
    && (switch_reedE == HIGH)
    && (lastSensor != 5)
    )
  {
    // LOW to HIGH transition:
    // we toggle the LED state
    digitalWrite(ledPin, LOW);
    analogWrite(turntable_motorPin, LOW);
    //Serial.println("Switch E");
    lastSensor = 5;
    delay(delayTime);
    return true;
    //analogWrite(turntable_motorPin, turntable_power);
  }else{
    return false;
//      analogWrite(turntable_motorPin, turntable_power);
  //    digitalWrite(ledPin, HIGH);
  }
  
  // remember the state of the button
  old_reedA = switch_reedA;
  old_reedB = switch_reedB;
  old_reedC = switch_reedC;
  old_reedD = switch_reedD;
  old_reedE = switch_reedE;
}


//~~~Used to get the turntable back to the starting position
boolean resetTurntable(){
  switch_reedA = LOW;
  switch_reedB = LOW;
  switch_reedC = LOW;
  switch_reedD = LOW;
  switch_reedE = LOW;
  
  old_reedA = LOW;
  old_reedB = LOW;
  old_reedC = LOW;
  old_reedD = LOW;
  old_reedE = LOW;
  analogWrite(turntable_motorPin, turntable_power);
  digitalWrite(ledPin, HIGH);
  
  // Read the state of the button
  switch_reedA = read_button(reedA, old_reedA);
  switch_reedB = read_button(reedB, old_reedB);
  switch_reedC = read_button(reedC, old_reedC);
  switch_reedD = read_button(reedD, old_reedD);
  switch_reedE = read_button(reedE, old_reedE);
  if ((old_reedA == LOW)
    && (switch_reedA == HIGH))
  {
    // LOW to HIGH transition:
    // we toggle the LED state
    digitalWrite(ledPin, LOW);
    analogWrite(turntable_motorPin, LOW);
    //Serial.println("Switch A");
    //delay(15000);
    //analogWrite(turntable_motorPin, turntable_power);
    lastSensor = 0;
    return true;
  }else{
    return false;
   
  }
   // remember the state of the button
  old_reedA = switch_reedA;
}

void openCloseValve (Servo servo, int openAngle, int closedAngle){
  
  //Open the servo
  servo.write(servo_value_angle);
  delay(15000);
  
  //Close the servo
  servo.write(servo_value_angleClose);
  delay(1500);
}

void closeValve (Servo servo, int closedAngle){
  //Close the servo
  servo.write(closedAngle);
  delay(1500);
}

void whiskDownStirUp(){
  rotateDispDeg(stir_inverse_degrees, 0.1, stir_dirPin, stir_stepPin);
  digitalWrite(stir_motorPin, HIGH);
  delay(5000);
  digitalWrite(stir_motorPin, LOW);  
  rotateDispDeg(stir_degrees, 0.1, stir_dirPin, stir_stepPin);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%END OF FUNCTION DECLARATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%START OF SETUP & LOOP DECLARATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  
  //Input Pins
  pinMode(reedA, INPUT);
  pinMode(reedB, INPUT);
  pinMode(reedC, INPUT);
  pinMode(reedD, INPUT);
  pinMode(reedE, INPUT);
 
  //Output Pins
  pinMode(ice_dirPin,OUTPUT);
  pinMode(ice_stepPin,OUTPUT);
  pinMode(topping_dirPin,OUTPUT);
  pinMode(topping_stepPin,OUTPUT);
  pinMode(stir_dirPin,OUTPUT);
  pinMode(stir_stepPin,OUTPUT);
  pinMode(turntable_motorPin,OUTPUT);
  pinMode(stir_motorPin,OUTPUT);
  
  pinMode(ledPin,OUTPUT);
  
  //Servos
  vanillaServo.attach(servo_vanillaPin);
  chocolateServo.attach(servo_chocoPin);
}

void loop() {
  /*CODE STRUCTURE
  * 1. Recieve Input From GUI 
  *   a. Process Input and Store Values
  * 2. Shift Turntable One Position
  * 3. Dispense Liquids
  *   a. Open then Close Servos
  * 4. Shift Turntable One Position
  * 5. Dispense Ice & Liquids
  * 6. Shift Turntable One Position
  * 7. Lower Stirrer & Dispense ice and liquids
  *   a. Power Motor
  *   b. Turn off motor and then raise mechanism
  * 8. Shift Turntable One Position
  * 9. Dispense Toppings & dispense ice, liquids and stir
  * 10. Shift Turntable One Position
  * 11. Pause to pass along cup.
  * 12. Repeat 10 and 11 four times.
  * 13. Shift Turntable One Position to reset to starting position.
  */
  
  //Start by making sure the turntable is actually at slot A
  closeValve(vanillaServo, 0);
  closeValve(chocolateServo, 0);
  while (!resetTurntable()){}while(!shiftTurntable()){}
  
  //Read from the GUI and shift once to the right
  while(!readFromGUI());

  while(!shiftTurntable()){}
  

  //#################################STEPWISE CODE#################################
  //***Cup A at slot B
    //Dispense chocolate & close for cup A
    //Dispense vanilla & close for cup A
    if (van_A == 1)
      openCloseValve(vanillaServo, 180, 0);
    if (choc_A == 1)
      openCloseValve (chocolateServo, 180, 0);
  while(!shiftTurntable()){}

  //***Cup A at slot C, Cup E at Slot B
    //Dispense Ice for cup A
    if (van_A != 0 || choc_A !=0)
      dispenseIce(ice_dirPin, ice_stepPin);
    //Dispense chocolate & close for cup E?
    //Dispense vanilla & close for cup E?
    if (van_E == 1)
      openCloseValve(vanillaServo, 180, 0);
    if (choc_E == 1)
      openCloseValve (chocolateServo, 180, 0);
  while(!shiftTurntable()){}
    //***Cup A at slot D, Cup E at slot C, Cup D at slot B
    //Lower stirrer & stir for cup A
    if (van_A != 0 || choc_A !=0)
      whiskDownStirUp();
    //Dispense Ice for cup E
    if (van_E != 0 || choc_E !=0)
      dispenseIce(ice_dirPin, ice_stepPin);
    //Dispense chocolate & close for cup D?
    //Dispense vanilla & close for cup D?
    if (van_D == 1)
      openCloseValve(vanillaServo, 180, 0);
    if (choc_D == 1)
      openCloseValve (chocolateServo, 180, 0);
  while(!shiftTurntable()){}
  //***Cup A at slot E, Cup E at slot D, Cup D at slot C, Cup C at Slot B
    //Dispense topping for cup A
    if(top_A == 1)
      dispense(topping_dirPin, topping_stepPin);
    //Lower stirrer & stir for cup E
    if (van_E != 0 || choc_E !=0)
      whiskDownStirUp();
    //Dispense Ice for cup D
    if (van_D != 0 || choc_D !=0)
      dispenseIce(ice_dirPin, ice_stepPin);
    //Dispense chocolate & close for cup C?
    //Dispense vanilla & close for cup C?
    if (van_C == 1)
      openCloseValve(vanillaServo, 180, 0);
    if (choc_C == 1)
      openCloseValve (chocolateServo, 180, 0);
  while(!shiftTurntable()){}
  //***Cup A at slot A, Cup E at slot E, Cup D at slot D, Cup C at Slot C, Cup B at Slot B
    //Pause for removal of cup A
    
    //Dispense topping for cup E
    if(top_E == 1)
      dispense(topping_dirPin, topping_stepPin);
    //Lower stirrer & stir for cup D
    if (van_D != 0 || choc_D !=0)
      whiskDownStirUp();
    //Dispense Ice for cup C
    if (van_C != 0 || choc_C !=0)
      dispenseIce(ice_dirPin, ice_stepPin);
    //Dispense chocolate & close for cup B?
    //Dispense vanilla & close for cup B?
    if (van_B == 1)
      openCloseValve(vanillaServo, 180, 0);
    if (choc_B == 1)
      openCloseValve (chocolateServo, 180, 0);
  while(!shiftTurntable()){}
  //***Cup E at slot A, Cup D at slot E, Cup C at Slot D, Cup B at Slot C
    //Pause for removal of cup E
    
    
    //Dispense topping for cup D
    if(top_D == 1)
      dispense(topping_dirPin, topping_stepPin);
    //Lower stirrer & stir for cup C
    if (van_C != 0 || choc_C !=0)
      whiskDownStirUp();
    //Dispense Ice for cup B
    if (van_B != 0 || choc_B !=0)
      dispenseIce(ice_dirPin, ice_stepPin);
  while(!shiftTurntable()){}
  
  //***Cup D at slot A, Cup C at slot E, Cup B at Slot D
    //Pause for removal of cup D
    //Dispense topping for cup C
    if(top_C == 1)
      dispense(topping_dirPin, topping_stepPin);
    //Lower stirrer & stir for cup B
    if (van_B != 0 || choc_B !=0)
      whiskDownStirUp();
  while(!shiftTurntable()){}
    
  //***Cup C at slot A, Cup B at slot E
    //Pause for removal of cup C    
    //Dispense topping for cup B
    if(top_E == 1)
      dispense(topping_dirPin, topping_stepPin);
  while(!shiftTurntable()){}
 
  //***Cup B at slot A
    //Pause for removal of cup B
  //while(!resetTurntable()){}
  writeToGUI();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%END OF SETUP & LOOP DECLARATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


