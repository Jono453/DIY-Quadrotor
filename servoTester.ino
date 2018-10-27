//Created on 27/10/18 by Jonathan Loong
//PWM ServoTester
//Potentiometer -> manual
//PushButton -> automatic
//LEDs to indicate mode

#include <Servo.h>
#define ServoPin 5 //pwm on nano
#define PushPin1 2
#define ledPin 9 //green LED
#define ledPinTwo 6 //red LED
#define PWM_RANGE 12

Servo testServo;  //creating object testServo of class Servo

int pos = 0;    // variable to store the servo position
int shift = 20;  //variable to increment position +/- 10
char PWMvalue;
int autoFlag = 0;
int manualFlag = 1;
int potpin = 4;  // analog pin A4 used to connect the potentiometer

int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
//nano has interrupt on pins 2 and 3 only
volatile int autoInterrupt = 2; 
volatile int manualInterrupt = 3;
int autoCtr = 0;

int command[] = {0,30,60,90,120,150,180,150,120,90,60,30}; //array for PWM signals

void setup() {
  
  testServo.attach(ServoPin);  // attaches the servo to PWM @D5
  pinMode(PushPin1, INPUT); //pushbutton 1 as input
  pinMode(ledPin, OUTPUT);  //green status LED (auto)
  pinMode(ledPinTwo, OUTPUT); //red status LED (manual)

  //interrupts
  pinMode(autoInterrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(autoInterrupt),startAuto, RISING);
  pinMode(manualInterrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(manualInterrupt),startManual, RISING);

  testServo.write(0); //at beginning put servo to 0 position
  delay(15);  
  Serial.begin(9600);
}

void startAuto()
{
  autoFlag = 1; //raise flag to start auto servo sequence
  autoCtr++; //increment counter
}

void startManual()
{
  manualFlag = 1 - manualFlag;
}

//---Open Serial Monitor---

void loop() {
  //if pushbutton not pressed, allow manual mode only
  while (autoFlag != 1) //still equal 0
  {
    digitalWrite(ledPinTwo, HIGH); //red LED (manual on)
    digitalWrite(ledPin, LOW); //green LED (auto off)
    if (Serial.available()) 
    {
       //---waiting for potentiometer for manual control---
       /*

        pos = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
        pos = map(pos, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
        testServo.write(pos);                  // sets the servo position according to the scaled value
                
        */
       //Serial.print("manual mode"); 
       //Control servo arm position from keyboard (-shift)
       PWMvalue = Serial.read();
       if (PWMvalue == 'g') {  //g on keyboard (servo arm left)
         pos = pos - shift;       
         testServo.write(pos);            
         delay(15);   
       }
       else if (PWMvalue == 'h') { //h on keyboard (+shift)
         pos = pos + shift;
         testServo.write(pos);           
         delay(15);            
       }    
    }
  }
  
  //if pushbutton pressed, start auto sequence as autoFlag = 1;
  digitalWrite(ledPin, HIGH); //green LED (auto on)
  digitalWrite(ledPinTwo, LOW); //red LED (manual off) 
  //automated servo sequence (after pushbutton pressed)
  for (int i = 0; i < PWM_RANGE; i++)
  {
    testServo.write(command[i]);
    delay(15);
  }

  //if pushbutton is pressed a multiple of 3 stop auto mode
  //if not, continue running auto mode
  //0 -> press once -> 1 -> press twice -> 3
  if (autoCtr != 3 == 0)
  {
    autoFlag = 0;
  }
}
