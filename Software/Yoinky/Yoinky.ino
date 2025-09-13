/**
 * Yoinky Sploinky(TbAG) code
 * Designed and Written by VietKong308, some editing/commenting by Will(Beef)
 * Brushless, springless AEB
 * Uses a large brushless motor with a spool to yank the plunger with elastic to return the plunger
      This motor has no limit switches, it detects end of stroke when the motor rpm goes down/stops when the plunger hits the turnaround
      Then the motor brakes for a small amount of time to prevent bounceback, then freewheel to allow the elastic to pull it back forward
 * Dart feeding is done by a second motor that is brushed moving a rack and pinion to push darts into the turnaround
 *    This motor is currently a Kraken run by an esc in brushed mode, we should be able to swap to a regular bidirectional motor controller that's cheaper maybe?
 *    There is a return switch at the forward position of this motor, backwards travel is defined by a timer
 *    The 'idle' state of the feeder is backwards, with a dart loaded in the chamber that seals with a flapper
 * 
 *
 * wt edits: added comments, added a few variables, changed the states to enum
 * wt recommended code edits: 
 *    -Add a selector to allow for full-auto/semi-auto
 *    -Cache trigger pulls so that if you spam trigger it fires that many shots
 *    -Maybe use the debounce library for trigger debouncing, makes cached trigger code very clean
 *    -More variables to allow for easier tweaking
 */

//Libraries
#include <PIO_DShot.h>
#include <Servo.h>

//Pin definitions
#define ESC_PIN 10            //changed variable name /wt
#define BOLT_PIN 13
#define TRIG_PIN 12          //added defined variable /wt
#define BOLT_LIMIT_PIN 8 

#define MOTOR_POLES 14

//ESC definitions
BidirDShotX1 *esc;
BidirDShotX1 *bolt;

//Main motor variables
enum fireState {idle, yanking, hold, retracting};   //tracks the state of the plunger motor
int targetRPM = 6500;  //Target for the main motor's speed
int plungerRetractTime = 60; //retract time in ms
int boltRetractTime = 30;
uint16_t throttle = 0; //Value sent to esc
uint32_t rpm = 10;
uint32_t lastmicros = 0;
int delta = 0;
int yeeterstate = 0;  //unused?
int maxrpm = 0;       //unused?
long booststart = 0;
int throt = 0;
int trpm = 0;
int throtpid = 0;
int integral = 0;     //unused?
int lastrpm = 0;
int error = 0;
int acc = 0;
int prevrpm = 0; 
int direction = -1; 
int prevthrot = 0;
int accel = 0; 
float alpha = 0.75;
float alphaaccel = 0.5;
float alphathrot = 0.2;
int prevaccel = 0; 
bool rpmreached = false;  //unused?
int crashcounter; 
bool ignorecrash = false;
int smoothedthrot; 

//bolt variables
int bolttime = 0;
int boltthrottle = 0;
enum boltState {boltIdle, load, backward, backwardhold}; //Tracks the state of the dart pusher

boltState boltstate;
fireState firestate;

//Other variables
int shots = 0;                //For tracking how many shots are owed? not used yet
bool alreadypressed = false;  //For debouncing the trigger


int PID()  //does some math to return what our ESC's throttle value should be based on target rpm and current rpm
{
    int error = (int)rpm - targetRPM;  //Checking how far we are from rpm target

    int rawinput =0.15*error+0.001*accel; 
    //integral += error;
    if(rawinput>20)  //Begins braking motor if input exceeds certain threshold (either RPM or acceleration are too high)
    {
      return max(-1000,min(-1,-0.2*(rawinput-20)));
    }
    else if(rawinput>0)
    {
      return  1;
    }
    else
    {
          return max(1,min(1000,-rawinput)); 
    }
}

long firestatetime() //returns how long we've been in the current firestate
{
  return millis() - booststart; 
}

long boltstatetime() //returns how long we've been in the current boltstate
{
  return millis() - bolttime;
}

int crashdetect() //detects if the plunger is at the end of its movement by correlating RPM and acceleration
{
    int phasepos = -2*rpm-14000;
   
    if(accel>phasepos||ignorecrash) 
    {
      //direction = -1;
      return 0;
    }
    else
    {
      //direction = 1;
      return 1;
    }
}

void sendinput()//Interpret desired throttle value as an ESC input command before sending to ESC 
{
  int directionSuffix = 500+direction*500;
  int filteredthrot = 0; 

  if(throt<smoothedthrot&&0<throt)
  {
    smoothedthrot = alphathrot*throt+(1-alphathrot)*smoothedthrot; 
  }
  
  if(firestate == yanking)
  {
    filteredthrot = min(throt,smoothedthrot); 
  }
  else
  {
      filteredthrot = throt;

  }

  if(filteredthrot < 0)
  {
    directionSuffix = 500-direction*500;
  }
  else if(filteredthrot ==0)
  {
  directionSuffix = 0;
  }

  int inputvalue = abs(filteredthrot) + directionSuffix;

  if(prevthrot >1000 && (inputvalue <=1000 && inputvalue >0)||inputvalue >1000 && (prevthrot <=1000 && prevthrot >0))  //controls whether we need to be braking or just sending throttle
  {
    esc->sendThrottle(0);      //send some 0 throttle to do rc car braking to get proportional braking
    delayMicroseconds(200);
    esc->sendThrottle(inputvalue);
  }
  else
  {
    esc->sendThrottle(inputvalue);

    //  Serial.print(rpm);
    // Serial.print(",");
    // Serial.print(",");
    // Serial.println(throt);

  }
  prevthrot = inputvalue; 
 
}

void setup() {  //Initialize pins and esc's
  Serial.begin(115200);
  // myservo.attach(9);
  esc = new BidirDShotX1(ESC_PIN);
  bolt = new BidirDShotX1(BOLT_PIN); 
  pinMode(TRIG_PIN, INPUT_PULLUP);       //Edited to use defined variables /wt
  pinMode(BOLT_LIMIT_PIN, INPUT_PULLUP);
  //pinMode(7, INPUT);
}

void loop() {

  delta = micros() - lastmicros;  //delta = microseconds since our last loop
  lastmicros = micros();

  prevrpm = rpm; //store previous main motor rpm
  esc->getTelemetryErpm(&rpm); //get the main motor electrical rpm
  rpm /= MOTOR_POLES / 2;      //convert to true rpm
  if(rpm>50000){             //if we've somehow registered over 50krpm, we didn't, so the value is wrong, throw it away and use the previous rpm.
    rpm = prevrpm; 
  }
  rpm = alpha*rpm+(1-alpha)*prevrpm;   //Smooth the rpm a little by averaging it 75/25 with the previous rpm value

  prevaccel = accel;  //store previous acceleration
  accel = alphaaccel*(((int)rpm -(int)prevrpm)/(delta*10e-6))+(1-alphaaccel)*prevaccel;  //Calculate and smooth current acceleration
  if(rpm < 100) {  //Reset crash ignore status once rpm falls belo 100rpm
    ignorecrash = true;
  }
  if(rpm > 2000) { //Once we've passed 2k rpms we can consider crashing
    ignorecrash = false;
  }
  
  if(!digitalRead(TRIG_PIN))  //Checking if the trigger is pressed  //Edited to use defined variable /wt
  {
    if(!alreadypressed && firestate == idle) //If we haven't already registered this trigger press, and are in the idle state
    {
      alreadypressed = true;
      direction*=-1;
      firestate = yanking;
      //
    }
  }
  else
  {
    alreadypressed = false; 
  }

  if (firestate == yanking)
  {
    
  
    
  if(firestatetime()>12)
  {
      throt = PID();
    crashcounter += crashdetect();  //check for crashes ; increment counter 
  }
  else
  {

    throt = 600;
  }
    if(crashcounter > 3 || firestatetime() > 200) //If we've crashed or have been in this state for too long switch to the next state
    {

      firestate = hold; 
    
    }
      Serial.print(rpm);
    Serial.print(",");
    Serial.print(accel);
    Serial.print(",");
    Serial.print(throt);
    Serial.print(",");
    Serial.println(firestatetime());

  }
  else if (firestate == hold)
  {
    if(rpm<2200)  //Full brake once RPM falls below certain value to prevent plunger from bouncing
    {
      throt = -1000; // Negative throttle causes the motor to brake due to rc car braking logic, which brakes motor when throttle changes direction, and changes spin direction
      //when reverse is 'double-tapped' ( reverse, neutral, reverse)
      //Important - Dshot cannot interpret negative values! This is done in the sendinput() command

    }
    else
    {
      throt = 1; //Otherwise send basically 0 throttle (needed to work with rc car braking)
    }
    
    if(firestatetime() > 500 || rpm == 0) //If we've been holding for too long or hit 0 rpm move to the retracting state
    {
      firestate = retracting; 
      booststart = millis();
      boltstate = load;    //start cycling the bolt
        bolttime = millis();
    }
      // Serial.print(rpm);
      // Serial.print(",");
      //   Serial.println(accel);
  }
  else if (firestate == retracting )  
  {
      throt = -400;// lower braking value slightly to speed up retraction
  
    if(firestatetime() > plungerRetractTime) //wait for the plunger to retract
    {

      firestate = idle; 
    }
      // Serial.print(rpm);
      // Serial.print(",");
      //   Serial.println(accel);
      
  }

  else if(firestate == idle)  //if we're at idle send 0 throttle and reset some variables
  {
    throt = 0; 
    smoothedthrot = 1000; 
    crashcounter = 0;
    booststart = millis(); 

  }
  sendinput();

  //bolt stuff 
  if (boltstate == boltIdle) //idle state so we're not firing, check if the dpad is telling us to do something
  {
    boltthrottle = 0; 

      if(analogRead(27) > 650)  //check if we're pressing on the dpad forward, home the bolt
      {
        //boltthrottle = 500;
        boltstate = load;
        bolttime = millis();
      }
      else if(analogRead(27)<300) //if we're pressing backward on the dpad, retract
      {
        boltthrottle = 1500;
      }

  // Serial.println(digitalRead(RACK_LIMIT_PIN));

  }
  else if(boltstate == load) //Move the bolt forward to load a dart
  {
    boltthrottle = 250;
    if(boltthrottle<550)
    {

      boltthrottle+=5;
    }
    if(digitalRead(BOLT_LIMIT_PIN) || boltstatetime() > 150)  //check if the bolt is hitting the limit switch or timed out //Edited to use defined variable /wt
    {
        //should probably add something here in case it times out to handle that? /wt
      boltstate = backward; 
      bolttime = millis(); 
    }
  }
  else if (boltstate == backward) //move the bolt backwards quickly for a specific amount of time
  {
    if(boltstatetime()>4)
    {
      boltthrottle = 1350;
      
    }
    
      if(boltstatetime() > boltRetractTime)  
      {

        boltstate = backwardhold; 
      }
    
  }
  else if (boltstate == backwardhold) //apply low power for a bit to prevent bolt from bouncing forward 
  {
    boltthrottle = 1250;
      if(boltstatetime() > 120) //once some time passes let go of the bolt, it'll stay backwards
      {
        boltstate = boltIdle; 
      }
    
  }
  bolt->sendThrottle(boltthrottle);

  //Serial.println(boltstate);
 
  delayMicroseconds(500); //wait a little before doing the next loop

}
