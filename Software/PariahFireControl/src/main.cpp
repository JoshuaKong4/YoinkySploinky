#include <Arduino.h>

// put function declarations here:
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PIO_DShot.h>
#include <EEPROM.h>
#include <Fonts/TomThumb.h>
#include <string>

#define PLUNGER_PIN 10
#define PLUNGER_LIMIT_PIN 20
#define BOLT_PIN 13
#define TRIG_PIN 12
#define BOLT_LIMIT_PIN 8
#define MENU_X_PIN 26
#define MENU_Y_PIN 27
#define BOLT_HANDLE_PIN 18
#define MOTOR_POLES 14
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
BidirDShotX1 *esc;
BidirDShotX1 *bolt;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

uint32_t targetRPM = 1000;
uint32_t dwell = 1000;
uint32_t ac1 = 1000;
uint32_t ac2 = 1000;
uint32_t boltRetractTime = 30;
uint32_t unspool = 30;

enum fireState
{
  test,
  idle,
  kickstart,
  yanking,
  hold,
  retracting,
  dynamicidle,
  powerskip
};
enum boltState
{
  boltIdle,
  load,
  backward,
  backwardhold
};
fireState firestate = test;
boltState boltstate = boltIdle;
int throt = 0;
int boltthrot = 0;
long delta = 0;
long lastmicros = 0;
long cyclestart = 0;
long triggertime = 0; 
long boltstart = 0;
uint32_t rpm = 0;
uint32_t prawrpm = 0;
uint32_t maxrpm = 0;
uint32_t rawrpm = 0;
uint32_t prevrpm = 0;
uint32_t startrpm = 0;
int accel = 0;
int rawacc = 0; 
int prevaccel = 0;
int rawaccel = 0;
int crashcounter = 0;
int prevoutput = 0;
int direction = 1;
float alpha = 0.2;
float alphaaccel = 0.2;
float alphathrot = 0.1;
bool ignorecrash = true;
bool alreadypressed = true;
bool alreadypressedbolt = false;
bool alreadyreturned = false;

// put function declarations here:
long boltstatetime()
{
  return millis() - boltstart;
}

long firestatetime()
{
  return millis() - cyclestart;
}
void crst()
{

  cyclestart = millis();
}
void brst()
{

  boltstart = millis();
}
void sendinput(bool debug) // Interpret desired throttle value as an ESC input command before sending to ESC
{
  int directionSuffix = 500 + direction * 500;
  int filteredthrot = 0;
  if (throt < 0)
  {
    directionSuffix = 1000;
    filteredthrot = max(-50, -throt);
  }
  else if (throt == 0)
  {
    esc->sendThrottle(0);
    return;
  }
  else
  {
    filteredthrot = map(throt, 0, 1000, 51, 1000);
  }
   int inputvalue = abs(filteredthrot) + directionSuffix;
  if ( debug)
  {
    //Serial.print((String)firestate);
    Serial.print(",");
    Serial.print(firestatetime());
    Serial.print(",");
    Serial.print(inputvalue);
    Serial.print(",");
    Serial.print(rawrpm);
    Serial.print(",");
    Serial.print(rpm);
    Serial.print(",");
    Serial.println(rawacc);
    
  }

 
  esc->sendThrottle(inputvalue);
}

int crashdetect() // detects if the plunger is at the end of its movement by correlating RPM and acceleration
{
  int phasepos = - rpm - 5000;

  if (accel > phasepos || ignorecrash)
  {
    // direction = -1;
    return 0;
  }
  else
  {
    // direction = 1;
    return 1;
  }
}
int PID() // does some math to return what our ESC's throttle value should be based on target rpm and current rpm
{
  int error = (int)rpm - targetRPM; // Checking how far we are from rpm target
  int ssoutput = map(targetRPM, 0, 20000, 0, 1000);
  int rawinput = 0.22 * error; //+0.001*accel;
  // integral += error;
  int output = 0;
  prevoutput = output;
  output = max(1, min(1000, -rawinput + ssoutput));
  //output = max(1, min(1000,  ssoutput));
  // output =  alphathrot*output+(1-alphathrot)*prevoutput;
  output = min(output, map(rpm, 0, 10000, 80, 1000));
  return output;
  // return output;
}

void setup()
{
  Serial.begin(115200);
  pinMode(TRIG_PIN, INPUT_PULLUP);
  pinMode(BOLT_HANDLE_PIN, INPUT_PULLUP);
  pinMode(BOLT_LIMIT_PIN, INPUT_PULLUP);
  pinMode(PLUNGER_LIMIT_PIN, INPUT_PULLUP);
  esc = new BidirDShotX1(PLUNGER_PIN);
  bolt = new BidirDShotX1(BOLT_PIN);
}

void loop()
{

  delta = micros() - lastmicros; // delta = microseconds since our last loop
  lastmicros = micros();
  prevrpm = rpm;    
  prawrpm = rawrpm;           // store previous main motor rpm
  esc->getTelemetryErpm(&rpm); // get the main motor electrical rpm
  rpm /= MOTOR_POLES / 2;
  rawrpm = rpm; // convert to true rpm
  rawacc = (int)rawrpm-(int)prawrpm;
;  if (rpm > 50000)
  { // if we've somehow registered over 50krpm, we didn't, so the value is wrong, throw it away and use the previous rpm.
    rpm = prevrpm;
  }
  if (abs((int)rpm - (int)prevrpm) > rpm / 4)
  {
    rpm = alpha / 2 * rpm + (1 - alpha / 2) * prevrpm;
    // Serial.print("fkadsjflask");
  }
  else
  { 
    rpm = alpha * rpm + (1 - alpha) * prevrpm;
  } // Smooth the rpm a little by averaging it 75/25 with the previous rpm value
  prevaccel = accel;                                                                                 // store previous acceleration
  accel = alphaaccel * (((int)rpm - (int)prevrpm) / (delta * 10e-6)) + (1 - alphaaccel) * prevaccel; // Calculate and smooth current acceleration
  // if(rpm < 100) {  //Reset crash ignore status once rpm falls belo 100rpm
  //   ignorecrash = true;
  // }
  if (rpm > 2000)
  { // Once we've passed 2k rpms we can consider crashing
    ignorecrash = false;
  }

  if (firestate == test)
  {
    firestate = idle;
    if(!digitalRead(BOLT_HANDLE_PIN))
    {
        throt = 50;
      
    }
    else if (!digitalRead(TRIG_PIN))
    {

      throt = map(rpm, 0, 20000, 100, 1000);
        Serial.print(",");
    Serial.print(firestatetime());
    Serial.print(",");
    Serial.println(rawrpm);

    }
    else
    {

      throt = 0;
      cyclestart = millis();
    }

    
  }

  else if (firestate == idle) // if we're at idle send 0 throttle and reset some variables
  {
    throt = 0;
    // smoothedthrot = 1000;
    // prevrpm = 0;
    crashcounter = 0;
    crst();

    if (!digitalRead(TRIG_PIN) && !alreadypressed)
    {
      firestate = kickstart;
      prevrpm = 0;
      direction *= -1;
      alreadypressed = true;
      alreadyreturned = false;
      maxrpm = 0; 
      triggertime = millis();
    }
    else if(digitalRead(TRIG_PIN))
    {

      alreadypressed = false;
    }
  }

  else if (firestate == kickstart)
  {
      Serial.print("ks");
   Serial.print(",");
    Serial.print(digitalRead(PLUNGER_LIMIT_PIN));
    Serial.print(",");
    Serial.print(firestatetime());
    Serial.print(",");
    Serial.print(throt);
    Serial.print(",");
    Serial.print(rawrpm);
    Serial.print(",");
    Serial.print(rpm);
    Serial.print(",");
    Serial.println(accel);  
    if(!digitalRead(PLUNGER_LIMIT_PIN))
    {
      alreadyreturned = true;
    throt = 150;//map(firestatetime(),0,20,80,120);
    }
    else
    {
      throt = 20;
    }
    if ((firestatetime() > 20&&alreadyreturned)||firestatetime() > 100)
    {
      firestate = yanking;
      alreadyreturned = false;
      crst();
    }
    
  }

  else if (firestate == yanking)
  {
    Serial.print("yanking");
   Serial.print(",");
    Serial.print(digitalRead(PLUNGER_LIMIT_PIN));
    Serial.print(",");
    Serial.print(firestatetime());
    Serial.print(",");
    Serial.print(throt);
    Serial.print(",");
    Serial.print(rawrpm);
    Serial.print(",");
    Serial.print(rpm);
    Serial.print(",");
    Serial.println(accel); 
      throt = PID();
      if(rpm>maxrpm)
      {
        maxrpm = rpm;
      }
      if(crashdetect())
      {

        crashcounter++;
        throt = 0;
      } // check for crashes ; increment counter
     
    
    if (digitalRead(PLUNGER_LIMIT_PIN) || firestatetime() > 200) // If we've crashed or have been in this state for too long switch to the next state
    {

      firestate = hold;
      Serial.println("   ");
      Serial.println("NEW TEST");
      startrpm = rpm;
      
      crst();
    }
    
  }

  else if (firestate == hold)
  {
    if (firestatetime()>dwell) 
    {
      throt = -15; 
  
    }
    else
    {
      throt = map(firestatetime(),0,dwell,ac1,ac2); 
    }

    if (firestatetime() > dwell) // If we've been holding for too long or hit 0 rpm move to the retracting state
    {
      firestate = retracting;
      crst();
      boltstate = load;    //start cycling the bolt
      boltstart= millis();
    }


        Serial.print("HOLD");
   Serial.print(",");
    Serial.print(digitalRead(PLUNGER_LIMIT_PIN));
    Serial.print(",");
    Serial.print(firestatetime());
    Serial.print(",");
    Serial.print(throt);
    Serial.print(",");
    Serial.print(rawrpm);
    Serial.print(",");
    Serial.print(rpm);
    Serial.print(",");
    Serial.println(rawacc);
    // Serial.print(rpm);
    // Serial.print(",");
    //   Serial.println(accel);
  }

  else if (firestate == retracting)
  {
    throt = -30; // lower braking value slightly to speed up retraction
     //Serial.println(firestatetime());

    if (!digitalRead(PLUNGER_LIMIT_PIN)) // wait for the plunger to retract
    {

      firestate = dynamicidle;
      crst();
    }

  /*      Serial.print("retracc");
   Serial.print(",");
    Serial.print(digitalRead(PLUNGER_LIMIT_PIN));
    Serial.print(",");
    Serial.print(millis()-triggertime);
    Serial.print(",");
    Serial.print(throt);
    Serial.print(",");
    Serial.print(rawrpm);
    Serial.print(",");
    Serial.print(prawrpm);
    Serial.print(",");
    Serial.println(rawacc);  */
    if(firestatetime()>20)
    {

    if (!digitalRead(TRIG_PIN) && !alreadypressed)
    {
      firestate = powerskip;
      prevrpm = 0;
      direction *= -1;
      alreadypressed = true;
      maxrpm = 0; 
      crst();
    }
    else if(digitalRead(TRIG_PIN))
    {

      alreadypressed = false;
    }
  }
    // Serial.print(rpm);
    // Serial.print(",");
    //   Serial.println(accel);
  }
  else if (firestate == dynamicidle)
  {
    if(firestatetime()<unspool)
    {
 throt = 0;
    }
    else
    {
    throt = -49;
    }
    // smoothedthrot = 1000;
    //prevrpm = 0;
    crashcounter = 0;
   if(firestatetime()>20+unspool)
   {

    firestate = idle;
   }
      if (!digitalRead(TRIG_PIN) && !alreadypressed)
    {
      firestate = powerskip;
      prevrpm = 0;
      direction *= -1;
      alreadypressed = true;
      alreadyreturned = false;
      maxrpm = 0; 
      crst();
    }
    else if(digitalRead(TRIG_PIN))
    {

      alreadypressed = false;
    }


  }
  else if(firestate == powerskip)
  {
    throt = 0;
      if(firestatetime()>3)
      {
        firestate = kickstart;
        crst();
      }

  }

  sendinput(false);




  // bolt section

  if (boltstate == boltIdle) // idle state so we're not firing, check if the dpad is telling us to do something
  {
    boltthrot = 0;

    if (!digitalRead(BOLT_HANDLE_PIN) && !alreadypressedbolt && firestate == idle) // check if we're pressing on the dpad forward, home the bolt
    {
      // boltthrottle = 500;
      boltstate = load;
      boltstart = millis();
      alreadypressedbolt = true;
    }
    if (digitalRead(BOLT_HANDLE_PIN))
    {

      alreadypressedbolt = false;
    }
  }
  else if (boltstate == load) // Move the bolt forward to load a dart
  {
    int mappedinput = map(boltstatetime(),0,70,130,470);
    boltthrot = min(470,mappedinput);
    if (!digitalRead(BOLT_LIMIT_PIN) || boltstatetime() > 150) // check if the bolt is hitting the limit switch or timed out //Edited to use defined variable /wt
    {
      // should probably add something here in case it times out to handle that? /wt
      boltstate = backward;
      brst();
    }
  }
  else if (boltstate == backward) // move the bolt backwards quickly for a specific amount of time
  {
    if (boltstatetime() > 4)
    {
      int mappedinput = map(boltstatetime(),0,12,1350,1100);
    boltthrot = max(100,mappedinput);
    }

    if (boltstatetime() > boltRetractTime)
    {

      boltstate = backwardhold;
    }
  }
  else if (boltstate == backwardhold) // apply low power for a bit to prevent bolt from bouncing forward
  {
    boltthrot = 1250;
    if (boltstatetime() > 120) // once some time passes let go of the bolt, it'll stay backwards
    {
      boltstate = boltIdle;
    }
  }
  bolt->sendThrottle(boltthrot);

  // Serial.println(boltstate);

  delayMicroseconds(200);
}

// GUI
// STUFF
// BELOW

int menustate = 0;
int increment = 0;
int menudelay = 0;
int listfontheight = 10;
String settings[] = {"RPM", "DWELL", "CA1", "CA2","UNSPL"};
uint32_t *references[] = {&targetRPM, &dwell, &ac1, &ac2,&unspool};
int increments[] = {100, 1, 100, 100,5};
int defaults[] = {5000, 5, 0, 0,0};
int minval[] = {3000, 0, 0, 0,0};
int maxval[] = {9000, 12, 1000, 1000,120};
int menumax = 5;
int state = 0;

void setup1()
{

  pinMode(TRIG_PIN, INPUT_PULLUP);
  pinMode(BOLT_HANDLE_PIN, INPUT_PULLUP);
  pinMode(BOLT_LIMIT_PIN, INPUT_PULLUP);
  EEPROM.begin(10);
  delay(100);

  state = EEPROM.read(0);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  delay(2000);

  for (int i = 0; i < menumax; i++)
  {
    if (EEPROM.read(i) == 255)
    {
      EEPROM.write(i, defaults[i] / increments[i]);
    }

    *references[i] = (uint32_t)(EEPROM.read(i) * increments[i]);
  }
}

void loop1()
{

  menudelay = 50;
  increment = 0;
  if (analogRead(MENU_X_PIN) > 100 && analogRead(MENU_X_PIN) < 250)
  {
    menudelay = 200;
    if (menustate > 0)
    {
      menustate--;
    }
  }
  else if (analogRead(MENU_X_PIN) > 650)
  {

    menudelay = 200;
    if (menustate < menumax + 1)
    {
      menustate++;
      menudelay = 200;
    }
  }
  else if (analogRead(MENU_Y_PIN) > 100 && analogRead(MENU_Y_PIN) < 250)
  {
    menudelay = 120;
    increment = 1;
  }
  else if (analogRead(MENU_Y_PIN) > 650)
  {
    menudelay = 120;
    increment = -1;
  }
  if (menustate == 0)
  {
    display.clearDisplay();
    display.setRotation(3);
    display.setFont(&TomThumb);
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(0, 40);
    // Display static text
    display.println("Pariah");
    // display.println(state);
    display.setTextSize(2);
    display.setCursor(18, 60);
    display.println("SEMI");
    display.setCursor(18, 84);
    display.println("RPM:");
    display.setCursor(18, 98);
    display.print(maxrpm);
    //display.print(analogRead(28));
    display.display();
  }
  else
  {
    display.clearDisplay();
    display.setRotation(2);
    display.setFont(&TomThumb);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 6);

    uint32_t values[] = {targetRPM, dwell, ac1, ac2,unspool};
    

    for (int i = 0; i < menumax; i++)
    {
      display.setCursor(16, listfontheight * i + 15);
      display.print(settings[i]);
      display.setCursor(75, listfontheight * i + 15);
      display.print(values[i]);
    }
    display.setCursor(16, listfontheight * (menumax) + 15);
    display.print("SAVE ALL");

    display.setCursor(0, listfontheight * (menustate - 1) + 15);
    display.println(">");
    display.display();
    if (menustate <= menumax)
    {
      if(*references[menustate - 1]!=0||increment>0)
      {
      *references[menustate - 1] += increment * increments[menustate - 1];
      }
      if (*references[menustate - 1] >= maxval[menustate - 1])
      {
        *references[menustate - 1] = maxval[menustate - 1];
      }
      else if (*references[menustate - 1] <= minval[menustate - 1])
      {
        *references[menustate - 1] = minval[menustate - 1];
      }
    }
    else
    {
      if (increment != 0)
      {
        menudelay = 3000;
        display.clearDisplay();
        display.setRotation(2);
        display.setFont(&TomThumb);
        display.setTextSize(4);
        display.setTextColor(WHITE);
        display.setCursor(0, 40);
        display.println("SAVED");
        display.display();

        for (int i = 0; i < menumax; i++)
        {
          EEPROM.write(i, *references[i] / increments[i]);
        }
        EEPROM.commit();
      }
    }
  }

  delay(menudelay);
}
