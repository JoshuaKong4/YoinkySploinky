#include <Arduino.h>

// put function declarations here:
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PIO_DShot.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
BidirDShotX1 *esc;
BidirDShotX1 *bolt;


#define PLUNGER_PIN 10
#define BOLT_PIN 13
#define TRIG_PIN 12
#define BOLT_LIMIT_PIN 8
#define MENU_X_PIN 26
#define MENU_Y_PIN 27
#define BOLT_HANDLE_PIN 18
uint32_t throt = 0; 
uint32_t boltthrot = 0; 
uint32_t rpm = 0; 
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(115200);


  pinMode(TRIG_PIN,INPUT_PULLUP);
  pinMode(BOLT_HANDLE_PIN,INPUT_PULLUP);
  pinMode(BOLT_LIMIT_PIN ,INPUT_PULLUP);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Press Trigger");
  display.display(); 
    while(digitalRead(TRIG_PIN))
  {
    delay(2);
  }
  display.clearDisplay();
  display.println("Press Bolt Cycle");
  display.display(); 

   while(digitalRead(BOLT_HANDLE_PIN))
  {
    delay(2);
  }
   while(digitalRead(TRIG_PIN))
  {
  display.clearDisplay();
  display.setCursor(0, 10); 
  display.println("DPAD TEST");
  display.println("PRESS TRIG TO CONFIRM");
  display.println(analogRead(MENU_X_PIN));
  display.println(analogRead(MENU_Y_PIN));
  display.display(); 
    delay(20);
  }
 
   while(digitalRead(BOLT_HANDLE_PIN))
  {

     display.clearDisplay();
  display.setCursor(0, 10); 
  display.println("PRESS TRIG");
  display.println("MOTOR WILL SPIN");
  display.println("PRESS BOLT TO CONFIRM");
  display.println("rpm:");
  display.println(rpm);
  display.display(); 
    

    if(digitalRead(TRIG_PIN))
    {

      throt = 0; 
    }
    else
    {

      throt = 100; 
    }
    delay(20);
  }
    throt = 0;
    delay(2000);

     display.clearDisplay();
  display.setCursor(0, 10); 
  display.println("PRESS BOLT");
  display.println("BOLT WILL CYCLE");
  display.println("PRESS TRIG TO CONFIRM");
  display.display(); 
  while(digitalRead(TRIG_PIN))
  {
    delay(20);
   
      if(!digitalRead(BOLT_HANDLE_PIN))
      {
          boltthrot= 200;
          delay(80);
          boltthrot= 0;
          if(!digitalRead(BOLT_LIMIT_PIN))
          {

             display.println("LIMIT SWITCH TRIGD");
             display.display();
          }
          delay(200);
          boltthrot= 1200;
          delay(80);
          boltthrot= 0;
          delay(500);


      }
  }
  
display.clearDisplay();
  display.setCursor(0, 10); 
  display.println("TEST OVER");
  display.display(); 
}

void loop() 
{

  
}
void setup1()
{
esc = new BidirDShotX1(PLUNGER_PIN);
bolt = new BidirDShotX1(BOLT_PIN);

}
void loop1()
{
  delayMicroseconds(200);
	esc->sendThrottle(throt);
  bolt->sendThrottle(boltthrot);
  esc->getTelemetryErpm(&rpm);
	rpm /= 7;

}