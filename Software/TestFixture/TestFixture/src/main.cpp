#include <Arduino.h>
#include <PIO_DShot.h>

// put function declarations here:
int myFunction(int, int);

BidirDShotX1 *esc;


#define PLUNGER_PIN 10
#define TRIG_PIN 12
#define BOLT_LIMIT_PIN 8
#define MENU_X_PIN 26
#define MENU_Y_PIN 27
#define BOLT_HANDLE_PIN 18

void setup() {
	// initialize the ESC. This cannot be done globally (needs to know the clock speed). Your MCU will crash if you try.
	esc = new BidirDShotX1(PLUNGER_PIN);
	// esc = new BidirDShotX1(PIN, 600, pio0, -1); // -> optional parameters: speed, pio, sm (-1 = autodetect)
}

void loop() {
	// sendThrottle is non-blocking. That means, we must not send it too fast (manual delayMicroseconds).
	// at DShot600, the maximum achievable loop rate is about 9kHz. Roughly 6k for DShot300, 12k for DShot1200.
	delayMicroseconds(200);
	esc->sendThrottle(0);
}
void setup1()
{


}
void loop1()
{

  
}