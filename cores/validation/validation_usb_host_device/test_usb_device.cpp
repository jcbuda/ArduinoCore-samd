
#define DEDE
#ifdef DEDE

// /opt/local/bin/pv /dev/cu.usbmodem621 > /dev/null
// /opt/local/bin/pv /dev/cu.usbmodem2621 > /dev/null
//
// Arduino DUE .......... > 1.5 MB/s
// Arduino Leonardo ..... > 330 kB/s
// Arduino ZERO ......... 48.8 kB/s
#include "Arduino.h"


//#if defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAM_DUE)
#define SERIAL_PORT SerialUSB
//#else
//#define SERIAL_PORT Serial
//#endif

void setup()
{
  USBDevice.init();
  USBDevice.attach();
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);

  SERIAL_PORT.begin(115200);
}


void burstSendTest();
void loopbackMultiBytesTest();
void loopbackSingleByteTest();
void speedTest();


void loop()
{
  int c = SERIAL_PORT.read();

  if (c == -1)
    return;

  switch (c)
  {
    case 'b' :
      burstSendTest();
      break;

    case 'l' :
      loopbackSingleByteTest();
      break;

    case 'L' :
      loopbackMultiBytesTest();
      break;

    case 's' :
      speedTest();
      break;
  }
}

void burstSendTest()
{
  // Send a burst of 1.000.000 bytes of data
  for (uint16_t i = 0; i < 20000; i++)
  {
    // 25 bytes
    SERIAL_PORT.print("0123456789abcdeABCDE=)(/&");
    // 25 bytes
    SERIAL_PORT.print("qwertyuiopasdfghjklzQWERT");
  }
}

void loopbackMultiBytesTest()
{
  while (true) {
   char aux[1024];
   int i = 0;
    //SerialUSB.flush();
// 	if (SerialUSB.available() > 0)
// 	{
// 		char inChar;
// 		while( -1 == (inChar = SerialUSB.read()));
// 		SerialUSB.print(inChar);
// 	} 
    while (SERIAL_PORT.available() > 0 && i < (sizeof(aux) - 1))
    {
      char c = SERIAL_PORT.read();
      aux[i++] = c;
    }
    if (i > 0) {
      aux[i++] = 0;
      SERIAL_PORT.print(aux);
    }
  }
}

void loopbackSingleByteTest() {
  while (true)
  {
    int c = SERIAL_PORT.read();
    if (c != -1)
      SERIAL_PORT.print((char) c);
  }
}

void speedTest()
{
  int frameSize = SERIAL_PORT.parseInt();
  if (frameSize < 1)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    while (true);
  }

  uint8_t speedTestBuffer[frameSize];
  for (int i = 0; i < frameSize; ++i)
    speedTestBuffer[i] = (uint8_t) (i & 0xFF);

  while (true)
  {
    SERIAL_PORT.write(speedTestBuffer, frameSize);
  }
}
 
#endif


//#define TOTO

#ifdef TOTO

//#define ARDUINO_MAIN
#include "Arduino.h"

void setup()
{
  USBDevice.init();
  USBDevice.attach();
  SerialUSB.begin(115200);
  while (!SerialUSB) ;
}

void loop()
{
	
/*
		if (SerialUSB.available() > 0)
		{
			char inChar;
			if( -1 != (inChar = SerialUSB.read() ) )  {
				SerialUSB.print(inChar);
			}
		}
		delay(10);
*/

/*
	if (SerialUSB.available() > 0)
	{
		char inChar;
		while( -1 == (inChar = SerialUSB.read()));
		SerialUSB.print(inChar);
	} 
*/
		
	
   String  aux = " ";
      SerialUSB.flush();
      while( SerialUSB.available() > 0 )
      {
         char c = SerialUSB.read();
         aux += c;
      }
      if(aux != " ")
         SerialUSB.println(aux);

}

#endif


#ifdef XXXX
#define ARDUINO_MAIN
#include "Arduino.h"

#ifdef HID_ENABLED
const int buttonPin = 4;          // input pin for pushbutton
int previousButtonState = HIGH;   // for checking the state of a pushButton
int counter = 0;                  // button push counter
#endif

void setup(void)
{
  USBDevice.init();
  USBDevice.attach();
#ifdef HID_ENABLED
	Mouse.begin();

	// make the pushButton pin an input:
	pinMode(buttonPin, INPUT);
	// initialize control over the keyboard:
	Keyboard.begin();
#endif

#ifdef CDC_ENABLED
	SerialUSB.begin(115200);
#endif
}

void loop(void)
{
	#ifdef HID_ENABLED
	Mouse.move(1, 0, 0);

	// read the pushbutton:
	int buttonState = digitalRead(buttonPin);
	// if the button state has changed, and it's currently pressed:
	if ((buttonState != previousButtonState) && (buttonState == HIGH))
	{
		// increment the button counter
		counter++;
		// type out a message
		Keyboard.print("You pressed the button ");
		Keyboard.print(counter);
		Keyboard.println(" times.");
	}
	// save the current button state for comparison next time:
	previousButtonState = buttonState;
	#endif

	#ifdef CDC_ENABLED
	if (SerialUSB.available() > 0)
	{
		char inChar;
		if( -1 != (inChar = SerialUSB.read() ) )  {
			SerialUSB.print(inChar);
		}
	}
	delay(10);
	
	#endif
}

#endif
