/*
  Copyright (c) 2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


// #define ADK_VALID
#ifdef ADK_VALID

#define ARDUINO_MAIN
//#include "variant.h"
#include "Arduino.h" 
#include <stdio.h>
#include <adk.h>


USBHost usb;
ADK adk(&usb,"Arduino SA",
            "Arduino_Terminal",
            "Arduino Terminal for Android",
            "1.0",
            "http://labs.arduino.cc/uploads/ADK/ArduinoTerminal/ThibaultTerminal_ICS_0001.apk",
            "1");

void setup(void)
{
  SERIAL_PORT_MONITOR.begin( 115200 );
  while (!SERIAL_PORT_MONITOR); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  SERIAL_PORT_MONITOR.println("\r\nADK demo start");

  if (usb.Init() == -1)
	SERIAL_PORT_MONITOR.println("OSC did not start.");

  delay(20);
}

#define RCVSIZE 128

void loop(void)
{
	uint8_t buf[RCVSIZE];
	uint32_t nbread = 0;
	char helloworld[] = "Hello World!\r\n";

	usb.Task();

	if( adk.isReady() == false ) {
		return;
	}
	/* Write hello string to ADK */
	adk.SndData(strlen(helloworld), (uint8_t *)helloworld);

	delay(1000);

	/* Read data from ADK and print to UART */
	adk.RcvData((uint8_t *)&nbread, buf);
	if (nbread > 0)
	{
		SERIAL_PORT_MONITOR.print("RCV: ");
		for (uint32_t i = 0; i < nbread; ++i)
		{
			SERIAL_PORT_MONITOR.print((char)buf[i]);
		}
		SERIAL_PORT_MONITOR.print("\r\n");
	}	
} 

#endif // ADK_VALID

// #define KB_VALID
#ifdef KB_VALID
/*
 Keyboard Controller Example

 Shows the output of a USB Keyboard connected to
 the Native USB port on an Arduino Due Board.

 created 8 Oct 2012
 by Cristian Maglie

 http://arduino.cc/en/Tutorial/KeyboardController

 This sample code is part of the public domain.
 */

// Require keyboard control library
#include <KeyboardController.h>

// Initialize USB Controller
USBHost usb;

// Attach keyboard controller to USB
KeyboardController keyboard(usb);

void printKey();

// This function intercepts key press
void keyPressed() {
  SERIAL_PORT_MONITOR.print("Pressed:  ");
  printKey();
}

// This function intercepts key release
void keyReleased() {
  SERIAL_PORT_MONITOR.print("Released: ");
  printKey();
}

void printKey() {
  // getOemKey() returns the OEM-code associated with the key
  SERIAL_PORT_MONITOR.print(" key:");
  SERIAL_PORT_MONITOR.print(keyboard.getOemKey());

  // getModifiers() returns a bits field with the modifiers-keys
  int mod = keyboard.getModifiers();
  SERIAL_PORT_MONITOR.print(" mod:");
  SERIAL_PORT_MONITOR.print(mod);

  SERIAL_PORT_MONITOR.print(" => ");

  if (mod & LeftCtrl)
    SERIAL_PORT_MONITOR.print("L-Ctrl ");
  if (mod & LeftShift)
    SERIAL_PORT_MONITOR.print("L-Shift ");
  if (mod & Alt)
    SERIAL_PORT_MONITOR.print("Alt ");
  if (mod & LeftCmd)
    SERIAL_PORT_MONITOR.print("L-Cmd ");
  if (mod & RightCtrl)
    SERIAL_PORT_MONITOR.print("R-Ctrl ");
  if (mod & RightShift)
    SERIAL_PORT_MONITOR.print("R-Shift ");
  if (mod & AltGr)
    SERIAL_PORT_MONITOR.print("AltGr ");
  if (mod & RightCmd)
    SERIAL_PORT_MONITOR.print("R-Cmd ");

  // getKey() returns the ASCII translation of OEM key
  // combined with modifiers.
  SERIAL_PORT_MONITOR.write(keyboard.getKey());
  SERIAL_PORT_MONITOR.println();
}

void setup()
{
  SERIAL_PORT_MONITOR.begin( 115200 );
  while (!SERIAL_PORT_MONITOR); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  SERIAL_PORT_MONITOR.println("Keyboard Controller Program started");

  if (usb.Init() == -1)
	  SERIAL_PORT_MONITOR.println("OSC did not start.");
  
  delay( 20 );
}

void loop()
{
  // Process USB tasks
  usb.Task();
}

#endif // KB_VALID



#define MS_VALID
#ifdef MS_VALID
/*
 Mouse Controller Example

 Shows the output of a USB Mouse connected to
 the Native USB port on an Arduino Due Board.

 created 8 Oct 2012
 by Cristian Maglie

 http://arduino.cc/en/Tutorial/MouseController

 This sample code is part of the public domain.
 */

// Require mouse control library
#include <MouseController.h>

// Initialize USB Controller
USBHost usb;

// Attach mouse controller to USB
MouseController mouse(usb);

// variables for mouse button states
boolean leftButton = false;
boolean middleButton = false;
boolean rightButton = false;

// This function intercepts mouse movements
void mouseMoved() {
  SERIAL_PORT_MONITOR.print("Move: ");
  SERIAL_PORT_MONITOR.print(mouse.getXChange());
  SERIAL_PORT_MONITOR.print(", ");
  SERIAL_PORT_MONITOR.println(mouse.getYChange());
}

// This function intercepts mouse movements while a button is pressed
void mouseDragged() {
  SERIAL_PORT_MONITOR.print("DRAG: ");
  SERIAL_PORT_MONITOR.print(mouse.getXChange());
  SERIAL_PORT_MONITOR.print(", ");
  SERIAL_PORT_MONITOR.println(mouse.getYChange());
}

// This function intercepts mouse button press
void mousePressed() {
  SERIAL_PORT_MONITOR.print("Pressed: ");
  if (mouse.getButton(LEFT_BUTTON)) {
    SERIAL_PORT_MONITOR.print("L");
    leftButton = true;
  }
  if (mouse.getButton(MIDDLE_BUTTON)) {
    SERIAL_PORT_MONITOR.print("M");
    middleButton = true;
  }
  if (mouse.getButton(RIGHT_BUTTON)) {
    SERIAL_PORT_MONITOR.print("R");
    rightButton = true;
  }
  SERIAL_PORT_MONITOR.println();
}

// This function intercepts mouse button release
void mouseReleased() {
  SERIAL_PORT_MONITOR.print("Released: ");
  if (!mouse.getButton(LEFT_BUTTON) && leftButton == true) {
    SERIAL_PORT_MONITOR.print("L");
    leftButton = false;
  }
  if (!mouse.getButton(MIDDLE_BUTTON) && middleButton == true) {
    SERIAL_PORT_MONITOR.print("M");
    middleButton = false;
  }
  if (!mouse.getButton(RIGHT_BUTTON) && rightButton == true) {
    SERIAL_PORT_MONITOR.print("R");
    rightButton = false;
  }
  SERIAL_PORT_MONITOR.println();
}

void setup()
{
  SERIAL_PORT_MONITOR.begin( 115200 );
  while (!SERIAL_PORT_MONITOR); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  SERIAL_PORT_MONITOR.println("Mouse Controller Program started");

  if (usb.Init() == -1)
      SERIAL_PORT_MONITOR.println("OSC did not start.");

  delay( 20 );
}

void loop()
{
  // Process USB tasks
  usb.Task();
}
#endif // MS_VALID


// #define DESC_VALID
#ifdef DESC_VALID
#include "Arduino.h"
#include <usbhub.h>
#include "wiring_constants.h"
#include "pgmstrings.h"
// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USBHost     usb;
//USBHub  Hub1(&Usb);
//USBHub  Hub2(&Usb);
//USBHub  Hub3(&Usb);
//USBHub  Hub4(&Usb);
//USBHub  Hub5(&Usb);
//USBHub  Hub6(&Usb);
//USBHub  Hub7(&Usb);

uint32_t next_time;

void print_hex(int v, int num_places);
void printintfdescr( uint8_t* descr_ptr );
byte getconfdescr( byte addr, byte conf );
void printconfdescr( uint8_t* descr_ptr );
void printunkdescr( uint8_t* descr_ptr );
void printepdescr( uint8_t* descr_ptr );
void printProgStr(const prog_char str[]);
void printHIDdescr( uint8_t* descr_ptr );

void PrintAllAddresses(UsbDeviceDefinition *pdev)
{
    UsbDeviceAddress adr;
    adr.devAddress = pdev->address.devAddress;
    SERIAL_PORT_MONITOR.print("\r\nAddr:");
    SERIAL_PORT_MONITOR.print(adr.devAddress, HEX);
    SERIAL_PORT_MONITOR.print("(");
    SERIAL_PORT_MONITOR.print(adr.bmHub, HEX);
    SERIAL_PORT_MONITOR.print(".");
    SERIAL_PORT_MONITOR.print(adr.bmParent, HEX);
    SERIAL_PORT_MONITOR.print(".");
    SERIAL_PORT_MONITOR.print(adr.bmAddress, HEX);
    SERIAL_PORT_MONITOR.println(")");
}

void PrintAddress(uint8_t addr)
{
    UsbDeviceAddress adr;
    adr.devAddress = addr;
    SERIAL_PORT_MONITOR.print("\r\nADDR:\t");
    SERIAL_PORT_MONITOR.println(adr.devAddress,HEX);
    SERIAL_PORT_MONITOR.print("DEV:\t");
    SERIAL_PORT_MONITOR.println(adr.bmAddress,HEX);
    SERIAL_PORT_MONITOR.print("PRNT:\t");
    SERIAL_PORT_MONITOR.println(adr.bmParent,HEX);
    SERIAL_PORT_MONITOR.print("HUB:\t");
    SERIAL_PORT_MONITOR.println(adr.bmHub,HEX);
}

void setup()
{
  SERIAL_PORT_MONITOR.begin( 115200 );
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  SERIAL_PORT_MONITOR.println("Start USB Desc");

  if (usb.Init() == -1)
      SERIAL_PORT_MONITOR.println("OSC did not start.");

  delay( 20 );

  next_time = millis() + 10000;
}

byte getdevdescr( byte addr, byte &num_conf );

void PrintDescriptors(uint8_t addr)
{
    uint8_t rcode = 0;
    byte num_conf = 0;

    rcode = getdevdescr( (byte)addr, num_conf );
    if( rcode )
    {
      printProgStr(Gen_Error_str);
      print_hex( rcode, 8 );
    }
    SERIAL_PORT_MONITOR.print("\r\n");

    for (int i=0; i<num_conf; i++)
    {
        rcode = getconfdescr( addr, i );                 // get configuration descriptor
        if( rcode )
        {
          printProgStr(Gen_Error_str);
          print_hex(rcode, 8);
        }
        SERIAL_PORT_MONITOR.println("\r\n");
    }
}

void PrintAllDescriptors(UsbDeviceDefinition *pdev)
{
    SERIAL_PORT_MONITOR.println("\r\n");
    print_hex(pdev->address.devAddress, 8);
    SERIAL_PORT_MONITOR.println("\r\n--");
    PrintDescriptors( pdev->address.devAddress );
}

void loop()
{
  usb.Task();

  if( usb.getUsbTaskState() == USB_STATE_RUNNING )
  {
    //if (millis() >= next_time)
    {
        usb.ForEachUsbDevice(&PrintAllDescriptors);
        usb.ForEachUsbDevice(&PrintAllAddresses);

        while( 1 );                           //stop
    }
  }
}

byte getdevdescr( byte addr, byte &num_conf )
{
  USB_DEVICE_DESCRIPTOR buf;
  byte rcode;
  rcode = usb.getDevDescr( addr, 0, 0x12, ( uint8_t *)&buf );
  if( rcode ) {
    return( rcode );
  }
  printProgStr(Dev_Header_str);
  printProgStr(Dev_Length_str);
  print_hex( buf.bLength, 8 );
  printProgStr(Dev_Type_str);
  print_hex( buf.bDescriptorType, 8 );
  printProgStr(Dev_Version_str);
  print_hex( buf.bcdUSB, 16 );
  printProgStr(Dev_Class_str);
  print_hex( buf.bDeviceClass, 8 );
  printProgStr(Dev_Subclass_str);
  print_hex( buf.bDeviceSubClass, 8 );
  printProgStr(Dev_Protocol_str);
  print_hex( buf.bDeviceProtocol, 8 );
  printProgStr(Dev_Pktsize_str);
  print_hex( buf.bMaxPacketSize0, 8 );
  printProgStr(Dev_Vendor_str);
  print_hex( buf.idVendor, 16 );
  printProgStr(Dev_Product_str);
  print_hex( buf.idProduct, 16 );
  printProgStr(Dev_Revision_str);
  print_hex( buf.bcdDevice, 16 );
  printProgStr(Dev_Mfg_str);
  print_hex( buf.iManufacturer, 8 );
  printProgStr(Dev_Prod_str);
  print_hex( buf.iProduct, 8 );
  printProgStr(Dev_Serial_str);
  print_hex( buf.iSerialNumber, 8 );
  printProgStr(Dev_Nconf_str);
  print_hex( buf.bNumConfigurations, 8 );
  num_conf = buf.bNumConfigurations;
  return( 0 );
}

void printhubdescr(uint8_t *descrptr, uint8_t addr)
{
    HubDescriptor  *pHub = (HubDescriptor*) descrptr;
    uint8_t        len = *((uint8_t*)descrptr);

    printProgStr(PSTR("\r\n\r\nHub Descriptor:\r\n"));
    printProgStr(PSTR("bDescLength:\t\t"));
    SERIAL_PORT_MONITOR.println(pHub->bDescLength, HEX);

    printProgStr(PSTR("bDescriptorType:\t"));
    SERIAL_PORT_MONITOR.println(pHub->bDescriptorType, HEX);

    printProgStr(PSTR("bNbrPorts:\t\t"));
    SERIAL_PORT_MONITOR.println(pHub->bNbrPorts, HEX);

    printProgStr(PSTR("LogPwrSwitchMode:\t"));
    SERIAL_PORT_MONITOR.println(pHub->LogPwrSwitchMode, BIN);

    printProgStr(PSTR("CompoundDevice:\t\t"));
    SERIAL_PORT_MONITOR.println(pHub->CompoundDevice, BIN);

    printProgStr(PSTR("OverCurrentProtectMode:\t"));
    SERIAL_PORT_MONITOR.println(pHub->OverCurrentProtectMode, BIN);

    printProgStr(PSTR("TTThinkTime:\t\t"));
    SERIAL_PORT_MONITOR.println(pHub->TTThinkTime, BIN);

    printProgStr(PSTR("PortIndicatorsSupported:"));
    SERIAL_PORT_MONITOR.println(pHub->PortIndicatorsSupported, BIN);

    printProgStr(PSTR("Reserved:\t\t"));
    SERIAL_PORT_MONITOR.println(pHub->Reserved, HEX);

    printProgStr(PSTR("bPwrOn2PwrGood:\t\t"));
    SERIAL_PORT_MONITOR.println(pHub->bPwrOn2PwrGood, HEX);

    printProgStr(PSTR("bHubContrCurrent:\t"));
    SERIAL_PORT_MONITOR.println(pHub->bHubContrCurrent, HEX);

    for (uint8_t i=7; i<len; i++)
        print_hex(descrptr[i], 8);

    //for (uint8_t i=1; i<=pHub->bNbrPorts; i++)
    //    PrintHubPortStatus(&Usb, addr, i, 1);
}

byte getconfdescr( byte addr, byte conf )
{
  uint8_t buf[ BUFSIZE ];
  uint8_t* buf_ptr = buf;
  byte rcode;
  byte descr_length;
  byte descr_type;
  unsigned int total_length;
  rcode = usb.getConfDescr( addr, 0, 4, conf, buf );  //get total length
  LOBYTE( total_length ) = buf[ 2 ];
  HIBYTE( total_length ) = buf[ 3 ];
  if( total_length > 256 ) {    //check if total length is larger than buffer
    printProgStr(Conf_Trunc_str);
    total_length = 256;
  }
  rcode = usb.getConfDescr( addr, 0, total_length, conf, buf ); //get the whole descriptor
  while( buf_ptr < buf + total_length ) {  //parsing descriptors
    descr_length = *( buf_ptr );
    descr_type = *( buf_ptr + 1 );
    switch( descr_type ) {
      case( USB_DESCRIPTOR_CONFIGURATION ):
        printconfdescr( buf_ptr );
        break;
      case( USB_DESCRIPTOR_INTERFACE ):
        printintfdescr( buf_ptr );
        break;
      case( USB_DESCRIPTOR_ENDPOINT ):
        printepdescr( buf_ptr );
        break;
      case 0x21:  // HID Descriptor
        printHIDdescr( buf_ptr );
        break;
      case 0x29:
        printhubdescr( buf_ptr, addr );
        break;
      default:
        printunkdescr( buf_ptr );
        break;
        }//switch( descr_type
    buf_ptr = ( buf_ptr + descr_length );    //advance buffer pointer
  }//while( buf_ptr <=...
  return( 0 );
}
/* prints hex numbers with leading zeroes */
// copyright, Peter H Anderson, Baltimore, MD, Nov, '07
// source: http://www.phanderson.com/arduino/arduino_display.html
void print_hex(int v, int num_places)
{
  int mask=0, n, num_nibbles, digit;

  for (n=1; n<=num_places; n++) {
    mask = (mask << 1) | 0x0001;
  }
  v = v & mask; // truncate v to specified number of places

  num_nibbles = num_places / 4;
  if ((num_places % 4) != 0) {
    ++num_nibbles;
  }
  do {
    digit = ((v >> (num_nibbles-1) * 4)) & 0x0f;
    SERIAL_PORT_MONITOR.print(digit, HEX);
  }
  while(--num_nibbles);
}
/* function to print configuration descriptor */
void printconfdescr( uint8_t* descr_ptr )
{
 USB_CONFIGURATION_DESCRIPTOR* conf_ptr = ( USB_CONFIGURATION_DESCRIPTOR* )descr_ptr;
  printProgStr(Conf_Header_str);
  printProgStr(Conf_Totlen_str);
  print_hex( conf_ptr->wTotalLength, 16 );
  printProgStr(Conf_Nint_str);
  print_hex( conf_ptr->bNumInterfaces, 8 );
  printProgStr(Conf_Value_str);
  print_hex( conf_ptr->bConfigurationValue, 8 );
  printProgStr(Conf_String_str);
  print_hex( conf_ptr->iConfiguration, 8 );
  printProgStr(Conf_Attr_str);
  print_hex( conf_ptr->bmAttributes, 8 );
  printProgStr(Conf_Pwr_str);
  print_hex( conf_ptr->bMaxPower, 8 );
  return;
}
/* function to print interface descriptor */
void printintfdescr( uint8_t* descr_ptr )
{
 USB_INTERFACE_DESCRIPTOR* intf_ptr = ( USB_INTERFACE_DESCRIPTOR* )descr_ptr;
  printProgStr(Int_Header_str);
  printProgStr(Int_Number_str);
  print_hex( intf_ptr->bInterfaceNumber, 8 );
  printProgStr(Int_Alt_str);
  print_hex( intf_ptr->bAlternateSetting, 8 );
  printProgStr(Int_Endpoints_str);
  print_hex( intf_ptr->bNumEndpoints, 8 );
  printProgStr(Int_Class_str);
  print_hex( intf_ptr->bInterfaceClass, 8 );
  printProgStr(Int_Subclass_str);
  print_hex( intf_ptr->bInterfaceSubClass, 8 );
  printProgStr(Int_Protocol_str);
  print_hex( intf_ptr->bInterfaceProtocol, 8 );
  printProgStr(Int_String_str);
  print_hex( intf_ptr->iInterface, 8 );
  return;
}

/* function to print HID descriptor */
void printHIDdescr( uint8_t* descr_ptr )
{
	USB_HID_DESCRIPTOR* ep_ptr = ( USB_HID_DESCRIPTOR* )descr_ptr;

    printProgStr(PSTR("\r\n\r\nHID Descriptor:\r\n"));
    printProgStr(PSTR("HID Class Release:\t"));
	print_hex( ep_ptr->bcdHID, 16 );
    printProgStr(PSTR("\r\nCountry Code:\t\t"));
	print_hex( ep_ptr->bCountryCode, 8 );
    printProgStr(PSTR("\r\nNumb Class Descriptor:\t"));
	print_hex( ep_ptr->bNumDescriptors, 8 );
    printProgStr(PSTR("\r\nDescriptor Type:\t"));
	if( ep_ptr->bDescrType == 0x22 ) 
		printProgStr(PSTR("REPORT DESCRIPTOR"));
	else
		print_hex( ep_ptr->bDescrType, 8 );
    printProgStr(PSTR("\r\nCSize Report Descr:\t"));
	print_hex( ep_ptr->wDescriptorLength, 16 );
}

/* function to print endpoint descriptor */
void printepdescr( uint8_t* descr_ptr )
{
 USB_ENDPOINT_DESCRIPTOR* ep_ptr = ( USB_ENDPOINT_DESCRIPTOR* )descr_ptr;
  printProgStr(End_Header_str);
  printProgStr(End_Address_str);
  if( 0x80 & ep_ptr->bEndpointAddress ) printProgStr(PSTR("IN\t\t"));
  else printProgStr(PSTR("OUT\t\t"));
  print_hex( (ep_ptr->bEndpointAddress & 0xF), 8 );
  printProgStr(End_Attr_str);
  if( 0x03 & ep_ptr->bmAttributes ) printProgStr(PSTR("INTERRUPT\t"));
  else if( 0x02 & ep_ptr->bmAttributes ) printProgStr(PSTR("BULK\t"));
  else if( 0x01 & ep_ptr->bmAttributes ) printProgStr(PSTR("ISO\t"));
  print_hex( ep_ptr->bmAttributes, 8 );
  printProgStr(End_Pktsize_str);
  print_hex( ep_ptr->wMaxPacketSize, 16 );
  printProgStr(End_Interval_str);
  print_hex( ep_ptr->bInterval, 8 );

  return;
}
/*function to print unknown descriptor */
void printunkdescr( uint8_t* descr_ptr )
{
  byte length = *descr_ptr;
  byte i;
  printProgStr(Unk_Header_str);
  printProgStr(Unk_Length_str);
  print_hex( *descr_ptr, 8 );
  printProgStr(Unk_Type_str);
  print_hex( *(descr_ptr + 1 ), 8 );
  printProgStr(Unk_Contents_str);
  descr_ptr += 2;
  for( i = 0; i < length; i++ ) {
    print_hex( *descr_ptr, 8 );
    descr_ptr++;
  }
}


/* Print a string from Program Memory directly to save RAM */
void printProgStr(const prog_char str[])
{
  char c;
  if(!str) return;
  while((c = pgm_read_byte(str++)))
    SERIAL_PORT_MONITOR.print(c);
}
#endif // DESC_VALID

