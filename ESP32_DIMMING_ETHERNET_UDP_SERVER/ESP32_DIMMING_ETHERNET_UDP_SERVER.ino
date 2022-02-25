/*
 *   $Id: ESP32_NTP.ino,v 1.8 2019/04/04 04:48:23 gaijin Exp $
 *
 *  UDP NTP client example program.
 * 
 *  Get the time from a Network Time Protocol (NTP) time server
 *  Demonstrates use of UDP sendPacket and ReceivePacket
 * 
 *  Created:  04 Sep 2010 by Michael Margolis
 *  Modified: 09 Apr 2012 by Tom Igoe
 *  Modified: 02 Sep 2015 by Arturo Guadalupi
 *  Munged:   04 Apr 2019 by PuceBaboon (for the ESP32 with a W5500 module)
 * 
 */

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "local_config.h"	// <--- Change settings for YOUR network here.
#include <driver/dac.h>


EthernetClient client;

const int NTP_PACKET_SIZE = 48;		// NTP time stamp is in the first 48 bytes of the message.
byte packetBuffer[NTP_PACKET_SIZE];	// Buffer for both incoming and outgoing packets.

/*
 * Wiz W5500 reset function.  Change this for the specific reset
 * sequence required for your particular board or module.
 */
void WizReset() {
    Serial.print("Resetting Wiz W5500 Ethernet Board...  ");
    pinMode(RESET_P, OUTPUT);
    digitalWrite(RESET_P, HIGH);
    delay(250);
    digitalWrite(RESET_P, LOW);
    delay(50);
    digitalWrite(RESET_P, HIGH);
    delay(350);
    Serial.println("Done.");
}


/*
 * This is a crock. It's here in an effort
 * to help people debug hardware problems with
 * their W5100 ~ W5500 board setups.  It's 
 * a copy of the Ethernet library enums and
 * should, at the very least, be regenerated
 * from Ethernet.h automatically before the
 * compile starts (that's a TODO item).
 *
 */
/*
 * Print the result of the hardware status enum
 * as a string.
 * Ethernet.h currently contains these values:-
 *
 *  enum EthernetHardwareStatus {
 *  	EthernetNoHardware,
 *  	EthernetW5100,
 *  	EthernetW5200,
 *  	EthernetW5500
 *  };
 *
 */
void prt_hwval(uint8_t refval) {
    switch (refval) {
    case 0:
        Serial.println("No hardware detected.");
        break;
    case 1:
        Serial.println("WizNet W5100 detected.");
        break;
    case 2:
        Serial.println("WizNet W5200 detected.");
        break;
    case 3:
        Serial.println("WizNet W5500 detected.");
        break;
    default:
        Serial.println
            ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
    }
}


/*
 * Print the result of the ethernet connection
 * status enum as a string.
 * Ethernet.h currently contains these values:-
 *
 *  enum EthernetLinkStatus {
 *     Unknown,
 *     LinkON,
 *     LinkOFF
 *  };
 *
 */
void prt_ethval(uint8_t refval) {
    switch (refval) {
    case 0:
        Serial.println("Uknown status.");
        break;
    case 1:
        Serial.println("Link flagged as UP.");
        break;
    case 2:
        Serial.println("Link flagged as DOWN. Check cable connection.");
        break;
    default:
        Serial.println
            ("UNKNOWN - Update espnow_gw.ino to match Ethernet.h");
    }
}


#define PIN_NO 25

// the number of the LED pin
const int ledPin = 25;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

// A UDP instance to let us send and receive packets over UDP.
EthernetUDP Udp;

void setup() {

//    dac_output_enable(PIN_NO);
  
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\tUDP NTP Client v3.0\r\n");

    // Use Ethernet.init(pin) to configure the CS pin.
    Ethernet.init(5);           // GPIO5 on the ESP32.
    // Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet
    WizReset();

    /* 
     * Network configuration - all except the MAC are optional.
     *
     * IMPORTANT NOTE - The mass-produced W5500 boards do -not-
     *                  have a built-in MAC address (depite 
     *                  comments to the contrary elsewhere). You
     *                  -must- supply a MAC address here.
     */
    Serial.println("Starting ETHERNET connection...");
    Ethernet.begin(eth_MAC, eth_IP, eth_DNS, eth_GW, eth_MASK);

    delay(200);

    Serial.print("Ethernet IP is: ");
    Serial.println(Ethernet.localIP());

    /*
     * Sanity checks for W5500 and cable connection.
     */
    Serial.print("Checking connection.");
    bool rdy_flag = false;
    for (uint8_t i = 0; i <= 20; i++) {
        if ((Ethernet.hardwareStatus() == EthernetNoHardware)
            || (Ethernet.linkStatus() == LinkOFF)) {
            Serial.print(".");
            rdy_flag = false;
            delay(80);
        } else {
            rdy_flag = true;
            break;
        }
    }
    if (rdy_flag == false) {
        Serial.println
            ("\n\r\tHardware fault, or cable problem... cannot continue.");
        Serial.print("Hardware Status: ");
        prt_hwval(Ethernet.hardwareStatus());
        Serial.print("   Cable Status: ");
        prt_ethval(Ethernet.linkStatus());
        while (true) {
            delay(10);          // Halt.
        }
    } else {
        Serial.println(" OK");

        Udp.begin(localPort);
    }


  // start listening for clients
//  server.begin();

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

}


#define MODE_DIMMING 

char bCmdByNetwork = 0x00;
boolean bDimming = true;
int iDimming = 0;

unsigned long sendTimeStamp;



void analogWrite(int pin, int value)
{
//  Serial.println(value);
//  dacWrite(PIN_NO, value);

  ledcWrite(ledChannel, value);

}

void loop() {

  if(0 < Serial.available()) {
    String str = Serial.readString();
    Serial.println(str);

    char cTempData[4];
    str.toCharArray(cTempData, 4);
    if(cTempData[0] == 'd') {
      bDimming = true;
      Serial.println("Dimming");
    }
    else if(cTempData[0] == 'a') {
      
      for(int i = 0; i < 256; i += 1) {
        analogWrite(PIN_NO, i);
        delay(i < 150 ? 20 : 10);
      }
      bDimming = false;
    }
    else if(cTempData[0] == 'b') {
      for(int i = 255; 0 <= i; i -= 1) {
        analogWrite(PIN_NO, i);
        delay(i < 150 ? 20 : 10);
      }
      bDimming = false;
    }
    else {
      bDimming = false;
      
      //cTempData : 100
      int value = atoi(cTempData);
      analogWrite(PIN_NO, value);
      Serial.print("LED ");
      Serial.println(value);
    }
  }

    if(bCmdByNetwork == 'a') {
      bCmdByNetwork = 0;
         for(int i = 0; i < 256; i += 1) {
          analogWrite(PIN_NO, i);
          delay(i < 150 ? 20 : 10);
        }
    }
    else if(bCmdByNetwork == 'b') {
        for(int i = 255; 0 <= i; i -= 1) {
          analogWrite(PIN_NO, i);
          delay(i < 150 ? 20 : 10);
        }
     bCmdByNetwork = 0; 
    }

    if(bDimming == true) {
      
      iDimming += 1;
      if(512 < iDimming) iDimming = 0;

      if(iDimming == 256) {
        delay(iDimming < 150 ? 20 : 10);
      }
      else
      if(iDimming < 256) {
        analogWrite(PIN_NO, iDimming);
        delay(iDimming < 150 ? 10 : 5);
      }
      else {
        analogWrite(PIN_NO, 512 - iDimming);
        delay(512 - iDimming ? 10 : 5);
      }


    }

  delay(5);

      if (Udp.parsePacket()) {
        
        // We've received a packet, read the data from it.
        Udp.read(packetBuffer, NTP_PACKET_SIZE);        // Read the packet into the buffer.
        Serial.println((char *)packetBuffer);

        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("OK", 2);
        Udp.endPacket();
    
        if(packetBuffer[0] == 'd') {
          bDimming = true;
          Serial.println("Dimming");
        }
        else if(packetBuffer[0] == 'a') {
          
          for(int i = 0; i < 256; i += 1) {
            analogWrite(PIN_NO, i);
            delay(i < 150 ? 20 : 10);
          }
          bDimming = false;
        }
        else if(packetBuffer[0] == 'b') {
          for(int i = 255; 0 <= i; i -= 1) {
            analogWrite(PIN_NO, i);
            delay(i < 150 ? 20 : 10);
          }
          bDimming = false;
        }
        else {
          bDimming = false;
          
          //cTempData : 100
          int value = atoi((char *)packetBuffer);
          analogWrite(PIN_NO, value);
          Serial.print("LED ");
          Serial.println(value);
        }
        
      }
      else {
        
      }

  
}
