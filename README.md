# Rail Mountable MiniMap Display for Airsoft

## Concept

This project is the real life implementation of the mini-maps commonly seen in FPS shooters. The main target is to increase BSA in airsoft games by showing real-time position of friendlies and POIs.

The core part of the project is to exchange GPS positions via LR radio links (>500m).

## Microcontroller

I chose the Adafruit M0 mostly for the integrated RFM69 radio. It is quite a bit more expensive than Arduino Mini/Nano boards but to be honest I think the ATmega chips with limited RAM are gonna have hard times driving the display.

## Radio

Possible options:

* HC-12 433Mhz, TTL interface
* NRF24L01+ 2.4Ghz, SPI interface
* RFM69 433/868/915Mhz, SPI interface

## Rail Mountable Case

I've played with some designs regarding the case. The one with the angled mount clips onto the top rail so is the most straight forward one. If it's a gun with side rails the side mount is also a good option. Worst case scenario there is the one without mounts and you just duct tape the case on.

## Code

I'm no expert in embedded systems coding but here are what I found:

Biggest problem is the radio talks half-duplex, which means if two radio transmits at the same time, everything gets gobbled up and nothing gets through. So every node needs to wait for its turn to transmit, and every node need to sync with each other so it knows when is its turn.

Another problem is I suspect the radio doesn't buffer on the receiving side(not sure about this though, but let's assume the worst just to be safe). So when any other node starts transmitting, it better be listening, otherwise it won't get the message. However, the radio share the SPI bus with the screen, and drawing on screen actually can take quite a bit of time (fill the whole screen can take ~100ms). So we need to plan out when we are transmitting, when we are receiving, and when to draw stuff on the screen.

For a 2 node scenario, a possible solution work like this:

* Loop starts at 0ms;
* The node transmits for a few times, with the send time within the messages;
* In between transmissions (which are really quick), refresh part of the screen;
* The whole transmission phase should only take ~100ms;
* Wait till 500ms since loop start;
* Listen for 200ms for the other node's messages;
* Sync with the other node by comparing the received time and the ref time within the messages
* Wait till 1000ms since loop start, factor in the sync correction, and start the new loop


## Libraries

[Adafruit RadioHead fork](https://github.com/adafruit/RadioHead)

[Adafruit SSD1351](https://github.com/adafruit/Adafruit-SSD1351-library)

[Adafruit HMC5883](https://github.com/adafruit/Adafruit_HMC5883_Unified)

[TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)

## Components

[Adafruit Feather M0 Radio with RFM69 Packet Radio](https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio/overview)

[Beitian BN-880 Flight Control GPS Module Dual Module Compass](https://www.banggood.com/UBLOX-NEO-M8N-BN-880-Flight-Control-GPS-Module-Dual-Module-Compass-p-971082.html?cur_warehouse=USA)

[Waveshare 1.5inch RGB OLED Module](https://www.waveshare.com/wiki/1.5inch_RGB_OLED_Module)

[Luxtude myColors 3350mAh Small Power Bank](https://www.amazon.ca/Luxtude-myColors-Nintendo-Portable-Lipstick/dp/B07MT8J4B5/ref=sr_1_1_sspa?keywords=mini+battery+bank&qid=1558924554&s=gateway&sr=8-1-spons&psc=1)