Reqube
======

Reqube is a hardware project created with an MSP430F5328 microcontroller and various peripherals. It allows for recycling bins to be weighed, identified,  and categorized according to their contents. This allows for some pretty sweet data collection. The initial idea was to push all data online so viewers could track their recycling behavior, but please feel free to expand it to whatever it can fulfil. 

Introduction
------------

Orignially, Reqube had many facets: web application, hardware implementation and a desktop app - which would recover the collected data from the implemented device and publish said data online (via the web app). We used force sensors to try and measure weight, an RFID tag/reader combo to identify bins, an SD card/reader, a matrix keypad - plus a button and some LEDs for simple interaction - and an XBee device that would communicate with the desktop app (which also had an XBee device).  This project only encompasses the hardware implementation, though it's fairly easy to implement the other components (check out my [JBee][1] repo for a crash course on how to communicate between the desktop app and the device). 

Why a Desktop App?
------------------

You could argue that instead of a desktop application, once could just integrate a GSM module to directly upload data online. Yes. The reason it relies on serial communication to a desktop is control: the administrators wanted to have final say on all data uploads, so we went ahead and provided a way for local backup (SD card) and controlled data flow (XBees which would need to first connect, then confirm transaction in order to upload data). Still, it'd be ideal to integrate it with GSM, so that would be a nice first step.



Why an MSP430?
--------------

Although it later evolved into something more, Reqube started out as a school project, and MSP430s were sort of a requirement. All-in-all, it can be ported to an Arduino fairly easily and it provided a good basis to learn a different hardware flavor other than the current norm.

Components
----------

The following list details all the components used:

- [MSP430F5328][2]

- [FlexiForce Sensors][3]

- [SD Card/Breakout Board][4] - make sure you get an SD card under 4GB, otherwise it won't work.

- [RFID Reader][5]

- [XBee Module][6] - we used a standard Series 1.

- [Keypad][7] - practically any keypad will do.

- [Arcade Button][8] - cause, why not?







[1]:https://github.com/yoaquim/JBeeCommunicator
[2]:http://www.ti.com/product/msp430f5328
[3]:http://www.tekscan.com/flexible-force-sensors
[4]:https://www.sparkfun.com/products/11403
[5]:https://www.sparkfun.com/products/11827
[6]:https://www.adafruit.com/products/128
[7]:https://www.adafruit.com/products/419
[8]:https://www.sparkfun.com/products/9341


