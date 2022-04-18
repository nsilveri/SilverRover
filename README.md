# SilverRover

![alt text](https://github.com/nsilveri/SilverRover/blob/main/photo_2021-06-26_22-21-18.jpg)

https://github.com/nsilveri/SilverRover/blob/main/VID-20210402-WA0045.mp4

A small Rover made with 2 DVD cases, 5 servomotors (Servo 9G), 4 rotary servomotors (Servo 9G R), 4 solar panels (5v 500mA * 4) managed by 2 Servo 9G each and 5 Lidars, with the sensor front motorized with 1 of 5 9G Servos, a Raspberry Pi Camera with IR for night vision, a Waveshare Solar Power Manager(B) with 10000mA battery;
all this driven by Raspberry Pico which receives commands from a Raspbery Pi via USB Serial, the Raspberry Pi receives commands from Web UI via MQTT and sends back the status of the Rover via MQTT.
    
    
  UPGRADES:
    
  24/01/2021
  
    1) Radar alert: every loop Arduino ask distance to all HC-SR04 and if one of this distance is <20, alert is true; this function is used to STOP the rover while it is       running; this feature can be disabled pressing 5 key.
    2) Added continuos running pressing E key: the rover run forward until the user press back key (S key on keyboard) or the radar alert income, the user can use all functions( right, left,....) while the rover is running.
    3) I decided to use 2 powerbank (1 as power supply for RPi, 1 as power supply for PCA9285 (all motors)) to solve the crash problem i have only with 1 powerbank to power all.
    4) I removed the WebCam and bought an RPi Camera with IR for night vision (https://www.amazon.it/gp/product/B07DRH5Y5S/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1), now the videostream is 1920X1080, and the delay of video stream is between 0.2 and 0.5 seconds with high wifi signal
    5) I switch to from Orange Pi to Raspberry Pi 2 with an USB WiFi dongle becouse the RPi Cam is only Raspberry PI compatible.
    6) Switched from motion software to this https://elinux.org/RPi-Cam-Web-Interface for now.

  20/02/2021
   
    1) [POWER ENERGY UNIT] Added 2 Waveshare Solar Power Management Module with a 5000mA battery for each, one battery for Arduino and PCA9685 module, one battery for        Raspberry Pi 2 and Camera
            
    2) [POWER ENERGY UNIT] positioned higher
          
  27/02/2021
   
    1) I put my phone with hotspot on the rover and used the software "remoteit" to connect remotely with SSH and Cam Website, it works very well without lags (i used an S8           with 4G conection). I just bought a 4G Dongle USB, i will try with it

   26/06/2021
   
    1) Switched from Raspberry Pi 2 to Raspberry Pi Zero to reduce battery consumption
    2) Switched from Arduino Nano to Raspberry Pico
    3) Switched from HC-SR04 to front and center position with Lidar sensor, the rear sensor remains HCSR-04
    4) Now the rover uses the second core on Pico,
    core0 (to the manager commands received by Serial from Rapberry Pi Zero), core1 (manage the Lidar sensor to update the distances around the rover,
    core0 will read the distances and handle the situation based on the distances)
    5) Added the "Skirt wall" function to make the rover able to automatically steer to always keep the same distance from a side wall as it advances
    6) Added Ackermann angle
    7) Added support for 4G Wingle to connect to the rover via 4G allowing you to go anywhere without limits due to the range of the WiFi connection
    8) Added slot to insert an additional 2500mAh power bank inside the Rover to power the 4G Wingle separately
    9) Added module to read the battery voltage
    10) The PCA9685 module is now connected to I2C0, managed by core0 while the Lidar sensors are connected to I2C1 managed by core1

    I will update the photos as soon as possible

  18/04/2022
   
    1) Switched from Raspberry Pi Zero to Raspberry Pi Zero 2
    2) Added a multithread python script to manage outcoming (first thread) rover commands to Pico and read incoming (second thread) telemetry through Serial1 
    3) Added Web UI to control the rover
    4) Used MQTT protocol to make communication between Web UI and Python script
    5) Used Apache Reverse Proxy to transform service ports into urls (ex. http://RoverIpAddress:8082/stream --->  http://RoverIpAddress/camera), this make the       possibility to open only 1 port (:80 for Web UI) on Remoteit vs previously 3 ports (:80 for Web UI, :8082 for camera video, :1884 for MQTT over WS, :5002 for       microphone stream)
    6) Added audio board USB with microphone
    7) Added external WiFi antenna to expand range when used via WiFi
    8) the Python script recognise if the rover is connected to generic WiFI or LTE Wingle WiFi, if it's connected to generic Wifi it will print the signal and quality WiFi strength, if it's connected to LTE Wingle Wifi it will print the signal strength, quality and carrier name of the LTE connection.
    9) Added an INA219 to read the battery voltage and current, the INA219 is modded with a 050R resistior to make more current readable and it is connected to I2C1 and is managed by core1, on Web UI if the nattery is discharging it will show a yellow value on Web UI, instead if the battery is charging it will show a green value.
    10) Added 2 solar panels with 5v 500mA each for a total of 4 solar panels, i tried under high sunlight and it produced about 1300mA, make the battery charging when the rover is stopped and the solar panels are open. 
    11) Added shock absorber to the wheels.
    12) Added magnetic plug to plug the battery charger to the battery. Actually the is a problem, when the plug is connected, the current is stopped for some ms and the Raspberry Pi reboot itself.
    13) Modded Servo9G for wheel, now they are DC motor and are controlled by L9110S driver, the L9110S driver is connected to 2 ports of PCA9685 module and driven by a PWM signal.
    14) Switch from two Solar Power Manager (A) to one Solar Power Manager (B) (A: only 5V 500mA, B: 5V 3000mA for each port)
    15) Switch from RPi Cam Web Interface to uStreamer [https://github.com/pikvm/ustreamer]

    I will update the photos as soon as possible
    
  TO DO:

      OLD:
        +===============================================================================+
        |     - Solar battery charge capability |OK|                                    |
        |                                                                               |
        |     - Put battery position higher     |OK|                                    |
        |                                                                               |
        |     - 4G connection to control rover with a teorically infinite distance |OK| |
        |                                                                               |
        |     - Battery monitoring voltage |OK|                                         |
        |                                                                               |
        |     - Web UI to remote control the rover |OK|                                 |
        |                                                                               |
        |     - Joystick to remote control the rover via Web UI |OK|                    |
        |                                                                               |
        |     - Add shock absorber to the wheels |OK|                                   |
        |                                                                               |
        |     - Add magnetic plug to charge the battery |OK|                            |
        |                                                                               |
        |     - Add external antenna to expand the range of the rover |OK|              |
        |                                                                               |
        |     - Add audio board USB with microphone |OK|                                |
        |                                                                               |
        +===============================================================================+
        
      CURRENT:
        +======================+
        |    Nothing for now   |
        +======================+
        
      NEW:
        +============================================================+
        |    Make the battery chargable without stopping the current |
        +============================================================+
    
