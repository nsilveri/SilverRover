# SilverRover

![alt text](https://github.com/nsilveri/SilverRover/blob/main/photo_2021-06-26_22-21-18.jpg)

https://github.com/nsilveri/SilverRover/blob/main/VID-20210402-WA0045.mp4

A small Rover made with 2 DVD cases, 5 servomotors (Servo 9G), 4 rotary servomotors (Servo 9G R), 2 solar panels (5v 500mA + 5v 500mA) managed by 2 Servo 9G each and 4 hcsr-04, with the sensor front motorized with 1 of 5 9G Servos, a Raspberry Pi Camera with IR for night vision, 2 Waveshare Solar Power Management Module with 5000mA battery for each;
all this driven by Raspberry Pico which receives commands from a Raspbery Pi via USB Serial, the Raspberry Pi receives commands via SSH.





CURRENT FEATURES
1) Arduino
    - Forward. -> OK
    - Back. -> OK
    - turn right.
    - turn right with 11 different angle.  (NEW) 
    - open both solar panels. -> OK 
    - close both solar panels. -> OK
    - radar scan with all 4 HCSR-04. -> OK
    - radar scan with motorised front sensor like a radar. -> OK
    - battery recharge from solar panels. (NOW WORKS)
    
 2) Orange Pi / Raspberry part
    - installed [https://www.acmesystems.it/minicom] used to send serial comands.
    - installed [https://elinux.org/RPi-Cam-Web-Interface] to stream video.
    - installed and configurated [https://remote.it/] to connect to the SSH and Camera Website service via 4G.
    
    
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
        +===============================================================================+
        
        CURRENT:
        +======================+
        |    Nothing for now   |
        +======================+
        
        NEW:
        +======================+
        |    Nothing for now   |
        +======================+
    
