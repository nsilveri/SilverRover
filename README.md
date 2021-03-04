# SilverRover
A small Rover made with 2 DVD cases, 5 servomotors (Servo 9G), 4 rotary servomotors (Servo 9G R), 2 solar panels (5v 500mA + 5v 500mA) managed by 2 Servo 9G each and 4 hcsr-04, with the sensor front motorized with 1 of 5 9G Servos, a Raspberry Pi Camera with IR for night vision, 2 power banks with 5000mAh;
all this driven by Arduino which receives commands from a Raspbery Pi via USB Serial, the Raspberry Pi receives commands via SSH





CURRENT FEATURES
1) Arduino
    - Forward -> OK
    - Back -> OK
    - turn right with 11 different angle  (NEW)
    - turn right with 11 different angle  (NEW) 
    - open both solar panels -> OK 
    - close both solar panels -> OK
    - radar scan with all 4 HCSR-04 -> OK
    - radar scan with motorised front sensor like a radar -> OK
    - battery recharge from solar panels (NOW WORKS)
    
    2) Orange Pi / Raspberry part
    - installed minicom used to send serial comands
    - installed https://elinux.org/RPi-Cam-Web-Interface to stream video
    
    24/01/2021
    1) Radar alert: every loop Arduino ask distance to all HC-SR04 and if one of this distance is <20, alert is true; this function is used to STOP the rover while it is       running; this feature can be disabled pressing 5 key.
    2) Added continuos running pressing E key: the rover run forward until the user press back key (S key on keyboard) or the radar alert income, the user can use all functions( right, left,....) while the rover is running.
    3) I decided to use 2 powerbank (1 as power supply for RPi, 1 as power supply for PCA9285 (all motors)) to solve the crash problem i have only with 1 powerbank to power all.
    4) I removed the WebCam and bought an RPi Camera with IR for night vision (https://www.amazon.it/gp/product/B07DRH5Y5S/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1), now the videostream is 1920X1080, and the delay of video stream is between 0.2 and 0.5 seconds with high wifi signal
    5) I switch to from Orange Pi to Raspberry Pi 2 with an USB WiFi dongle becouse the RPi Cam is only Raspberry PI compatible.
    6) Switched from motion software to this https://elinux.org/RPi-Cam-Web-Interface for now.
    
    20/02/2021
        - [POWER ENERGY UNIT] Added 2 Waveshare Solar Power Management Module with a 5000mA battery for each, one battery for Arduino and PCA9685 module, one battery for        Raspberry Pi 2 and Camera
        - [POWER ENERGY UNIT] positioned higher
    
    27/02/2021
        - I put my phone with hotspot on the rover and used the software "remoteit" to connect remotely with SSH and Cam Website, it works very well without lags (i used an S8           with 4G conection). I just bought a 4G Dongle USB, i will try with it
    
    
    TO DO:
        
        
        OLD:
        +=================================================+
        |     - Solar battery charge capability |OK|      |
        |                                                 |
        |    - Put battery position higher     |OK|       |
        +=================================================+
        
        CURRENT:
        +===============================================================================================+
        |     - 4G connection to control rover with a teorically infinite distance                      |
        |           I will use an USB Dongle 4G, for now i tested with phone on it with hotspot         |                    |
        +===============================================================================================+
        
        NEW:
        +=====================================================================================+
        |     - add active suspension with 8 SG90 servomotors (another PCA9685 needed)        |
        |                                                                                     |
        |     - make arduino capable to monitoring the 2 battery power                        |
        +=====================================================================================+
    
