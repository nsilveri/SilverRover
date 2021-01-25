# SilverRover
A little Rover made with 2 DVD cases, 5 servomotors (Servo 9G), 4 rotational servomotors (Servo 9G R), 2 solar panel (5v 500mA + 5v 500mA) managed by 2 Servo 9G for each and 4 hcsr-04, with the forward sensor motorized with 1 of 5 Servo 9G, one Raspberry Pi Camera with IR for night vision,a powerbank with 2 ports and 5000mAh; all this driven by Arduino that receive commands from an Raspbery Pi via USB Serial, the Raspberry Pi receives comands via SSH





CURRENT FEATURES
1) Arduino
    - Forward -> OK
    - Back -> OK
    - turn left with 2 different angle -> OK
    - turn right with 2 different angle -> OK
    - open both solar panels -> OK 
    - close both solar panels -> OK
    - radar scan with all 4 HCSR-04 -> OK
    - radar scan with motorised front sensor like a radar -> OK
    - battery recharge from solar panels -> currently not working, my powerbank doesn't support charge while it powers devices
    
    24/01/2021
    1) Radar alert: every loop Arduino ask distance to all HC-SR04 and if one of this distance is <20, alert is true; this function is used to STOP the rover while it is       running; this feature can be disabled pressing 5 key.
    2) Added continuos running pressing E key: the rover run forward until the user press back key (S key on keyboard) or the radar alert income, the user can use all functions( right, left,....) while the rover is running.
    3) I decided to use 2 powerbank (1 as power supply for RPi, 1 as power supply for PCA9285 (all motors)) to solve the crash problem i have only with 1 powerbank to power all.
    4) I removed the WebCam and bought an RPi Camera with IR for night vision (https://www.amazon.it/gp/product/B07DRH5Y5S/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1), now the videostream is 1920X1080, and the delay of video stream is between 0.2 and 0.5 seconds with high wifi signal
    5) I switch to from Orange Pi to Raspberry Pi 2 with an USB WiFi dongle becouse the RPi Cam is only Raspberry PI compatible.
    6) Switched from motion software to this https://elinux.org/RPi-Cam-Web-Interface for now.
    
    
2) Orange Pi / Raspberry part
    - installed minicom used to send serial comands
    - installed https://elinux.org/RPi-Cam-Web-Interface to stream video
    
    TO DO
      - Solar battery charge capability
      - Put battery position higher
     
    WHISHES
    - Connect a internet key to Orange Pi to go everywhere without limits :D
    
    
