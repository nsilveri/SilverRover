# SilverRover
A little Rover made with 2 DVD cases, 5 servomotors (Servo 9G), 4 rotational servomotors (Servo 9G R), 2 solar panel (5v 500mA + 5v 500mA) managed by 2 Servo 9G for each and 4 hcsr-04, with the forward sensor motorized with 1 of 5 Servo 9G, one webcam 640X480 with deleted IR-Cut, a IR-LED put on the webcam taken from an old tv remote. a powerbank with 2 ports and 5000mAh; all this driven by Arduino that receive commands from an Orange Pi via USB Serial, the Orange Pi receives comands via SSH





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
    
    
2) Orange Pi / Raspberry part
    - installed minicom used to send serial comands
    - installed motion to stream webcam video remotely
    for now to manage the Rover i use VLC to see the webcam stream and putty to connect my pc via SSH to Orange Pi and using minicom software to send arduino comands
    
    TO DO
      - Solar battery charge capability
      - Put battery position higher
     
    WHISHES
    - Connect a internet key to Orange Pi for go everywhere without limits :D
    
    
