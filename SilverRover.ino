#include <Wire.h>
#include "SerialUART.h"
#include <HCSR04.h>
#include <EEPROM.h>
#include <Servo.h>
#include <i2cdetect.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2CScanner.h"
#include "Adafruit_VL53L0X.h"
I2CScanner scanner;
Servo radar_servo;
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates
#define STERZO_SINISTRA_MAX 220
#define STERZO_DRITTO 330//328
#define STERZO_DESTRA_MAX 440
#define DRITTO STERZO_DRITTO
#define CHIUSO 0
#define APERTO 1
#define META 2
#define ATTIVATO 1
#define DISATTIVATO 0
#define reset_button 22

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33
#define LOX5_ADDRESS 0x34
#define LOX6_ADDRESS 0x35
//#define LOX4_ADDRESS 0x33
int sensor1,sensor2, sensor3, sensor4, sensor5, sensor6;
#define SHT_LOX1 11
#define SHT_LOX2 12
#define SHT_LOX3 13
#define SHT_LOX4 10
#define SHT_LOX5 9
#define SHT_LOX6 8
//#define SHT_LOX4 
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;
VL53L0X_RangingMeasurementData_t measure6;

//----------------------------------------------------------------------------------------------------------------------
//#include "FaBoPWM_PCA9685.h"
//#include <HCSR04.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();//----------------------------------------------------------------------------------------------------
//HCSR04 hc(5,new int[4]{3,2,4,6},4);
//FaBoPWM faboPWM;
const uint16_t STOP = 1500;
const uint16_t FORWARD = 2000;
const uint16_t BACK = 1000;
const uint16_t run_delay = 2100; //2100 percorre circa 50cm
uint8_t POSIZIONE_PANNELLO_DX = CHIUSO;
uint8_t POSIZIONE_PANNELLO_SX = CHIUSO;
uint8_t peso_destra = 0;
uint8_t peso_sinistra = 0;
uint8_t stato_precedente = CHIUSO;
const uint8_t panel_speed = 5;
//uint16_t radar_scan[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//uint16_t radar_scan_motor[48] = {110, 120, 130, 140, 150, 160, 170, 180 , 190, 200 , 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580};
uint16_t radar_scan_180[48];
uint8_t webcam_sterzo = ATTIVATO;
uint8_t radar_sterzo = DISATTIVATO;
uint8_t pca_off = ATTIVATO;
uint16_t posizione_sterzo = STERZO_DRITTO;
uint16_t webcam_sopra_sotto = 250;
uint16_t webcam_sinistra_destra = 268;
uint16_t webcam_sopra_sotto_effettuato = DISATTIVATO;
uint8_t risparmio_energetico = DISATTIVATO;
uint8_t sterzo_automatico = DISATTIVATO;
uint16_t radar_scan_posizion = DRITTO;
uint8_t radar_direction = 0;
uint8_t avviso_ant = 0;
uint8_t avviso_dx_ant_45 = 0;
uint8_t avviso_sx_ant_45 = 0;
uint8_t avviso_dx_center = 0;
uint8_t avviso_sx_center = 0;
uint8_t motor_speed_id = 10;

uint16_t radar_delay = 100;
//int array_valore_sterzo[10] = {55,
uint16_t precisione_sterzo = 7;
uint16_t valore_sterzata = 11;//precisione_sterzo;
int8_t webcam_X = 0;
int8_t webcam_Y = 0;
uint8_t ruote_sterzanti_post = ATTIVATO;
uint8_t ruote_sterzanti_ant = ATTIVATO;
uint16_t radar_position = 360;
uint16_t radar_dx_ant_position = 110;
uint16_t radar_sx_ant_position = 580;
uint16_t panel_position_dx = 0;
uint16_t panel_position_sx = 0;
uint8_t panels_security_distance = 0;
uint8_t security_distance = 250;

int8_t avanzamento_c = DISATTIVATO;
int8_t anticollisione = ATTIVATO;

int radar_dx_ant = 0;
int radar_dx_ant_aux = 0;
int radar_dx_center = 0;
int radar_dx_ant_45 = 0;
int radar_dx_ant_45_aux = 0;

int radar_sx_ant = 0;
int radar_sx_ant_aux = 0;
int radar_sx_center = 0;
int radar_sx_ant_45 = 0;
int radar_sx_ant_45_aux = 0;
int radar_ant = 0;
int radar_ant_aux = 0;

uint16_t radar_motor_speed = 400;

int16_t motor_speed = 0;
int16_t ant_sx_speed = 0;
int16_t ant_dx_speed = 0;
int16_t post_dx_speed = 0;
int16_t post_sx_speed = 0;

int16_t trimmer = 0;
int16_t limite_sinistro = STERZO_SINISTRA_MAX + trimmer;
int16_t limite_destro = STERZO_DESTRA_MAX + trimmer;
int16_t centro = STERZO_DRITTO + trimmer;

int radar_post = 0;
uint16_t task = radar_motor_speed;
//VoltMeter==============================
int analogInput_1 = analogRead(27);
int analogInput_2 = analogRead(26);

float Vout_1 = 0.00;
float Vin_1 = 0.00;

float Vout_2 = 0.00;
float Vin_2 = 0.00;

float input_voltage_1 = 0.0;
float input_voltage_2 = 0.0;

float temp_1 = 0.0;
float temp_2 = 0.0;
float R1_1 = 30000.00; // resistance of R1 (30K)
float R2_1 = 7500.00; // resistance of R2 (7.5K)

float R1_2 = 30000.00; // resistance of R1 (30K)
float R2_2 = 7500.00; // resistance of R2 (7.5K)

int val_1 = 0;
int val_2 = 0;

uint32_t tempo_funzionamento = millis();
uint32_t inizio_conteggio_distanza = 0;
uint32_t fine_conteggio_distanza = 0;
float distanza_percorsa;

int post_sx_speed_2 = 0;

bool costeggia_muro_enabled = false;

const int chipSelect = 10;

uint8_t diagnostic = DISATTIVATO;

void setID() {
///Robojax.com code see video https://youtu.be/0glBk917HPg  
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);    
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);     
  
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);    
  digitalWrite(SHT_LOX5, HIGH);
  digitalWrite(SHT_LOX6, HIGH);
  delay(10);

  // activating LOX1 and reseting other sensors
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);    
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot 1 VL53L0X"));
    //while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot 2 VL53L0X"));
    //while(1);
  }
  
  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot 3 VL53L0X"));
    //while(1);
  }  
  
   //activating LOX4
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  //initing LOX4
  if(!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot 4 VL53L0X"));
    //while(1);
  }  

  //activating LOX5
  digitalWrite(SHT_LOX5, HIGH);
  delay(10);

  //initing LOX5
  if(!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("Failed to boot 5 VL53L0X"));
    //while(1);
  }  

  //activating LOX6
  digitalWrite(SHT_LOX6, HIGH);
  delay(10);

  //initing LOX6
  if(!lox6.begin(LOX6_ADDRESS)) {
    Serial.println(F("Failed to boot 6 VL53L0X"));
    //while(1);
  }
}

void read_five_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
  lox5.rangingTest(&measure5, false); // pass in 'true' to get debug data printout!
  lox6.rangingTest(&measure6, false); // pass in 'true' to get debug data printout!
    
  // print sensor one reading
  if(measure1.RangeStatus != 4 && measure1.RangeMilliMeter < 2000) {     // if not out of range
    radar_sx_ant_45 = measure1.RangeMilliMeter;    
  } else {
    radar_sx_ant_45 = 2000;
  }
  if (diagnostic == ATTIVATO)
  {
      Serial.print("1: ");
      Serial.print(" ");
      Serial.print(radar_sx_ant_45);
      Serial.print("mm");
      Serial.print(" ");
  }

  // print sensor two reading
  
  if(measure2.RangeStatus != 4 && measure2.RangeMilliMeter < 2000) {
    radar_ant = measure2.RangeMilliMeter;
    
  } else {
    radar_ant = 2000;
  }
  if (diagnostic == ATTIVATO)
  {
      Serial.print("2: ");
      Serial.print(radar_ant);
      Serial.print("mm");
  }

   //Serial.print(" ");

   ///Robojax.com code see video https://youtu.be/0glBk917HPg
  // print sensor three reading
  
  if(measure3.RangeStatus != 4 && measure3.RangeMilliMeter < 2000) {
    radar_dx_ant_45 = measure3.RangeMilliMeter;
    
  } else {
    radar_dx_ant_45 = 2000;
  }
  if (diagnostic == ATTIVATO)
  {
    Serial.print("3: ");
    Serial.print(radar_dx_ant_45);
    Serial.print("mm");
  }
  
  
  //Serial.print(" ");

//   print sensor four reading
  
  if(measure4.RangeStatus != 4 && measure4.RangeMilliMeter < 2000) {
    radar_dx_center = measure4.RangeMilliMeter;
    
  } else {
    radar_dx_center = 2000;
  }  
  if (diagnostic == ATTIVATO)
  {
    Serial.print("4: ");
    Serial.print(radar_dx_center);
    Serial.print("mm");
  }
// print sensor 5 reading
  
  if(measure5.RangeStatus != 4 && measure5.RangeMilliMeter < 2000) {
    radar_sx_center = measure5.RangeMilliMeter;
    
  } else {
    radar_sx_center = 2000;
  }
  if (diagnostic == ATTIVATO)
  {
    Serial.print("5: ");
    Serial.print(radar_sx_center);
    Serial.print("mm");
    Serial.println();
  }
}


void setup1() {

  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  Serial.begin(9600);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  pinMode(SHT_LOX5, OUTPUT);
  pinMode(SHT_LOX6, OUTPUT);  
  pinMode(reset_button, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);
  setID();
  radar_servo.attach(15);
  Serial1.println();
  delay(2000);
  Serial1.println("Setup CORE 1 [OK]");
}

void rad_mot()
{
  int pos;

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    radar_servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    radar_servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void loop1() {
  read_five_sensors();
  delay(300);
  //anticollision();
  //rad_mot();
}


void setup() {
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(9600);
  EEPROM.begin(512);
  scanner.Init();
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  //EEPROM.write(10,0);
  Serial1.println( F("Welcome to the SilverRover"));
  Serial1.println( F("Inserisci il comando da inviare al Rover."));
  Serial1.println( F("scrivi 'help' per vedere i comandi disponibili"));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  panels_positions_data_load();
  load_trimmer();
  load_odo_eeprom();
  sterzo_dritto();
  ruote_sterzanti_post = DISATTIVATO;
  //pca_on_off();//disattiva pca
  //VoltMeter==============================
  pinMode(analogInput_1, INPUT); //assigning the input port
  pinMode(analogInput_2, INPUT);
  //=========================================
}

void load_trimmer()
{
  int trim_ = 0;
  EEPROM.get(10, trim_);
  if(trim_ >= -10 && trim_ <= 10)
  {
  trimmer = trim_;
  if (diagnostic == ATTIVATO)
    {
      Serial1.print(F("Trimmer caricato: "));
      Serial1.println(trimmer);
    }
  }else trimmer = 0;
}

void panels_positions_data_load()
{
  int x = EEPROM.read(0);
  int y = EEPROM.read(1);
  //trimmer = EEPROM.read(10);
  if (x == 0 || x == 1 && y == 0 || y == 1)
  {
    POSIZIONE_PANNELLO_DX = EEPROM.read(0);
    POSIZIONE_PANNELLO_SX = EEPROM.read(1);
    if(POSIZIONE_PANNELLO_DX == APERTO || POSIZIONE_PANNELLO_SX == APERTO)
    {
      panels_security_distance = 150;
    }else if(POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO)
    {
      panels_security_distance = 0;
    }
    if (diagnostic == ATTIVATO)
    {
      Serial1.print(F("DX: "));
      Serial1.println(POSIZIONE_PANNELLO_DX);
      Serial1.print(F("SX: "));
      Serial1.println(POSIZIONE_PANNELLO_SX);
    }
  }
}

void panels_positions_data_save()
{
  EEPROM.write(0, POSIZIONE_PANNELLO_DX);
  EEPROM.write(1, POSIZIONE_PANNELLO_SX);
  if (EEPROM.commit())
  {
    if (diagnostic == ATTIVATO)
    {
      Serial1.println("EEPROM successfully committed");
    }
  } else {
    Serial1.println("ERROR! EEPROM commit failed");
  }
}

//void write_file_sd_card()
//{
//  myFile = SD.open("test.txt", FILE_WRITE);
//  if (myFile) {
//    //Serial1.print("Writing to test.txt...");
//    myFile.print(distanza_percorsa);
//    // close the file:
//    myFile.close();
//    Serial1.println("done.");
//  } else {
//    // if the file didn't open, print an error:
//    Serial1.println("error opening test.txt");
//  }
//}

//void carica_distanza_percorsa_sd_card()
//{
//  myFile = SD.open("test.txt");
//  if (myFile) {
//    Serial1.println("test.txt:");
//
//    // read from the file until there's nothing else in it:
//    while (myFile.available()) {
//      Serial1.write(myFile.read());
//      distanza_percorsa = myFile.read();
//    }
//    // close the file:
//    myFile.close();
//  } else {
//    // if the file didn't open, print an error:
//    Serial1.println("error opening test.txt");
//  }
//}

//void sd_card_init() {
//  //Serial1.print("Initializing SD card...");
//
//  if (!SD.begin(10)) {
//    Serial1.println("initialization SD Card failed!");
//    while (1);
//  }
//  //Serial1.println("initialization done.");
//
//
//  // re-open the file for reading:
//
//  //sd_card_info();
//}

void pca_on_off()
{
  if (pca_off == DISATTIVATO)
  {
    pca_off = ATTIVATO;
    digitalWrite(2, HIGH);
    Serial1.println(F("Motori disattivati"));
  } else if (pca_off == ATTIVATO)
  {
    pca_off = DISATTIVATO;
    digitalWrite(2, LOW);
    Serial1.println(F("Motori attivati"));
  }
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial1.print(pulselength); Serial1.println( F(" us per period"));
  pulselength /= 4096;  // 12 bits of resolution
  Serial1.print(pulselength); Serial1.println( F(" us per bit"));
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial1.println(pulse);
}


void forward(uint16_t comand)
{
  if (comand == STOP)
  {
    fine_conteggio_distanza = millis();
    pwm.writeMicroseconds(4, STOP);//-30 //ANT_SX
    pwm.writeMicroseconds(5, STOP);//-50 //ANT_DX
    pwm.writeMicroseconds(6, STOP);//-150q //POST_DX
    pwm.writeMicroseconds(7, STOP - 50);//POST_SX -> SCOLLEGATO
    calcolo_distanza_percorsa();
    if(avanzamento_c == ATTIVATO)
         avanzamento_c = DISATTIVATO;
  } else if (comand == FORWARD)
  {
    inizio_conteggio_distanza = millis();
    pwm.writeMicroseconds(4, FORWARD - motor_speed - ant_sx_speed);//ANT_SX
    pwm.writeMicroseconds(5, BACK + motor_speed + ant_dx_speed);//ANT_DX
    pwm.writeMicroseconds(6, BACK + motor_speed + post_dx_speed);//POST_DX
    pwm.writeMicroseconds(7, FORWARD - motor_speed - post_sx_speed - 465);//POST_SX -> SCOLLEGATO
  } else if (comand == BACK)
  {
    inizio_conteggio_distanza = millis();
    pwm.writeMicroseconds(4, BACK + motor_speed + ant_sx_speed);//ANT_SX
    pwm.writeMicroseconds(5, FORWARD - motor_speed - ant_dx_speed);//ANT_DX
    pwm.writeMicroseconds(6, FORWARD - motor_speed - post_dx_speed);//POST_DX
    pwm.writeMicroseconds(7, BACK + motor_speed - post_sx_speed + 355);//POST_SX -> SCOLLEGATO
  }
}

bool zona_radar(uint16_t value)
{
  if (value > 230 && value < 400)
    return true;
  return false;
}

void webcam_control()
{
  uint16_t webcam_sinistra_destra_l = webcam_sinistra_destra;
  uint16_t webcam_sopra_sotto_l = webcam_sopra_sotto;
  if (zona_radar(webcam_sinistra_destra) && webcam_sopra_sotto > 435)
    webcam_sopra_sotto_l = 435;
  pwm.setPWM(15, 0, webcam_sinistra_destra_l);
  pwm.setPWM(14, 0, webcam_sopra_sotto_l);
}

void _webcam_sterzo(uint16_t pulselen)
{
  webcam_sinistra_destra = pulselen - 60;
  webcam_sopra_sotto = 315;
  pwm.setPWM(15, 0, webcam_sinistra_destra);
  pwm.setPWM(14, 0, webcam_sopra_sotto);
}

void sterzo_anteriore(uint16_t pulselen)
{
  pulselen = pulselen + trimmer;
  int p_p = map(pulselen, limite_sinistro, limite_destro, 264 , 397);
//  int ant_sx_int = map(pulselen, limite_sinistro, centro, -270, 0);
//  int ant_dx_int = map(pulselen -11, centro, limite_destro, 0, 270);
//  int post_sx_int = map(pulselen - 11, limite_sinistro, centro,  65, 0);
//  int post_sx_ext = map(pulselen - 11, centro, limite_destro, 0, 20);
//  int post_dx_int = map(pulselen - 11, centro, limite_destro, 0, 380);
//  int post_dx_ext = map(pulselen - 11, limite_sinistro, centro, -200, 0);
//  int post_sx_int = map(pulselen - 11, 220, 330, 65, 0);
//  int post_sx_ext = map(pulselen - 11, 330, 440, 0, 20);
  if (pulselen > centro)
  {
    pwm.setPWM(0, 0, pulselen - 8); //-5
    pwm.setPWM(1, 0, p_p + 68);
    //============================================
    ant_dx_speed = map(pulselen, centro, limite_destro, 0, 270);//ant_dx_int;
    post_dx_speed = map(pulselen, centro, limite_destro, 0, 380);//post_dx_int;
    post_sx_speed = map(pulselen, centro, limite_destro, 0, 20);//post_sx_ext;
    if(avanzamento_c == ATTIVATO){
           forward(FORWARD);
        }
    
    //============================================
  } else if (pulselen < centro)
  {
    pwm.setPWM(0, 0, p_p - 8); //-5
    pwm.setPWM(1, 0, pulselen + 68);
    //============================================
    ant_sx_speed =  map(pulselen, limite_sinistro, centro, 270, 0);//ant_sx_int;
    post_sx_speed = map(pulselen, limite_sinistro, centro,  30, 0);//post_sx_int;//post_int;
    post_dx_speed = map(pulselen, limite_sinistro, centro, 200, 0);//post_dx_ext;
    if(avanzamento_c == ATTIVATO){
           forward(FORWARD);
        }
    //============================================
  } else {
    pwm.setPWM(0, 0, pulselen - 8); //-5
    pwm.setPWM(1, 0, pulselen + 68);
    ant_sx_speed = 0;
    ant_dx_speed = 0;//ant_int;
    post_dx_speed = 0;//post_int;
    post_sx_speed = 0;//post_ext;
    if(avanzamento_c == ATTIVATO){
           forward(FORWARD);
        }
  }
  if(diagnostic == ATTIVATO)
  {
      Serial1.print("ANT SX SPEED: ");
      Serial1.println(ant_sx_speed);
      Serial1.print("POST SX SPEED (Modded servo): ");
      Serial1.println(post_sx_speed);
      Serial1.print("ANT DX SPEED: ");
      Serial1.println(ant_dx_speed);
      Serial1.print("POST DX SPEED: ");
      Serial1.println(post_dx_speed);
  }
}


void sterzo_posteriore(uint16_t pulselen)
{
  //  if(webcam_sterzo == ATTIVATO)
  //    _webcam_sterzo(pulselen);
  //  if(radar_sterzo ==ATTIVATO)
  //    radar_motor(pulselen);
  //    delay(200);
  pwm.setPWM(2, 0, pulselen + 38); //40
  pwm.setPWM(3, 0, pulselen - 5);

}

void sterzo(uint16_t angolo_serzata)
{
  uint16_t pulselen_ant = angolo_serzata;
  uint16_t pulselen_post = pulselen_ant + 25;
  int calc_angle = STERZO_DRITTO - pulselen_ant;
  uint16_t angle_post = STERZO_DRITTO + calc_angle + 25;
  if (ruote_sterzanti_ant == ATTIVATO);
  sterzo_anteriore(pulselen_ant);
  if (ruote_sterzanti_post == ATTIVATO)
    sterzo_posteriore(angle_post);
  if (webcam_sterzo == ATTIVATO)
    _webcam_sterzo(angle_post);
  if (radar_sterzo == ATTIVATO)
    radar_motor(angle_post);
  //delay(200);
}


void sterzo_dritto()
{
  if (posizione_sterzo != DRITTO)
  {
    if (posizione_sterzo < DRITTO) {
      for (uint16_t i = posizione_sterzo; i <= DRITTO; i++)
      {
        posizione_sterzo = i;
        sterzo(posizione_sterzo);
        delay(2);

      }
    } else if (posizione_sterzo > DRITTO)
    {
      for (uint16_t i = posizione_sterzo; i >= DRITTO; i--)
      {
        posizione_sterzo = i;
        sterzo(posizione_sterzo);
        delay(2);
      }
    }


  }
  posizione_sterzo = DRITTO;
  sterzo(STERZO_DRITTO);
}


void panel_dx_da_meta_a_chiuso()
{
  if (POSIZIONE_PANNELLO_DX == META)
  {
    for (uint16_t i = 300; i < 520; i++)
    {
      pwm.setPWM(8, 0, i/*i*/);
      panel_position_dx = i;
      pwm.setPWM(9, 0, 328 - i/*i*/);
      delay(panel_speed);
    }
    pwm.setPWM(8, 0, 0/*i*/);
    POSIZIONE_PANNELLO_DX = 0;
  }
}

void panel_dx_da_chiuso_a_meta()
{
  if (POSIZIONE_PANNELLO_DX == 0)
  {
    for (uint16_t i = 520; i > 300; i--) {
      pwm.setPWM(8, 0, i/*i*/);
      panel_position_dx = i;
      pwm.setPWM(9, 0, i/*i*/);
      delay(panel_speed);
    }
    pwm.setPWM(8, 0, 0/*i*/);
    POSIZIONE_PANNELLO_DX = META;
  }
}

void panel_dx_da_meta_a_aperto()
{
  if (POSIZIONE_PANNELLO_DX == META)
  {
    for (uint16_t i = 300; i > 90; i--)
    {
      pwm.setPWM(8, 0, i/*i*/);
      panel_position_dx = i;
      pwm.setPWM(9, 0, i/*i*/);
      delay(panel_speed);
    }
    pwm.setPWM(8, 0, 0/*i*/);
    POSIZIONE_PANNELLO_DX = APERTO ;
  }
}

void panel_dx_da_aperto_a_meta()
{
  if (POSIZIONE_PANNELLO_DX == APERTO )
  {
    for (uint16_t i = 90; i < 300; i++)
    {
      pwm.setPWM(8, 0, i/*i*/);
      panel_position_dx = i;
      pwm.setPWM(9, 0, i/*i*/);
      delay(panel_speed);
    }
    pwm.setPWM(8, 0, 0/*i*/);
    POSIZIONE_PANNELLO_DX = META;
  }
}

void panel_sx_da_meta_a_chiuso()
{
  if (POSIZIONE_PANNELLO_SX == META)
  {
    for (uint16_t i = 450; i > 190; i--)
    {
      //pwm.setPWM(10, 0, 328 + i/*i*/);
      pwm.setPWM(10, 0, i/*i*/);
      panel_position_sx = i;
      delay(panel_speed);
    }
    pwm.setPWM(10, 0, 0/*328+i/*i*/);
    POSIZIONE_PANNELLO_SX = 0;
  }
}

void panel_sx_da_chiuso_a_meta()
{
  if (POSIZIONE_PANNELLO_SX == 0)
  {
    for (uint16_t i = 190; i < 450; i++)
    {
      pwm.setPWM(10, 0, i/*328+i/*i*/);
      panel_position_sx = i;
      //pwm.setPWM(11, 0, i/*328-i/*i*/);
      delay(panel_speed);
    }
    POSIZIONE_PANNELLO_SX = META;
  }
}

void panel_sx_da_meta_a_aperto()
{
  if (POSIZIONE_PANNELLO_SX == META)
  {
    for (uint16_t i = 450; i < 640; i++/*int i=0;i<240;i++*/)
    {
      pwm.setPWM(10, 0, i/*328+i/*i*/);
      panel_position_sx = i;
      //pwm.setPWM(11, 0, i/*328+i/*i*/);
      delay(panel_speed);
    }
  }
  pwm.setPWM(10, 0, 0/*328+i/*i*/);
  POSIZIONE_PANNELLO_SX = APERTO ;
}

void panel_sx_da_aperto_a_meta()
{
  if (POSIZIONE_PANNELLO_SX == APERTO )
  {
    for (uint16_t i = 640; i > 450; i--)
    {
      //pwm.setPWM(10, 0, 328 - i/*i*/);
      pwm.setPWM(10, 0, i/*i*/);
      panel_position_sx = i;
      delay(panel_speed);
    }
    POSIZIONE_PANNELLO_SX = META;
  }
}
void spegni_motori_pannelli()
{
  pwm.setPWM(8, 0, 0/*i*/);
  pwm.setPWM(10, 0, 0/*i*/);
}
void panel_sx_dx_da_meta_a_aperto()
{
  if (POSIZIONE_PANNELLO_SX == META && POSIZIONE_PANNELLO_DX == META)
  {
    //panel_position_dx = 300;
    //panel_position_sx = 450;
    for (uint8_t i = 0; i < 210; i++)
    {
      if (panel_position_dx > 90)
      {
        pwm.setPWM(8, 0, 300 - i/*i*/);
        panel_position_dx = 300 - i;
      }
      if (panel_position_sx < 640)
      {
        pwm.setPWM(10, 0, 430 + i/*i*/);
        panel_position_sx = 430 + i;
      }
      delay(panel_speed);
    }
    spegni_motori_pannelli();
    POSIZIONE_PANNELLO_SX = APERTO ;
    POSIZIONE_PANNELLO_DX = APERTO ;
  }
}

void panel_sx_dx_da_apero_a_meta()
{
  if (POSIZIONE_PANNELLO_SX == APERTO && POSIZIONE_PANNELLO_DX == APERTO )
  {
    for (uint8_t i = 210; i > 0; i--)
    {
      if (panel_position_dx < 300)
      {
        pwm.setPWM(8, 0, 300 - i/*i*/);
        panel_position_dx = 300 - i;
      }
      if (panel_position_sx > 449)
      {
        pwm.setPWM(10, 0, 450 + i/*i*/);
        panel_position_sx = 450 + i;
      }
      delay(panel_speed);
    }
    POSIZIONE_PANNELLO_SX = META;
    POSIZIONE_PANNELLO_DX = META;
  }
}

void open_panels()
{
  if (POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO && peso_sinistra == DISATTIVATO && peso_destra == DISATTIVATO)
  {
    panel_dx_da_chiuso_a_meta();
    Serial1.println( F("Pannello destro aperto a metà"));
    panel_sx_da_chiuso_a_meta();
    Serial1.println( F("Pannello sinistro aperto a metà"));
    panel_sx_dx_da_meta_a_aperto();
    //          POSIZIONE_PANNELLO_SX = 0;
    Serial1.println( F("Entrambi i pannelli aperti"));
    POSIZIONE_PANNELLO_DX = APERTO ;
    POSIZIONE_PANNELLO_SX = APERTO ;
    stato_precedente = APERTO ;
  } else if (peso_sinistra == ATTIVATO)
  {
    stato_precedente = 1;
    peso_a_sinistra();
  } else if (peso_destra == ATTIVATO)
  {
    stato_precedente = 1;
    peso_a_destra();
  }
  panels_positions_data_save();
  //Serial1.println( F("azioni Pannello sx effettuate"));
}

void close_panels()
{
  if (POSIZIONE_PANNELLO_DX == APERTO && POSIZIONE_PANNELLO_SX == APERTO && peso_sinistra == DISATTIVATO && peso_destra == DISATTIVATO)
      {
            panel_sx_dx_da_apero_a_meta();
            Serial1.println( F("Pannello sinistro aperto a metà"));
            Serial1.println( F("Pannello destro aperto a metà"));
            panel_sx_da_meta_a_chiuso();
            panel_dx_da_meta_a_chiuso();
        
            //          POSIZIONE_PANNELLO_SX = 0;
            Serial1.println( F("Entrambi i pannelli chiusi"));
            Serial1.println( F("azioni Pannello sx effettuate"));
            POSIZIONE_PANNELLO_DX = 0;
            POSIZIONE_PANNELLO_SX = 0;
            stato_precedente = 0;
      } else if (peso_sinistra == ATTIVATO)
      {
            stato_precedente = 0;
            peso_a_sinistra();
      } else if (peso_destra == ATTIVATO)
      {
            stato_precedente = 0;
            peso_a_destra();
      }
   panels_positions_data_save();
}

void left_panel()
{
  if (POSIZIONE_PANNELLO_SX == 0)
  {
    panel_sx_da_chiuso_a_meta();
    Serial1.println( F("Pannello sinistro aperto a metà"));
    panel_sx_da_meta_a_aperto();
    //POSIZIONE_PANNELLO_SX = APERTO ;
    Serial1.println( F("Pannello sinistro aperto completamente"));
  } else if (POSIZIONE_PANNELLO_SX == APERTO )
  {
    panel_sx_da_aperto_a_meta();
    Serial1.println( F("Pannello sinistro aperto a metà"));
    //panel_dx_da_chiuso_a_meta();

    panel_sx_da_meta_a_chiuso();
    //POSIZIONE_PANNELLO_SX = 0;
    if (POSIZIONE_PANNELLO_SX == 0)
      Serial1.println( F("Pannello sinistro 0"));
  } else if (POSIZIONE_PANNELLO_SX == 2)
  {
    panel_sx_da_meta_a_chiuso();
    //POSIZIONE_PANNELLO_SX = 0;
    if (POSIZIONE_PANNELLO_SX == 0)
      Serial1.println( F("Pannello sinistro 0"));
  }
  Serial1.println( F("Azioni pannello sx effettuate"));
}

void right_panel()
{
  if (POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX != 0)
  {
    panel_dx_da_chiuso_a_meta();
    Serial1.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_aperto();
    //POSIZIONE_PANNELLO_DX = APERTO ;
    Serial1.println( F("Pannello destro completamente aperto"));
  } else if (POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX != 0)
  {
    panel_dx_da_aperto_a_meta();
    Serial1.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_chiuso();
    if (POSIZIONE_PANNELLO_DX == 0)
      Serial1.println( F("Pannello destro 0"));
  } else if (POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX == 0)
  {
    panel_sx_da_chiuso_a_meta();
    panel_dx_da_aperto_a_meta();
    Serial1.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_chiuso();
    panel_sx_da_meta_a_chiuso();
    if (POSIZIONE_PANNELLO_DX == 0)
      Serial1.println( F("Pannello destro 0"));
  } else if (POSIZIONE_PANNELLO_DX == 2 && POSIZIONE_PANNELLO_SX != 0)
  {
    //panel_dx_da_aperto_a_meta();
    Serial1.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_chiuso();
    if (POSIZIONE_PANNELLO_DX == 0)
      Serial1.println( F("Pannello destro 0"));
  }
  Serial1.println( F("azioni Pannello dx effettuate"));
}


void radar_motor(uint16_t angle_motor)
{
  pwm.setPWM(12, 0, angle_motor + 15);
}

/**/void peso_a_sinistra()
{
  Serial1.println( F("1"));
  if (peso_sinistra == 0)
  {
    //Serial1.println( F("2"));
    if (POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO)
    {
      panel_sx_da_chiuso_a_meta();
      panel_sx_da_meta_a_aperto();
      peso_sinistra = ATTIVATO;
      //Serial1.println( F("3"));
    } else if (POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX == APERTO)
    {
      //Serial1.println( F("4"));
      panel_dx_da_aperto_a_meta();
      panel_dx_da_meta_a_chiuso();
      peso_sinistra = ATTIVATO;
    }

  } else if (peso_sinistra == ATTIVATO)
  {
    //Serial1.println( F("2"));
    if (stato_precedente == 0)
    {
      panel_sx_da_aperto_a_meta();
      panel_sx_da_meta_a_chiuso();
      peso_sinistra = DISATTIVATO;
      //Serial1.println( F("3"));
    } else if (stato_precedente == APERTO )
    {
      //Serial1.println( F("4"));
      panel_dx_da_chiuso_a_meta();
      panel_dx_da_meta_a_aperto();
      peso_sinistra = DISATTIVATO;
    }
  }
  //Serial1.println( F("5"));
}//*/


/**/void peso_a_destra() // muove i pannelli solari in modo da ottenere maggior peso verso destra per evitare eventuali ribaltamenti dovuti alla pendenza
{
  if (peso_destra == DISATTIVATO) //controllo per verificare che la funzione non sia già attiva
  {
    if (POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO)
    {
      panel_sx_da_chiuso_a_meta();
      panel_dx_da_chiuso_a_meta();
      panel_dx_da_meta_a_aperto();
      panel_sx_da_meta_a_chiuso();
      peso_destra = ATTIVATO;
    } else if (POSIZIONE_PANNELLO_DX == APERTO && POSIZIONE_PANNELLO_SX == APERTO)
    {
      panel_sx_da_aperto_a_meta();
      panel_sx_da_meta_a_chiuso();
      peso_destra = ATTIVATO;
    }

  } else if (peso_destra == ATTIVATO)
  {
    Serial1.println( F("peso destro attivo 2"));
    if (stato_precedente == CHIUSO)
    {
      //Serial1.println(stato_precedente);
      panel_sx_da_chiuso_a_meta();
      panel_dx_da_aperto_a_meta();
      panel_dx_da_meta_a_chiuso();
      panel_sx_da_meta_a_chiuso();
      Serial1.println( F("3"));
      peso_destra = DISATTIVATO;
    } else if (stato_precedente == APERTO)
    {
      Serial1.println(stato_precedente);
      panel_sx_da_chiuso_a_meta();
      panel_sx_da_meta_a_aperto();
      peso_destra = DISATTIVATO;
    }
    //peso_sinistra = DISATTIVATO;
  }
}

void load_odo_eeprom()
{
  //EEPROM.readFloat(2);
  float value = 0;
  EEPROM.get(3, value);
  distanza_percorsa = value;
  //  Serial1.print(F("DISTANZA CARICATA: "));
  //    Serial1.println(distanza_percorsa);
  if (diagnostic == ATTIVATO)
  {
    Serial1.print(F("DISTANZA CARICATA: "));
    Serial1.println(distanza_percorsa);
  }

}

void save_odo_eeprom(float distanza_percorsa_odo)
{
  EEPROM.put(3, distanza_percorsa_odo);
  if (diagnostic == ATTIVATO)
  {
    Serial1.println(F("DISTANZA SALVATA!"));
  }
}

void calcolo_distanza_percorsa()
{
  float aux_time;
  float aux_distance;
  aux_time = fine_conteggio_distanza - inizio_conteggio_distanza;
  aux_distance = aux_time / 52.5; //distanza percorsa in centimetri
  distanza_percorsa = distanza_percorsa + aux_distance; //distanza percorsa in centimetri totale
  save_odo_eeprom(distanza_percorsa);//write_file_sd_card(); // salva distanza percorsa su SD
}


void avanzamento_continuo()
{
  avanzamento_c = ATTIVATO;
  forward(FORWARD);
}

void avanti(int run_delay_1)
{
  if (posizione_sterzo == DRITTO)
  {
    radar_scan_180[361];
    Serial1.print( F("Distanza: "));
    Serial1.println(radar_ant);
    if (/*distance_ant>520 && */run_delay_1 == 21000)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti per 5mt"));
      delay(run_delay_1);
      forward(STOP);
    } else if (/*distance_ant>55 && */run_delay_1 == 0)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti per 50cm "));
      delay(2100);
      forward(STOP);
    } else if (/*distance_ant>12 && */run_delay_1 == 52)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti per 1cm "));
      delay(run_delay_1);
      forward(STOP);
    }
    else if (/*distance_ant<55 && distance_ant>25 && */run_delay_1 == 0)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti per 25cm"));
      delay(1050);
      forward(STOP);
    }
  } else if (posizione_sterzo > DRITTO)
  {
    //              Serial1.print( F("Distanza anteriore: "));
    //              Serial1.println(distance_ant);
    //              Serial1.print( F("Distanza a destra: "));
    //              Serial1.println(distance_dx);
    //               if(distance_ant>=20 && distance_dx>=10)
    //              {
    //                  forward(FORWARD);
    //                  Serial1.println( F("Avanti"));
    //                  delay(1050);// indietreggia di 25 cm verso destra
    //                  forward(STOP);
    //              }else if(/*distance_ant>12 && */run_delay_1 == 52/* && /*distance_dx>=8*/)
    //              {
    //                  forward(FORWARD);
    //                  Serial1.println( F("Avanti per 1cm "));
    //                  delay(run_delay_1);
    //                  forward(STOP);
    if (run_delay_1 != 52)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti"));
      delay(1050);// indietreggia di 25 cm verso sinistra
      forward(STOP);
    } else if (/*distance_ant>12 && */run_delay_1 == 52/* && distance_sx>=8*/)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti per 1cm "));
      delay(run_delay_1);
      forward(STOP);
    }
    //              }else if(distance_ant<10 && distance_dx<10)
    //              {
    //                  Serial1.println( F("Ostacolo a destra troppo vicino"));
    //              }
  } else if (posizione_sterzo < DRITTO)
  {
    //              Serial1.print( F("Distanza anteriore: "));
    //              Serial1.println(distance_ant);
    //              Serial1.print( F("Distanza a sinistra: "));
    //              Serial1.println(distance_sx);
    //               if(distance_ant>=20 && distance_sx>=10)
    //              {
    if (run_delay_1 != 52)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti"));
      delay(1050);// indietreggia di 25 cm verso sinistra
      forward(STOP);
    } else if (/*distance_ant>12 && */run_delay_1 == 52/* && distance_sx>=8*/)
    {
      forward(FORWARD);
      Serial1.println( F("Avanti per 1cm "));
      delay(run_delay_1);
      forward(STOP);
    }
    //              }else if(distance_ant<10 && distance_sx<10)
    //              {
    //                  Serial1.println( F("Ostacolo a sinistra troppo vicino"));
    //              }
  }
}

void indietro()
{
  if (avanzamento_c == ATTIVATO)
  {
    fine_conteggio_distanza = millis();
    forward(STOP);
    calcolo_distanza_percorsa();
    //avanzamento_c = DISATTIVATO;
  } else if (avanzamento_c == DISATTIVATO)
  {
    int distance_post = 0;
    int distance_sx = 0;
    int distance_dx = 0;
    //        distance_post = hc.dist(1);
    //        delay(200);
    //        distance_sx = hc.dist(3);
    //        delay(200);
    //        distance_dx = hc.dist(2);
    //        delay(200);

    if (posizione_sterzo == DRITTO)
    {
      //              Serial1.print( F("Distanza posteriore: "));
      //              Serial1.println(distance_post);
      //              if(distance_post>=12 && distance_dx>=15 && distance_sx>=15 && distance_post!=0 && distance_dx!=0 && distance_sx!=0)
      //              {
      forward(BACK);
      Serial1.println( F("Indietro"));
      delay(525);
      forward(STOP);
      //}
      //              else if(distance_post<55 && distance_post>25 && distance_post!=0)
      //              {
      //                  forward(BACK);
      //                  Serial1.println( F("Indietro per 25cm"));
      //                  delay(1050);
      //                  forward(STOP);
      //              }else if(distance_post<25 && distance_post!=0)
      //              {
      //                Serial1.print(F(" Sei troppo vicino ad un ostacolo, azione non consentita "));
      //              }
    } else if (posizione_sterzo > DRITTO)
    {
      //              Serial1.print( F("Distanza posteriore: "));
      //              Serial1.println(distance_post);
      //              Serial1.print( F("Distanza a destra: "));
      //              Serial1.println(distance_dx);
      //               if(distance_post>=15 && distance_dx>=25 && distance_post!=0 && distance_dx!=0)
      //              {
      forward(BACK);
      Serial1.println( F("Indietro"));
      delay(525);// indietreggia di 12,5 cm verso destra
      forward(STOP);
      //              }else if(distance_post<10 && distance_dx<10)
      //              {
      //                  Serial1.println( F("Ostacolo a destra troppo vicino"));
      //              }
    } else if (posizione_sterzo < DRITTO)
    {
      //              Serial1.print( F("Distanza posteriore: "));
      //              Serial1.println(distance_post);
      //              Serial1.print( F("Distanza a sinistra: "));
      //              Serial1.println(distance_sx);
      //               if(distance_post>=20 && distance_sx>=10 && distance_post!=0 && distance_sx!=0)
      //              {
      forward(BACK);
      Serial1.println( F("Indietro"));
      delay(1050);// indietreggia di 25 cm verso sinistra
      forward(STOP);
      //              }else if(distance_post<10 && distance_sx<10 && distance_post!=0 && distance_sx!=0)
      //              {
      //                  Serial1.println( F("Ostacolo a destra troppo vicino"));
      //              }
    }
  }
}


bool radar_alarm()
{
  //    radar_ant = radar_scan_180[361];
  //delay(radar_delay);
  //radar_motor(DRITTO);
  //    radar_dx_center = hc.dist(2);
  //            delay(radar_delay);
  //    radar_sx_center = hc.dist(3);
  //            delay(radar_delay);
  //    radar_post = hc.dist(1);
  //            delay(radar_delay);
  //    if(radar_ant/10<20)
  //    {
  //       return true;//AVANTI
  //    }
  //    if(radar_dx_center<20)
  //    {
  //       return true;//AVANTI
  //    }//DESTRO
  //    if(radar_sx_center<20)
  //    {
  //       return true;//AVANTI
  //    }//SINISTRA
  //    if(radar_post<20)
  //    {
  //       return true;//AVANTI
  //    }//POSTERIORE
  //    return false;
}

bool radar_ant_alarm()
{
  if (radar_ant_aux < 200 && radar_ant_aux != 0)
  {
    return true;//AVANTI
  }
  return false;
}

bool radar_ant_sx_alarm()
{
  if (radar_sx_ant < 200 && radar_sx_ant != 0)
  {
    return true;//AVANTI
  }
  return false;
}

bool radar_ant_dx_alarm()
{
  if (radar_dx_ant < 200 && radar_dx_ant != 0)
  {
    return true;//AVANTI
  }
  return false;
}

bool radar_sx_alarm()
{
  //      uint16_t radar_sx_center_1 = hc.dist(3);
  //              delay(radar_delay);
  //      uint16_t radar_sx_center_2 = hc.dist(3);
  //              delay(radar_delay);
      if(radar_sx_center<200)
      {
         return true;//AVANTI
      }//SINISTRA
      return false;
}

bool radar_dx_alarm()
{
  //      uint16_t radar_dx_center_1 = hc.dist(2);
  //              delay(radar_delay);
  //      uint16_t radar_dx_center_2 = hc.dist(2);
  //              delay(radar_delay);
      if(radar_dx_center<200)
      {
         return true;//AVANTI
      }//DESTRO
      return false;
}

//bool radar_post_alarm()
//{
//  //    radar_post = hc.dist(1);
//  //            delay(radar_delay);
//  //    if(radar_post<20)
//  //    {
//  //       return true;//AVANTI
//  //    }//POSTERIORE
//  //    return false;
//}

int lunghezza_int(int input)
{
  int count = 0;
  while (input > 0) {
    input /= 10;
    count++;
  }
  return count;
}

//void radar_update()
//{
//  //            radar_ant = sensor.readRangeSingleMillimeters()/10;
//  //            //delay(radar_delay);
//  //            radar_motor(radar_dx_ant_position);
//  //            delay(200);
//  //            radar_dx_ant = sensor.readRangeSingleMillimeters()-60;
//  //            delay(radar_delay);
//  //            radar_motor(radar_sx_ant_position);
//  //            delay(400);
//  //            radar_sx_ant = sensor.readRangeSingleMillimeters()-60;
//  //            delay(radar_delay);
//  //            radar_scan_180[361];
//  //            radar_dx_center = hc.dist(2);
//  //            delay(radar_delay);
//  //            radar_sx_center = hc.dist(3);
//  //            delay(radar_delay);
//  //            radar_post = hc.dist(1);
//  //            delay(radar_delay);
//}

void radar_s()
{
                int spazi = 0;
                Serial1.println(F("+=============|RADAR|=====================+"));
                  Serial1.print(F("|45:"));
                Serial1.print(radar_sx_ant_45);
                Serial1.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_sx_ant_45);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial1.print(F(" "));
                  }
                Serial1.print(F("A: "));
               Serial1.print(radar_ant);
                Serial1.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_ant);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial1.print(F(" "));
                  }
                Serial1.print(F("45:"));
                Serial1.print(radar_dx_ant_45);
                Serial1.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_dx_ant_45);
               for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial1.print(F(" "));
                  }
                spazi = 6 - lunghezza_int(radar_dx_ant);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial1.print(F(" "));
                  }
                Serial1.println(F("|"));
                Serial1.println(F("+=========================================+"));
                delay(radar_delay);
                Serial1.print(F("|S: "));
                Serial1.print(radar_sx_center);
                Serial1.print(F("mm"));
                spazi = 10 - lunghezza_int(radar_sx_center);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial1.print(F(" "));
                  }
                delay(radar_delay);
                //Serial1.print(F("F               "));
                Serial1.print(F("D: "));
                Serial1.print(radar_dx_center);
                Serial1.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_dx_center);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial1.print(F(" "));
                  }
                Serial1.println(F("|"));
                Serial1.println(F("+=========================================+"));
                //Serial1.println(F("|"));
                delay(radar_delay);
                Serial1.print(F("|"));
                Serial1.print(F("                     D: "));
                Serial1.print(radar_post);
                Serial1.print(F("mm"));
                spazi = 21 - lunghezza_int(radar_post);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial1.print(F(" "));
                  }
                Serial1.println(F("|"));
                Serial1.println(F("+=========================================+"));
}

void stampa_distanza_percorsa()
{
  Serial1.print(F("Distanza totale percorsa: "));
  if (distanza_percorsa < 100) {
      Serial1.print(distanza_percorsa);
      Serial1.println(F(" cm"));
    } else {
      Serial1.print(distanza_percorsa / 100);
      Serial1.println(F(" m"));
    }
}

void i2c_detect()
{
  byte x = 0;
  byte y = 0;

  Serial.begin(9600);
  while(!Serial){ delay(1000); }
  Serial.println();
  Serial.println("I2C Address Scanner.");
  Serial.println ("I2C_0:");
  //Wire.begin();
  for(byte i=0; i<255; i++){
    Wire.beginTransmission(i);
    if(Wire.endTransmission()==(byte)0){
      x++;
      //delay(300);
    }
  }
  Serial.println ("I2C_1:");
  for(byte i=0; i<255; i++){
      Wire1.beginTransmission(i);
      if(Wire1.endTransmission()==(byte)0){
      y++;
    }
  }
  Serial.print("(I2C0: ");
  Serial.print(x, DEC);
  Serial.println(") detected.");
  Serial.print("(I2C1: ");
  Serial.print(y, DEC);
  Serial.println(") detected.");
}

void rover_commands(int str)
{
  if (str == 'a')
  {
    int p = 0;
    if (posizione_sterzo - valore_sterzata >= STERZO_SINISTRA_MAX)
    {
      posizione_sterzo = posizione_sterzo - valore_sterzata;
      sterzo(posizione_sterzo);
      Serial1.print( F("Sterzo: "));
      Serial1.println(posizione_sterzo/*posizione_sterzo*/);
      delay(100);
    }
  } else if (str == 'd')
  {
    int p = 0;
    if (posizione_sterzo + valore_sterzata <= STERZO_DESTRA_MAX)
    {
      posizione_sterzo = posizione_sterzo + valore_sterzata;
      sterzo(posizione_sterzo);
      Serial1.print( F("Sterzo: "));
      Serial1.println(posizione_sterzo/*posizione_sterzo*/);
      delay(100);
    }
  } else if (str == 'w') {
    avanti(0);
  } else if (str == 's') {
    indietro();
  } else if (str == 'e') {
    //anticollisione = ATTIVATO;
    //Serial1.print( F("Anticollisione attivata automaticamente"));
    avanzamento_continuo();

  } else if (str == 'q') {
    avanti(52);
  }else if (str == '4') {
    if (ruote_sterzanti_post == ATTIVATO && ruote_sterzanti_ant == ATTIVATO)
    {
      ruote_sterzanti_post = DISATTIVATO;
      sterzo(STERZO_DRITTO);
      Serial1.println( F("Sterzo posteriore disattivato"));

    } else if (ruote_sterzanti_post == DISATTIVATO)
    {
      ruote_sterzanti_post = ATTIVATO;
      Serial1.println( F("Sterzo posteriore attivato"));
    } else {
      Serial1.println( F("Impossibile disattivare lo sterzo posteriore: attiva lo sterzo anteriore per disattivare il posteriore"));
    }
  }
  else if (str == 'p') {
    if (anticollisione == ATTIVATO)
    {
      //radar_update();
      if (radar_dx_center > 200 && radar_sx_center > 200)
      {
        open_panels();
        panels_security_distance = 100;
      }else Serial1.println(F("Possibili ostacoli ai lati del rover"));
    } else open_panels();
  } else if (str == 'o') {
    close_panels();
    panels_security_distance = 0;
  } else if (str == 'r') {
    radar_s();
  } else if (str == '1') {
    if (webcam_sterzo == DISATTIVATO)
    {
      webcam_sterzo = ATTIVATO;
      Serial1.println(F("Webcam Sterzo attivato"));
    }
    else if (webcam_sterzo = ATTIVATO)
    {
      webcam_sterzo = DISATTIVATO;
      Serial1.println(F("Webcam Sterzo disattivato"));
    }
  } else if (str == '5') {
    if (anticollisione == DISATTIVATO)
    {
      anticollisione = ATTIVATO;
      //Serial_radar.print('5');
      Serial1.println(F("Anticollisione attivata"));
    }
    else if (anticollisione = ATTIVATO)
    {
      anticollisione = DISATTIVATO;
      Serial1.println(F("Anticollisione disattivata"));
    }
  } else if (str == 'Q') {
    if (diagnostic == DISATTIVATO)
    {
      diagnostic = ATTIVATO;
      Serial1.println(F("Diagnostica attivata"));
    }
    else if (diagnostic = ATTIVATO)
    {
      diagnostic = DISATTIVATO;
      Serial1.println(F("Diagnostica disattivata"));
    }
  } else if (str == '4') {
    _webcam_sterzo(330);
  } else if (str == '2') {
    sterzo_dritto();
  } else if (str == '6') {
    if (sterzo_automatico == DISATTIVATO)
    {
      sterzo_automatico = ATTIVATO;
      Serial1.println(F("Sterzo automatico attivato"));
    }
    else if (sterzo_automatico = ATTIVATO)
    {
      sterzo_automatico = DISATTIVATO;
      Serial1.println(F("Sterzo automatico disattivato"));
    }
  }/**/else if (str == 'n')
  {
    //radar_update();
    if (radar_sx_ant > 200 && radar_sx_center > 200)
      peso_a_sinistra();
    else Serial1.println(F("Possibili ostacoli a sinistra del rover"));

  } else if (str == 'm')
  {
    //radar_update();
    if (radar_dx_center > 200)
      peso_a_destra();
    else Serial1.println(F("Possibili ostacoli a sinistra del rover"));
  } else if (str == 'i')
  {
    if (webcam_sinistra_destra > 235 && webcam_sinistra_destra < 355 && webcam_sopra_sotto + 15 > 435) {
      Serial1.println(F("Movimento massimo cam raggiunto"));
    } else {
      webcam_sopra_sotto = webcam_sopra_sotto + 15;
      webcam_control();
      webcam_Y--;
      Serial1.print(F("Posizione Cam ( "));
      Serial1.print(webcam_sinistra_destra);
      Serial1.print(F(", "));
      Serial1.print(webcam_sopra_sotto);
      Serial1.println(F(" )"));
    }

  } else if (str == 'k')
  {
    //                if(webcam_sopra_sotto_effettuato == 0 && webcam_sopra_sotto_old > 435)
    //                {
    //                  webcam_sopra_sotto_effettuato = 1;
    //}
    if(webcam_sopra_sotto - 15<=225)
    {
      Serial1.println(F("Movimento massimo cam raggiunto"));
    }else{
        webcam_sopra_sotto = webcam_sopra_sotto - 15;
        webcam_control();
        webcam_Y++;
        Serial1.print(F("Posizione Cam ( "));
        Serial1.print(webcam_sinistra_destra);
        Serial1.print(F(", "));
        Serial1.print(webcam_sopra_sotto);
        Serial1.println(F(" )"));
    }

  } else if (str == 'l')
  {
    if(webcam_sinistra_destra - 15>=100)
    {
        webcam_sinistra_destra = webcam_sinistra_destra - 15;
        webcam_control();
        webcam_X++;
        Serial1.print(F("Posizione Cam ( "));
        Serial1.print(webcam_sinistra_destra);
        Serial1.print(F(", "));
        Serial1.print(webcam_sopra_sotto);
        Serial1.println(F(" )"));
    } else Serial1.println(F("Limite rotazione CAM raggiunto"));

  } else if (str == 'j')
  {
    if(webcam_sinistra_destra + 15<=505)
    {
        webcam_sinistra_destra = webcam_sinistra_destra + 15;
        webcam_control();
        webcam_X--;
        Serial1.print(F("Posizione webcam ( "));
        Serial1.print(webcam_sinistra_destra);
        Serial1.print(F(", "));
        Serial1.print(webcam_sopra_sotto);
        Serial1.println(F(" )"));
    }else Serial1.println(F("Limite rotazione CAM raggiunto"));
  } else if (str == 'b')
  {

    val_1 = analogRead(analogInput_1);//reads the analog input
    Vout_1 = (val_1 * 4.20) / 1023.00; // formula for calculating voltage out i.e. V+, here 5.00
    //Vin = Vout_1 / (R2_1/(R1_1+R2_1)); // formula for calculating voltage in i.e. GND
    Vin_1 = Vout_1 / (R2_1 / (R1_1 + R2_1));
    if (Vin_1 < 0.09) //condition
    {
      Vin_1 = 0.00;//statement to quash undesired reading !
    }
    float perc_1 = map(Vin_1, 3.6, 4.2, 0, 100);

    val_2 = analogRead(analogInput_2);//reads the analog input
    Vout_2 = (val_2 * 4.20) / 1023.00; // formula for calculating voltage out i.e. V+, here 5.00
    //Vin = Vout_2 / (R2_1/(R1_1+R2_1)); // formula for calculating voltage in i.e. GND
    Vin_2 = Vout_2 / (R2_2 / (R1_2 + R2_2));
    if (Vin_2 < 0.09) //condition
    {
      Vin_2 = 0.00; //statement to quash undesired reading !
    }
    float perc_2 = map(Vin_2, 3.6, 4.2, 0, 100);
    Serial1.print(F("\t BATTERIA MOTORI / ARDUINO = "));
    Serial1.print(Vin_1);
    Serial1.print(F("V"));
    Serial1.print(F(" | "));
    Serial1.print(perc_1);
    Serial1.println(F("%"));
    Serial1.print(F("\t BATTERIA RASPBERRY = "));
    Serial1.print(Vin_2);
    Serial1.print(F("V"));
    Serial1.print(F(" | "));
    Serial1.print(perc_2);
    Serial1.println(F("%"));
  } else if (str == '0') {
    pca_on_off();
  }else if (str == '7') {
       Serial1.print( F("Trovato muro a "));
       if(radar_dx_center < radar_sx_center)
       {
       Serial1.println( F("destra -->"));
       }else Serial1.println( F("sinistra <--"));
        if (costeggia_muro_enabled)
        {
          costeggia_muro_enabled = false;
          Serial1.println( F("Modalità costeggia muro disabilitata"));
        } else {
          costeggia_muro_enabled = true;
          Serial1.println( F("Modalità costeggia muro abilitata"));
        }
  } else if (str == '8') {
    stampa_distanza_percorsa();
  } else if (str == 'D') {
    i2c_detect();
  }else if (str == 'z') {
    if(motor_speed + 10 <= 100  && motor_speed_id >= 1)
    {
        motor_speed = motor_speed + 10;
        motor_speed_id--;
        Serial1.print("Velocita': ");
        Serial1.println(motor_speed_id);
        if(avanzamento_c == ATTIVATO){
           forward(FORWARD);
        }
    }
  }else if (str == 'x') {
    if(motor_speed - 10 >= 0 && motor_speed_id <= 10)
    {
        motor_speed = motor_speed - 10;
        motor_speed_id++;
        Serial1.print("Velocita': ");
        Serial1.println(motor_speed_id);
        if(avanzamento_c == ATTIVATO){
           forward(FORWARD);
        }
    }
  }else if (str == 'O') {
        Serial1.println("Azzeramento ODO: ");
        delay(1000);
        save_odo_eeprom(0);
  }else if (str == 'c') {
    if(trimmer>=-10)
        {
        trimmer--;
        Serial1.print("Trimmer: ");
        Serial1.println(trimmer);
        sterzo(posizione_sterzo);
        EEPROM.put(10,trimmer);
        if(diagnostic == ATTIVATO)
          Serial1.print("Trimmer salvato! ");
        }
  }else if (str == 'v') {
        if(trimmer<=10)
        {
        trimmer++;
        Serial1.print("Trimmer: ");
        Serial1.println(trimmer);
        sterzo(posizione_sterzo);
        EEPROM.put(10,trimmer);
        if(diagnostic == ATTIVATO)
          Serial1.print("Trimmer salvato! ");
        }
        }else if (str == 'h') {
    //DA AGGIORNARE
    Serial1.println( F("SilverRover 2.0"));
    Serial1.println( F("Ver. 2.0 (...)"));
    Serial1.println( F("Ver. 1.2.0 (Aggiunto angolo di Ackermann, sterzo automatico per evitare gli ostacoli, controllo ostacoli prima di aprire i pannelli)"));
    Serial1.println( F("COMANDI DISPONIBILI:"));
    Serial1.println( F("1)  |w||a||s||d|: avanti, sterza a sinistra, sterza a destra, indietro"));
    Serial1.println( F("2)  |i||j||k||l|: pan/tilt webcam"));
    Serial1.println( F("3)  stop: se è in movimento, il rover si ferma"));
    Serial1.println( F("4)  pannello_dx: apre il pannello solare destro (SCONSIGLIATO)"));
    Serial1.println( F("5)  pannello_sx: apre il pannello solare sinistro (SCONSIGLIATO)"));
    Serial1.println( F("6)  |p|: apre i pannelli solari"));
    Serial1.println( F("7)  |o|: chiude i pannelli solari"));
    Serial1.println( F("8)  |r|: effettua la scansione con tutti i sonar (avanti, dietro, sinistra, destra) riportando la distanza in cm di eventuali ostacoli rilevati"));
    Serial1.println( F("9)  |t|: effettua la scansione con il sonar anteriore, questo sonar è motorizzato,"));
    Serial1.println( F("10) |n|: se abilitato mantiene aperto soltanto il pannello solare sinistro per aumentare il peso verso sinistra"));
    Serial1.println( F("11) |m|: se abilitato mantiene aperto soltanto il pannello solare destro per aumentare il peso verso destra"));
    Serial1.println( F("12) |z|: dimunuisce la precisione dello sterzo diminuendo gli step di sterzata"));
    Serial1.println( F("13) |x|: aumenta la precisione dello sterzo aumentando gli step di sterzata"));
    Serial1.println( F("14) |1|: se abilitato, la webcam ruoterà insieme alle ruote anteriori"));
    Serial1.println( F("15) |2|: posizione lo sterzo dritto (valore: 330)"));
    Serial1.println( F("17) |4|: attiva/disattiva lo sterzo anteriore (default: ATTIVATO)"));
    Serial1.println( F("18) |5|: attiva/disattiva lo sterzo posteriore (default: ATTIVATO)"));
  }else {
    Serial1.println( F("Comando sconosciuto..."));
  }
}

void anticollision()
{
  if (anticollisione == ATTIVATO)
  {
    //rp2040.idleOtherCore();
    if (radar_ant <= 200 && radar_ant != 0)
    {
        if(avviso_ant == DISATTIVATO){
            Serial1.println( F("RADAR ANT ALARM!"));
            avviso_ant = ATTIVATO;
       }
        forward(STOP);
    }else avviso_ant = DISATTIVATO;
    //=================SX_45_Procedure===============================
    if (radar_sx_ant_45  + panels_security_distance <= 180  && radar_sx_ant_45 != 0/*radar_sx_alarm()*/)
    {
      if(avviso_sx_ant_45 == DISATTIVATO){
          Serial1.println( F("RADAR SX 45 ALARM!"));
          avviso_sx_ant_45 = ATTIVATO;
      }
      if (sterzo_automatico == ATTIVATO)
      {
        posizione_sterzo = 440;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial1.print( F("1) "));
        Serial1.print( F("Sterzo automatico: "));
        Serial1.println(posizione_sterzo/*posizione_sterzo*/);
        //}
        delay(1500);
        posizione_sterzo = DRITTO;
        sterzo(posizione_sterzo);
      }
    }else avviso_sx_ant_45 = DISATTIVATO;
    //=================DX_45_Procedure===============================
    if (radar_dx_ant_45 <= 180  + panels_security_distance && radar_dx_ant_45 != 0/*radar_dx_alarm()*/)
    {
     if(avviso_dx_ant_45 == DISATTIVATO){
      Serial1.println( F("RADAR DX 45 ALARM!"));
      avviso_dx_ant_45 = ATTIVATO;
     }
      if(sterzo_automatico == ATTIVATO)
      {
        posizione_sterzo = limite_sinistro;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial1.print( F("2) "));
        Serial1.print( F("Sterzo automatico: "));
        Serial1.println(posizione_sterzo/*posizione_sterzo*/);
        //}
        delay(1500);
        posizione_sterzo = DRITTO;
        sterzo(posizione_sterzo);
      }
    }else avviso_dx_ant_45 = DISATTIVATO;
    //=================DX_Procedure===============================
  if (radar_dx_center <= security_distance + panels_security_distance && radar_sx_center >= security_distance + panels_security_distance && radar_dx_ant_45 != 0/*radar_dx_alarm()*/)
    {
      if(avviso_dx_center == DISATTIVATO && sterzo_automatico == DISATTIVATO){
        Serial1.println( F("RADAR DX ALARM!"));
          avviso_dx_center = ATTIVATO;
      }
      if(sterzo_automatico == ATTIVATO)
      {
        if(posizione_sterzo != 319)
        {
            posizione_sterzo = 319;
            sterzo(posizione_sterzo);
            if(diagnostic == ATTIVATO)
              Serial1.print( F("3) "));
            Serial1.print( F("Sterzo automatico: "));
            Serial1.println(posizione_sterzo/*posizione_sterzo*/);
        }
      }
    }else if(radar_dx_center >= security_distance  + panels_security_distance && radar_sx_center > radar_dx_center/*radar_dx_alarm()*/)
    {
      avviso_dx_center = DISATTIVATO;
      if (sterzo_automatico == ATTIVATO)
      {
        if(posizione_sterzo != 341)
        {
        posizione_sterzo = 341;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial1.print( F("4) "));
        Serial1.print( F("Sterzo automatico: "));
        Serial1.println(posizione_sterzo/*posizione_sterzo*/);
        }
      }
    }
      //else //forward(STOP);
    //=================SX_Procedure===============================
    else if (radar_sx_center <= security_distance + panels_security_distance && radar_dx_center >= security_distance + panels_security_distance && radar_sx_center != 0/*radar_dx_alarm()*/)
    {
      if(avviso_sx_center == DISATTIVATO && sterzo_automatico == DISATTIVATO){
        Serial1.println( F("RADAR SX ALARM!"));
          avviso_sx_center = ATTIVATO;
      }
      if (sterzo_automatico == ATTIVATO)
      {
        if(posizione_sterzo != 341)
        {
        posizione_sterzo = 341;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial1.print( F("5) "));
        Serial1.print( F("Sterzo automatico: "));
        Serial1.println(posizione_sterzo/*posizione_sterzo*/);
        }
      }
    }else if (radar_sx_center >= security_distance + panels_security_distance && radar_dx_center > radar_sx_center /*radar_dx_alarm()*/)
    {
      avviso_sx_center = DISATTIVATO;
      if (sterzo_automatico == ATTIVATO)
      {
//        if(radar_sx_center < radar_dx_center)
//          {
                if(avviso_sx_center == DISATTIVATO && sterzo_automatico == DISATTIVATO){
                    Serial1.println( F("RIAVVICINAMENTO A SINISTRA!"));
                    avviso_sx_center = ATTIVATO;
                }
                if (sterzo_automatico == ATTIVATO)
                {
                  if(posizione_sterzo != 319)
                  {
                    posizione_sterzo = 319;
                    sterzo(posizione_sterzo);
                    if(diagnostic == ATTIVATO)
                      Serial1.print( F("6) "));
                    Serial1.print( F("Sterzo automatico: "));
                    Serial1.println(posizione_sterzo/*posizione_sterzo*/);
                  }
                }
        else if(radar_sx_center == 2000 && avviso_dx_center == 2000 && posizione_sterzo != 330)
        {
          posizione_sterzo = 330;
          sterzo(posizione_sterzo);
          if(diagnostic == ATTIVATO)
              Serial1.print( F("7) "));
          Serial1.print( F("Sterzo automatico: "));
          Serial1.println(posizione_sterzo/*posizione_sterzo*/);
          Serial1.println( F("Nessun ostacolo di riferimento trovato."));
          sterzo_automatico = DISATTIVATO;
          Serial1.println( F("Sterzo automatico disabilitato automaticamente."));
        }   
      }
      }
      }
    //=================SX_DX_Procedure==================================
    else if(radar_sx_center <= security_distance + panels_security_distance && radar_dx_center <= security_distance + panels_security_distance/*radar_dx_alarm()*/)
        if(radar_sx_center < radar_dx_center)
          {
            if (sterzo_automatico == ATTIVATO)
                {
                  if(posizione_sterzo != 341)
                  {
                      posizione_sterzo = 341;
                      sterzo(posizione_sterzo);
                      Serial1.print( F("9) Sterzo automatico: "));
                      Serial1.println(posizione_sterzo/*posizione_sterzo*/);
                  }
                }
        }else if (radar_sx_center > radar_dx_center)
                {
                  if (sterzo_automatico == ATTIVATO)
                  {
                    if(posizione_sterzo != 319)
                    {
                      posizione_sterzo = 319;
                      sterzo(posizione_sterzo);
                      Serial1.print( F("10) Sterzo automatico: "));
                      Serial1.println(posizione_sterzo/*posizione_sterzo*/);
                    }
                  }
          }
//    if(radar_sx_center > 150 + panels_security_distance && radar_dx_center > 150 + panels_security_distance/*radar_dx_alarm()*/)
//    {
//                  if (sterzo_automatico == ATTIVATO)
//                  {
//                      posizione_sterzo = DRITTO;
//                      sterzo(posizione_sterzo);
//                      Serial1.print( F("Sterzo automatico: "));
//                      Serial1.println(posizione_sterzo/*posizione_sterzo*/);
//                  }
//          }
  
//    if(radar_ant > 200)
//      avviso_ant = DISATTIVATO;
//    if (radar_sx_ant_45 > 130  + panels_security_distance)
//      avviso_sx_ant_45 = DISATTIVATO;
//    if (radar_dx_ant_45 > 130  + panels_security_distance)
//      avviso_dx_ant_45 = DISATTIVATO;
//    if (radar_dx_center > 150  + panels_security_distance)
//      avviso_dx_center = DISATTIVATO;
//    if (radar_sx_center > 150  + panels_security_distance)
//      avviso_sx_center = DISATTIVATO;
  }
  //rp2040.resumeOtherCore();

//void realtime_radar()
//{
//  if (radar_direction == 0)
//  {
//    radar_scan_posizion = radar_scan_posizion + 1;
//  }
//  if (radar_direction == 1) {
//    radar_scan_posizion = radar_scan_posizion - 1;
//  }
//  radar_motor(radar_scan_posizion);
//  if (radar_scan_posizion == 110) {
//    radar_direction = 0;
//  if (radar_scan_posizion == 580) {
//    radar_direction = 1;
//    radar_sx_ant_aux = radar_sx_ant;
//  }
//}

void costeggia_muro()
{
  if (radar_sx_center < radar_dx_center)
  {
    if (radar_sx_center < 250 && radar_sx_ant != 0)
    {
      sterzo(STERZO_DRITTO + valore_sterzata);
    } else if (radar_sx_ant > 270 && radar_sx_ant != 0)
    {
      sterzo(posizione_sterzo - valore_sterzata);
    }
  } else {
    if (radar_dx_ant < 250 && radar_dx_ant != 0)
    {
      sterzo(STERZO_DRITTO - valore_sterzata);
    } else if (radar_sx_ant > 270 && radar_dx_ant != 0)
    {
      sterzo(posizione_sterzo + valore_sterzata);
    }
  }
}

void raspberry_serial_communication()
{
  while (Serial1.available() > 0 ) {
    uint8_t str = Serial1.read();
    rover_commands(str);
  }
}

void loop() {
  raspberry_serial_communication();
  anticollision();
}
