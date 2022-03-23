#include <Wire.h>
//#include <HCSR04.h>
//#include <PacketSerial.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include "Adafruit_VL53L0X.h"
#include "ADS1X15.h"
#include "SDL_Arduino_INA3221.h"
#include <Adafruit_INA219.h>
//#include <INA219B.h>
#include <INA219_WE.h>
//#include <ArduinoJson.h>
//#include <MsgPacketizer.h>
//#include <CmdBuffer.hpp>
//#include <CmdCallback.hpp>
//#include <CmdParser.hpp>
//CmdCallback<3> cmdCallback;
//CmdBuffer<32> myBuffer;
//CmdParser     myParser;

#include <CommandParser.h>

typedef CommandParser<> MyCommandParser;

MyCommandParser parser;
//ADS1115 ADS(0x48);
Servo radar_servo;
//SPI PINs DEFINITION
#include <SPI.h>
char buff [50];
volatile byte indx;
volatile boolean process;
#define SPI_SCK_PIN 18
#define SPI_MISO_PIN 20
#define SPI_MOSI_PIN 19
#define SPI_CS_PIN 17

//====================
#define PYTHON_SERVER 1
#define KEYBOARD 0
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates
#define STERZO_SINISTRA_MAX 220
#define STERZO_DRITTO 330 //328
#define STERZO_DESTRA_MAX 440
#define DRITTO STERZO_DRITTO
#define CHIUSO 0
#define APERTO 1
#define META 2
#define ATTIVATO 1
#define DISATTIVATO 0
#define reset_button 22

#define STERZO_ANT_DX 1
#define STERZO_ANT_SX 0
#define STERZO_POST_dx 3
#define STERZO_POST_SX 2
#define RUOTA_ANT_DX 5
#define RUOTA_ANT_SX 4
#define RUOTA_POST_DX 7
#define RUOTA_POST_SX 6
#define WHEEL_FORWARD_LEFT 6//4
#define WHEEL_BACKWARD_LEFT 5
#define WHEEL_FORWARD_RIGHT 4
#define WHEEL_BACKWARD_RIGHT 7
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33
#define LOX5_ADDRESS 0x34
#define LOX6_ADDRESS 0x35
int sensor1,sensor2, sensor3, sensor4, sensor5, sensor6;
#define SHT_LOX1 11
#define SHT_LOX2 12
#define SHT_LOX3 13
#define SHT_LOX4 10
#define SHT_LOX5 9
#define SHT_LOX6 8
bool sys_msg = false;
SDL_Arduino_INA3221 ina3221_rover(0x40);
//SDL_Arduino_INA3221 ina219_battery(0x41);
//Adafruit_INA219 ina219_battery(0x44);
INA219_WE ina219_battery = INA219_WE(&Wire1, 0x44);
#define INA3221_VOLTAGE_BATTERY_CHANNEL 1
#define ACS712_CURRENT_BATTERY_CHANNEL 2
#define SOLAR_CELL_CHANNEL 2
#define OUTPUT_CHANNEL 3
#define PCA_ON_OFF_PIN 22
static char incomingByte = 0;
uint8_t COMMAND_MODE = KEYBOARD;
uint8_t accelerator = 0;
uint8_t str_command = 0;


//PacketSerial MyPacketSerial;

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
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();//----------------------------------------------------------------------------------------------------

const uint16_t STOP = 0;//1500;
const uint16_t FORWARD = 2000;
const uint16_t BACK = 1000;
const uint16_t run_delay = 2100; //2100 percorre circa 50cm
uint8_t POSIZIONE_PANNELLO_DX = CHIUSO;
uint8_t POSIZIONE_PANNELLO_SX = CHIUSO;
uint8_t POSIZIONE_PANNELLO_REAR = CHIUSO;
uint8_t peso_destra = 0;
uint8_t peso_sinistra = 0;
uint8_t stato_precedente = CHIUSO;
const uint8_t panel_speed = 5;
uint16_t radar_scan_180[48];
uint8_t webcam_sterzo = ATTIVATO;
uint8_t radar_sterzo = DISATTIVATO;
uint8_t pca_off = ATTIVATO;
uint16_t posizione_sterzo = STERZO_DRITTO;
uint16_t webcam_sopra_sotto = 250 - 45;
uint16_t webcam_sinistra_destra = 268;
uint16_t webcam_sopra_sotto_effettuato = DISATTIVATO;
uint16_t radar_scan_posizion = DRITTO;
uint8_t risparmio_energetico = DISATTIVATO;
uint8_t sterzo_automatico = DISATTIVATO;
uint8_t radar_direction = 0;
uint8_t avviso_ant = 0;
uint8_t avviso_dx_ant_45 = 0;
uint8_t avviso_sx_ant_45 = 0;
uint8_t avviso_dx_center = 0;
uint8_t avviso_sx_center = 0;
uint8_t motor_speed_id = 10;
bool disabled_motors = DISATTIVATO;

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
uint16_t panel_position_rear = 0;
uint16_t panels_security_distance = 0;
uint16_t security_distance = 250;
uint8_t loop1_times = 0;
int residual_time_battery = 0;

int8_t avanzamento_c = DISATTIVATO;
int8_t indietro_c = DISATTIVATO;
int8_t anticollisione = ATTIVATO;

int radar_dx_ant = 0;
int radar_dx_ant_aux = 0;
int radar_dx_center = 0;
int radar_dx_center_aux = 0;
int radar_dx_ant_45 = 0;
int radar_dx_ant_45_aux = 0;

int radar_sx_ant = 0;
int radar_sx_ant_aux = 0;
int radar_sx_center = 0;
int radar_sx_center_aux = 0;
int radar_sx_ant_45 = 0;
int radar_sx_ant_45_aux = 0;
int radar_ant = 0;
int radar_ant_aux = 0;

uint8_t counter_second_level = 0;
uint8_t moltiplicatore_sterzata_automatica = 1;

uint16_t radar_motor_speed = 400;

int16_t motor_speed = 2025;
uint8_t mot_speed_perc = 0;
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
//INA3221
float current[3];
float voltage[3];

float Vin_1 = 0;
float Vin_2 = 0;
float Battery_1_Volt = 0;
float Battery_2_Volt = 0;
float f = 0;

float temp_1 = 0.0;
float temp_2 = 0.0;

float shuntvoltage1 = 0;
float busvoltage1 = 0;
float current_mA1 = 0;
float loadvoltage1 = 0;
float max_current1 = 0;
float min_current1 = 0;
float battery_percentage = 0;
float power_load1 = 0;
float mah_battery = 0;

float shuntvoltage2 = 0;
float busvoltage2 = 0;
float current_mA2 = 0;
float loadvoltage2 = 0;
float max_current2 = 0;
float min_current2 = 0;
float solar_efficiency = 0;

float shuntvoltage3 = 0;
float busvoltage3 = 0;
float current_mA3 = 0;
float loadvoltage3 = 0;
float max_current3 = 0;
float min_current3 = 0;
float power_load3 = 0;

// INA219
float shuntvoltage_battery = 0;
float busvoltage_battery = 0;
float current_mA_battery = 0;
float loadvoltage_battery = 0;
float power_mW_battery = 0;

//PID============= 
float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms

float kp=8; //Mine was 8
float ki=0.2; //Mine was 0.2
float kd=3100; //Mine was 3100
float distance_setpoint = 21;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;

//=================

uint32_t tempo_funzionamento = millis();
uint32_t inizio_conteggio_distanza = 0;
uint32_t fine_conteggio_distanza = 0;
float distanza_percorsa;


int post_sx_speed_2 = 0;
uint8_t reset_lidar_sensor = 0;
uint8_t turn_off_lidar_sensor = 0;
bool costeggia_muro_enabled = false;
const int chipSelect = 10;
uint8_t Lidar_enabled = 1;
uint8_t diagnostic = DISATTIVATO;
char buffer_message[12];
int8_t run_value = 0;
int index_index = 0;
uint16_t LED_debug_timing = 0;
bool LED_debug = false;

//pinMode(15, OUTPUT);

//uint16_t speed = 1500;

void disable_lidar_sensors()
{
  Lidar_enabled = 0;
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);    
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);
}

void setID() {  //SET ID FOR EACH LIDAR SENSOR TO I2C1 COMMUNICATION
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

void read_five_sensors() { //READ DATA FROM LIDAR SENSORS AND STORE DATA ABOUT DISTANCE ON GLOBAL VARIABLES
  
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

void setup1() {  //SETUP FUNCTION FOR CORE1

  //I2C0 Setup
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin(); //I2C1 Begin
  Serial.begin(9600); //Serial0 Begin (USB ---> PC)

  ina3221_rover.begin();
  ina219_battery.init();
  //ina219.begin();
  ina219_battery.setShuntSizeInOhms(0.05);
  
  Serial.print("Manufactures ID=0x");
  int MID_rover;
  MID_rover = ina3221_rover.getManufID();
  int MID_battery;
  //MID_battery = //ina219_battery.getManufID();
  Serial.println(MID_rover,HEX);
  //Serial.println(MID_battery,HEX);

  
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
  //radar_servo.attach(15);
  Serial.println();
  mot_speed_perc = map( motor_speed, 0, 4050, 0, 100);
  delay(2000);
  Serial.println("Setup CORE 1 [OK]");
}

void rad_mot() // FUNCTION TO MOVE THE FRONT LIDAR SENSOR CONTINUOSLY BETWEEN 0 - 180 DEGREES, NOT CURRENTLY USED
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


const int MPU_ADDR = 0x3C; // I2C address of SW6106 IC of the HAT
byte READ_LENGTH = 1; //amount of bytes to read from every adress 

//##################### BATTERY READING #################################
float getBattery(){
  
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x14); //First register adress for the battery 
  Wire1.endTransmission(true); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire1.requestFrom(MPU_ADDR, READ_LENGTH);  //read 1 byte from the i2c adress

  byte LSB = Wire1.read();


  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x15); //Second register adress for the battery
  Wire1.endTransmission(true);
  Wire1.requestFrom(MPU_ADDR, READ_LENGTH); 

  byte MSB = Wire1.read();

  //Set last four bits to 0 as they are unused
  for(int i = 4; i<=7;i++){
    bitClear(MSB,i);
    }


  uint16_t val = ((MSB << 8) | LSB); // Battery Voltage
  
  float vbat = val*0.0012;
  
  return vbat;
  
}



//###################### NTC Temperature #####################################
int getTemperature(){
  
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0X1B); //First adress to determine ntc temperature
  Wire1.endTransmission(true);
  Wire1.requestFrom(MPU_ADDR, READ_LENGTH);

  byte ntc_temp_1b = Wire1.read();
  
  //clears from bit 0 to 3 as they are not used
  for (int i = 0; i < 4; i++) {
    bitClear(ntc_temp_1b, i);
  }

  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x1C); //Second register adress to determine ntc temperature
  Wire1.endTransmission(true);
  Wire1.requestFrom(MPU_ADDR, READ_LENGTH);

  byte ntc_temp_1c = Wire1.read();
  //Final ADC value that corresponds to the NTC temperature. For more information look Datasheet
  uint16_t ntc_temp = ((ntc_temp_1b << 4) | ntc_temp_1c ); //Shift only four to left not 8 as indicated in datasheet.
  return ntc_temp;
  
}


  
//########### ERROR DETECTION WITH IRQ #############
int getError(){
  
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x0A); //Requesting information to register for IRQ-related issues
  Wire1.endTransmission(true);

  Wire1.requestFrom(MPU_ADDR,READ_LENGTH);

  byte IRQ = Wire1.read();
  return IRQ;

}



void SW6106()
{

    

    //################ DATA OUTPUT ############################
    float vbat = getBattery();
    uint16_t temperature = getTemperature();
    uint8_t error = getError();
    Serial.print("Vbat: ");
    Serial.print(vbat);
    Serial.print(" v.");
    Serial.print(" NTC Temperature: ");
    Serial.print(temperature);
    Serial.print(" IRQ Error: ");
    Serial.println(error,BIN); // If any bit of the IRQ binary value is 1 that means an issue has occured. Look in datasheet

}

 
void read_battery_power_INA219()
{
  //float ACSoffset = 2500;//2490.2;
  //uint8_t mVperAmp = 185;
  //busvoltage2 = ina3221_rover.getBusVoltage_V(SOLAR_CELL_CHANNEL);
  //shuntvoltage2 = ina3221_rover.getShuntVoltage_mV(SOLAR_CELL_CHANNEL);
  //current_mA2 = -ina3221_rover.getCurrent_mA(SOLAR_CELL_CHANNEL);
  //loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);

  //shuntvoltage_battery = ina219_battery.getBusVoltage_V(1);// + //ina219_battery.getShuntVoltage_mV(2);
  //Serial.println(shuntvoltage_battery);
  //ina219_battery.getBusVoltage_V
  busvoltage_battery = ina219_battery.getBusVoltage_V(); //(INA3221_VOLTAGE_BATTERY_CHANNEL);// + //ina219_battery.getBusVoltage_V(2);
  current_mA_battery = ina219_battery.getCurrent_mA();//(ACS712_CURRENT_BATTERY_CHANNEL);// - ACSoffset) / 0.185;;//ina219_battery.getBusVoltage_raw(ACS712_CURRENT_BATTERY_CHANNEL);// - ACSoffset)) / 0.185;// / mVperAmp); //ina219.getCurrent_mA();// + //ina219_battery.getCurrent_mA(2);
  power_mW_battery = ina219_battery.getBusPower();//busvoltage_battery * current_mA_battery;
  //Serial.print("current= ");
  //Serial.println(current_mA_battery);
  //loadvoltage_battery = busvoltage + (shuntvoltage / 1000);
}

void read_electrical_power_INA3221()//THIS FUNCTION IS CALLED BY CORE1, READ ANALOG INPUT FROM ADS1115, THE ANALOG INPUTS ARE V+ OF BOTH BATTERIES, THE VOLTAGE IS USED TO CALCULATE THE BATTERY STATUS
{
  /*
  //busvoltage1 = ina3221_rover.getBusVoltage_V(ACS712_CURRENT_BATTERY_CHANNEL);
  //shuntvoltage1 = ina3221_rover.getShuntVoltage_mV(ACS712_CURRENT_BATTERY_CHANNEL);
  //current_mA1 = -ina3221_rover.getCurrent_mA(ACS712_CURRENT_BATTERY_CHANNEL);  // minus is to get the "sense" right.   - means the battery is charging, + that it is discharging
  loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);
  power_load3 = loadvoltage3 * (current_mA3 / 1000);
  mah_battery = power_load3 / shuntvoltage1;
  power_load1 = shuntvoltage1 * (mah_battery / 1000);
  solar_efficiency = map(current_mA2, 0, 1000, 0, 100);
  max_current1 = current_mA1;
  
  if (diagnostic == ATTIVATO)
  {
      Serial.println("------------------------------");
      Serial.print("LIPO_Battery Bus Voltage:   "); Serial.print(busvoltage1); Serial.println(" V");
      Serial.print("LIPO_Battery Shunt Voltage: "); Serial.print(shuntvoltage1); Serial.println(" mV");
      Serial.print("LIPO_Battery Load Voltage:  "); Serial.print(loadvoltage1); Serial.println(" V");
      Serial.print("LIPO_Battery Current 1:       "); Serial.print(current_mA1); Serial.println(" mA");
      Serial.println("");
  }
  */

  busvoltage2 = ina3221_rover.getBusVoltage_V(SOLAR_CELL_CHANNEL);
  shuntvoltage2 = ina3221_rover.getShuntVoltage_mV(SOLAR_CELL_CHANNEL);
  current_mA2 = -ina3221_rover.getCurrent_mA(SOLAR_CELL_CHANNEL);
  loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);

  /*
  if(current_mA2 >= max_current2 || max_current2 == 0)
  {
    max_current2 = current_mA2;
  }
  if(current_mA2 <= min_current2 || min_current2 == 0)
  {
    min_current2 = current_mA2;
  } 

  if (diagnostic == ATTIVATO)
  {
      Serial.print("Solar Cell Bus Voltage 2:   "); Serial.print(busvoltage2); Serial.println(" V");
      Serial.print("Solar Cell Shunt Voltage 2: "); Serial.print(shuntvoltage2); Serial.println(" mV");
      Serial.print("Solar Cell Load Voltage 2:  "); Serial.print(loadvoltage2); Serial.println(" V");
      Serial.print("Solar Cell Current 2:       "); Serial.print(current_mA2); Serial.println(" mA");
      Serial.println("");
  }
  */

  busvoltage3 = ina3221_rover.getBusVoltage_V(OUTPUT_CHANNEL);
  shuntvoltage3 = ina3221_rover.getShuntVoltage_mV(OUTPUT_CHANNEL);
  current_mA3 = ina3221_rover.getCurrent_mA(OUTPUT_CHANNEL);
  loadvoltage3 = busvoltage3 + (shuntvoltage3 / 1000);

  /*
  if(current_mA3 >= max_current3 || max_current3 == 0)
  {
    max_current3 = current_mA3;
  }
  if(current_mA3 <= min_current3|| min_current3 == 0)
  {
    min_current3 = current_mA3;
  } 
  

  if (diagnostic == ATTIVATO)
  {
      Serial.print("Output Bus Voltage 3:   "); Serial.print(busvoltage3); Serial.println(" V");
      Serial.print("Output Shunt Voltage 3: "); Serial.print(shuntvoltage3); Serial.println(" mV");
      Serial.print("Output Load Voltage 3:  "); Serial.print(loadvoltage3); Serial.println(" V");
      Serial.print("Output Current 3:       "); Serial.print(current_mA3); Serial.println(" mA");
      Serial.println("");
  }
  */
  int bat_perc = map(busvoltage1 * 1000, 3200, 4200, 0, 100);
  int mah_bat = map(busvoltage1 * 1000, 3200, 4200, 0, 10000);
  residual_time_battery = mah_bat / mah_battery;
  
}

void lidar_stability_control(int uno, int due, int tre, int quattro, int cinque)
{
  //rp2040.idleOtherCore();
  uint8_t counter = 0;
  if(radar_sx_ant_45_aux == radar_sx_ant_45)
  {
    counter++;
  }else radar_sx_ant_45_aux = radar_sx_ant_45;
  
  if(radar_ant_aux == radar_ant)
  {
    counter++;
  }else radar_ant_aux = radar_ant;

  if(radar_dx_ant_45_aux == radar_dx_ant_45)
  {
    counter++;
  }else radar_dx_ant_45_aux = radar_dx_ant_45;
  
  if(radar_sx_center_aux = radar_sx_center)
  {
    counter++;
  }else radar_sx_center_aux = radar_sx_center;
  
  if(radar_dx_center_aux == radar_dx_center)
  {
    counter++;
  }else radar_dx_center_aux = radar_dx_center;

  if(counter == 5)
    counter_second_level++;
  //rp2040.resumeOtherCore();
}

void data_sender()                                                                                                                                                                                                                                                                                                          
{
  //Serial1.println("{\"rf\":" + String(radar_ant) + ", \"r45l\":" + String(radar_sx_ant_45) + ", \"r45r\":" + String(radar_dx_ant_45) + ", \"rs\":" + String(radar_sx_center) + + ", \"rd\":" + String(radar_dx_center) + ", \"v_1\":" + String(busvoltage_battery) + ", \"v_2\":" + String(busvoltage2) + ", \"v_3\":" + String(busvoltage3) + ", \"a_1\":" + String(current_mA_battery) + ", \"a_2\":" + String(current_mA2) + ", \"a_3\":" + String(current_mA3) + ", \"steer_pos\":" + String(posizione_sterzo) + ", \"run\":" + String(run_value) + ", \"panels_on_off\":" + String(POSIZIONE_PANNELLO_SX + 1) + String(POSIZIONE_PANNELLO_DX + 1)  + ", \"cam_sx_dx\":" + String(webcam_sinistra_destra) + ", \"cam_up_dn\":" + String(webcam_sopra_sotto) + ", \"rover_celsius\":" + String(analogReadTemp()) + ", \"bat_time\":" + String(residual_time_battery) + ", \"motor_speed\":" + String(mot_speed_perc) + "}"); 
  Serial1.println("telemetry," + String(radar_ant) + "," + String(radar_sx_ant_45) + "," + String(radar_dx_ant_45) + "," + String(radar_sx_center) + + "," + String(radar_dx_center) + "," + String(busvoltage_battery) + "," + String(busvoltage2) + "," + String(busvoltage3) + "," + String(current_mA_battery) + "," + String(current_mA2) + "," + String(current_mA3) + "," + String(posizione_sterzo) + "," + String(run_value) + "," + String(POSIZIONE_PANNELLO_SX + 1) + String(POSIZIONE_PANNELLO_DX + 1)  + "," + String(webcam_sinistra_destra) + "," + String(webcam_sopra_sotto) + "," + String(analogReadTemp()) + "," + String(residual_time_battery) + "," + String(mot_speed_perc) + "," + "ET"); 
  /*
  if(sys_msg == true)
  {
    Serial1.println("SYS_MSG," + String(sys_message) + ",SE");
    sys_msg = false;
  }
  */
  //Serial.println("{\"rf\":" + String(radar_ant) + ", \"r45l\":" + String(radar_sx_ant_45) + ", \"r45r\":" + String(radar_dx_ant_45) + ", \"rs\":" + String(radar_sx_center) + + ", \"rd\":" + String(radar_dx_center) + ", \"v_1\":" + String(busvoltage_battery) + ", \"v_2\":" + String(busvoltage2) + ", \"v_3\":" + String(busvoltage3) + ", \"a_1\":" + String(current_mA_battery) + ", \"a_2\":" + String(current_mA2) + ", \"a_3\":" + String(current_mA3) + ", \"steer_pos\":" + String(posizione_sterzo) + ", \"run\":" + String(run_value) + ", \"panels_on_off\":" + String(POSIZIONE_PANNELLO_SX + 1) + String(POSIZIONE_PANNELLO_DX + 1)  + ", \"cam_sx_dx\":" + String(webcam_sinistra_destra) + ", \"cam_up_dn\":" + String(webcam_sopra_sotto) + ", \"rover_celsius\":" + String(analogReadTemp()) + ", \"bat_time\":" + String(residual_time_battery) + ", \"motor_speed\":" + String(mot_speed_perc) + "}");
  //Serial.println(current_mA_battery);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       motor_speed
}

void loop1() { //LOOP FUNCTION FOR CORE1, STORE ALL LIDAR SENSOR AND VOLTAGE VALUES EVERY N MS
  if(reset_lidar_sensor == 1)
  {
    setID();
    reset_lidar_sensor = 0;
    turn_off_lidar_sensor = 0;
    Serial.println(F("Sensori Lidar reimpostati "));
    delay(1000);
  }

  if(turn_off_lidar_sensor == 1 && Lidar_enabled == 1)
  {
    Lidar_enabled = 0;
    disable_lidar_sensors();
  }

  if(turn_off_lidar_sensor == 0 && Lidar_enabled == 0)
    {
      setID();
      Lidar_enabled = 1;
    }

  if(turn_off_lidar_sensor == 0 && Lidar_enabled == 1)
     read_five_sensors();

  if(loop1_times == 5)
  {
    read_battery_power_INA219();
    read_electrical_power_INA3221();
    loop1_times = 0;
  }
  data_sender();

  loop1_times++;
  
  //Serial.print("Manufactures ID=0x");
  //int MID_rover;
  //MID_rover = ina3221_rover.getManufID();
  //SW6106();
  //int MID_battery;
  //Serial.print("Manufactures ID=0x");
  //MID_battery = //ina219_battery.getManufID();
  //Serial.println(MID_rover,HEX);
  //Serial.println(MID_battery,HEX);
  //delay(100);
  //anticollision();
  //rad_mot();
}

void cmd_kb(MyCommandParser::Argument *args, char *response) {
  //uint8_t cmd = args[0].asString;
  //Serial.print("string: "); Serial.println(args[0].asString);
  
  //Serial.print("double: "); Serial.println(args[1].asDouble);
  uint8_t cmd = (int32_t)args[0].asInt64;
  Serial.print("int64: "); Serial.println(cmd); // NOTE: on older AVR-based boards, Serial doesn't support printing 64-bit values, so we'll cast it down to 32-bit
  Serial.print("Cmd: "); Serial.println(cmd);
  //Serial.print("uint64: "); Serial.println((uint32_t)args[3].asUInt64); // NOTE: on older AVR-based boards, Serial doesn't support printing 64-bit values, so we'll cast it down to 32-bit
  strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
  Serial.println("Cmd from Keyboard");
  //String rover_cmd = myParser->getCmdParam(2);
  //uint8_t cmd = rover_cmd.toInt();
  //Serial.println(rover_cmd);
  //Serial.println(cmd);
  rover_commands(cmd);
}

void accelerator_control(uint8_t accelerator_value)
{
  if(accelerator_value != 50)
  {
    if(accelerator_value > 50)
    {
      mot_speed_perc = map( motor_speed, 0, 4050, 0, 100);
      motor_speed = map(accelerator_value, 50, 100, 200, 4050);
      forward(FORWARD);
    }else if(accelerator_value < 50)
    {
      mot_speed_perc = map( motor_speed, 0, 4050, 0, 100);
      motor_speed = map(accelerator_value, 50, 0, 200, 4050);
      forward(BACK);
    }
  }else {
      forward(STOP);
  } 
}

uint16_t steer_aux = 0;
uint16_t cam_x_aux = 0;
uint16_t cam_y_aux = 0;
uint16_t acc_z_aux = 0;

void cmd_js(MyCommandParser::Argument *args, char *response) {

  strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
  //Serial.println("Cmd from Keyboard");
  Serial.println("Cmd from Joystick");
  uint16_t steer = (int32_t)args[0].asInt64;
  //uint16_t steer = steer.toInt();
  Serial.println(steer);
  uint16_t cam_x = (int32_t)args[1].asInt64;
  //uint16_t cam_x = cam_x.toInt();
  Serial.println(cam_x);
  uint16_t cam_y = (int32_t)args[2].asInt64;
  //uint16_t cam_y = cam_y.toInt();
  Serial.println(cam_y);
  uint16_t acc_z = (int32_t)args[3].asInt64;
  //uint16_t acc_z = acc_z.toInt();
  Serial.println(acc_z);
  //uint8_t rover_cmd = (int32_t)args[4].asInt64;;
  //uint16_t rover_cmd_int = rover_cmd.toInt();
  //Serial.println(rover_cmd_int);
  if(steer != steer_aux)
    {
        posizione_sterzo = steer + trimmer;
        sterzo(posizione_sterzo);
        steer_aux = steer;
    }
  if(cam_x != cam_x_aux)
    {
        webcam_sinistra_destra = cam_x;
        //webcam_sopra_sotto = cam_y;
        webcam_control();
        cam_x_aux = cam_x;
        //cam_y_aux = cam_y;
    }
  if(cam_y != cam_y_aux)
    {
        //webcam_sinistra_destra = cam_x;
        webcam_sopra_sotto = cam_y;
        webcam_control();
        //cam_x_aux = cam_x;
        cam_y_aux = cam_y;
    }
  if(acc_z != acc_z_aux)
    {
        accelerator = acc_z;
        accelerator_control(acc_z);
        acc_z_aux = acc_z;
    }
}

void setup() {   //SETUP FOR CORE0
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(115200);
  
  SPI.setRX(SPI_MISO_PIN);
  SPI.setTX(SPI_MOSI_PIN);
  SPI.setSCK(SPI_SCK_PIN);
  SPI.setCS(SPI_CS_PIN);
  EEPROM.begin(512);
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();

  Serial.println( F("Welcome to the SilverRover"));
  Serial.println( F("Inserisci il comando da inviare al Rover."));
  Serial.println( F("scrivi 'help' per vedere i comandi disponibili"));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  time = millis();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PCA_ON_OFF_PIN, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(22, OUTPUT);
  //panels_positions_data_load();
  //load_trimmer();
  //load_odo();
  //load_odo_eeprom();
  sterzo_dritto();
  ruote_sterzanti_post = DISATTIVATO;
  
  //pca_on_off();//disattiva pca
     
   // update received data directly
   //MsgPacketizer::subscribe(Serial1, 0x12/*recv_index*/, posizione_sterzo, webcam_sinistra_destra, webcam_sopra_sotto, accelerator, str_command);
  //NEW
  parser.registerCommand("KB", "i", &cmd_kb);
  parser.registerCommand("JS", "iiii", &cmd_js);
  //NEW
  //cmdCallback.addCmd(PSTR("set"), &functSet);
  
}

void load_trimmer() //LOAD TRIMMER SETTING STORED ON EEPROM(10)
{
  //rp2040.idleOtherCore();
  int trim_ = EEPROM.read(10);
  //rp2040.resumeOtherCore();
  if(trim_ >= -10 && trim_ <= 10)
  {
  trimmer = trim_;
  if (diagnostic == ATTIVATO)
    {
      Serial.print(F("Trimmer caricato: "));
      Serial.println(trimmer);
    }
  }else trimmer = 0;
}

void panels_positions_data_load()
{
  //rp2040.idleOtherCore();
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
      Serial.print(F("DX: "));
      Serial.println(POSIZIONE_PANNELLO_DX);
      Serial.print(F("SX: "));
      Serial.println(POSIZIONE_PANNELLO_SX);
    }
  }
  //rp2040.resumeOtherCore();
}

void panels_positions_data_save() //LOAD PANNELS POSITIONS STORED ON EEPROM(0) AND EEPROM(1)
{
  //rp2040.idleOtherCore();
  EEPROM.write(0, POSIZIONE_PANNELLO_DX);
  EEPROM.write(1, POSIZIONE_PANNELLO_SX);
  
  if (EEPROM.commit())
  {
    if (diagnostic == ATTIVATO)
    {
      Serial.println("EEPROM successfully committed");
    }
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }
  //rp2040.resumeOtherCore();
}

void pca_on_off()  //FUNCTION TO ENABLE / DISABLE PCA9685 MODULE TO SAVE ENERGY ON IDLE TIMES
{
  if (pca_off == DISATTIVATO)
  {
    pca_off = ATTIVATO;
    digitalWrite(PCA_ON_OFF_PIN, HIGH);
    Serial.println(F("Motori disattivati"));
  } else if (pca_off == ATTIVATO)
  {
    pca_off = DISATTIVATO;
    digitalWrite(PCA_ON_OFF_PIN, LOW);
    Serial.println(F("Motori attivati"));
  }
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println( F(" us per period"));
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println( F(" us per bit"));
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
}

void forward(uint16_t comand) //FUNCTION TO MANAGE THE CONTINUOUS SERVO MOTORS TO MOVE ROVER FORWARD, BACK AND STOP IT
{
  if (comand == STOP)
  {
    fine_conteggio_distanza = millis();
    pwm.setPWM(WHEEL_FORWARD_LEFT, 0, STOP);
    pwm.setPWM(WHEEL_BACKWARD_LEFT, 0, STOP);
    pwm.setPWM(WHEEL_FORWARD_RIGHT, 0, STOP);
    pwm.setPWM(WHEEL_BACKWARD_RIGHT, 0, STOP);
    /*
    pwm.writeMicroseconds(RUOTA_ANT_SX, STOP);//-30 //ANT_SX -10 //4
    pwm.writeMicroseconds(RUOTA_ANT_DX, STOP);//-50 //ANT_DX //5
    pwm.writeMicroseconds(RUOTA_POST_DX, STOP);//-150q //POST_DX //6
    pwm.writeMicroseconds(RUOTA_POST_SX, STOP);//POST_SX -> SCOLLEGATO  - 50 //7
    */
    calcolo_distanza_percorsa();
    if(avanzamento_c == ATTIVATO)
         avanzamento_c = DISATTIVATO;
    if(indietro_c == ATTIVATO)
         indietro_c == DISATTIVATO;
  } else if (comand == FORWARD)
  {
    inizio_conteggio_distanza = millis();
    pwm.setPWM(WHEEL_FORWARD_LEFT, 0, motor_speed);
    pwm.setPWM(WHEEL_BACKWARD_LEFT, 0, STOP);
    pwm.setPWM(WHEEL_FORWARD_RIGHT, 0, STOP);
    pwm.setPWM(WHEEL_BACKWARD_RIGHT, 0, motor_speed);
    /*
    pwm.writeMicroseconds(RUOTA_ANT_SX, FORWARD - motor_speed  + 100);//ANT_SX - ant_sx_speed
    pwm.writeMicroseconds(RUOTA_ANT_DX, BACK + motor_speed - 100);//ANT_DX  - 100 + ant_dx_speed
    pwm.writeMicroseconds(RUOTA_POST_DX, BACK + motor_speed );//POST_DX + post_dx_speed
    pwm.writeMicroseconds(RUOTA_POST_SX, FORWARD - motor_speed );//POST_SX -> SCOLLEGATO  - 465 - post_sx_speed
    */
  } else if (comand == BACK)
  {
    inizio_conteggio_distanza = millis();
    pwm.setPWM(WHEEL_FORWARD_LEFT, 0, STOP);
    pwm.setPWM(WHEEL_BACKWARD_LEFT, 0, motor_speed);
    pwm.setPWM(WHEEL_FORWARD_RIGHT, 0, motor_speed);
    pwm.setPWM(WHEEL_BACKWARD_RIGHT, 0, STOP);
    /*
    pwm.writeMicroseconds(RUOTA_ANT_SX, BACK + motor_speed );//ANT_SX + ant_sx_speed
    pwm.writeMicroseconds(RUOTA_ANT_DX, FORWARD - motor_speed );//ANT_DX - ant_dx_speed
    pwm.writeMicroseconds(RUOTA_POST_DX, FORWARD - motor_speed );//POST_DX - post_dx_speed
    pwm.writeMicroseconds(RUOTA_POST_SX, BACK + motor_speed - 200);//POST_SX -> SCOLLEGATO  + 355 - post_sx_speed 
    */
  }
}

bool zona_radar(uint16_t value) //THIS FUNCTION PREVENT THE COLLISION BETWEEN THE CAMERA AND FRONT LIDAR SENSORS
{
  if (value > 230 && value < 400)
    return true;
  return false;
}

void webcam_control() //FUNCTION TO CONTROL CAMERA
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
  webcam_sinistra_destra = pulselen - 50;//60;
  webcam_sopra_sotto = 345; // + 15;
  pwm.setPWM(15, 0, webcam_sinistra_destra);
  pwm.setPWM(14, 0, webcam_sopra_sotto);
}

void sterzo_anteriore(uint16_t pulselen) //FUNCTION TO MANAGE FRONT STEER
{
  pulselen = pulselen + trimmer - 11;
  int p_p = map(pulselen, limite_sinistro, limite_destro, 252 , 410);// 264 , 397);
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
    pwm.setPWM(0, 0, p_p - 8); //-5 p_p
    pwm.setPWM(1, 0, pulselen + 80); // pulselen
    //============================================
    //ant_dx_speed = map(pulselen, centro, limite_destro, 0, 270);//ant_dx_int;
    //post_dx_speed = map(pulselen, centro, limite_destro, 0, 380);//post_dx_int;
    //post_sx_speed = map(pulselen, centro, limite_destro, 0, 20);//post_sx_ext;
    //if(avanzamento_c == ATTIVATO){
           //forward(FORWARD);
        //}
    
    //============================================
  }else if (pulselen < centro)
  {
    pwm.setPWM(0, 0, pulselen - 8); //-5 p_p
    pwm.setPWM(1, 0, p_p + 80); //pulselen
    //============================================
    //ant_sx_speed =  map(pulselen, limite_sinistro, centro, 270, 0);//ant_sx_int;
    //post_sx_speed = map(pulselen, limite_sinistro, centro,  30, 0);//post_sx_int;//post_int;
    //post_dx_speed = map(pulselen, limite_sinistro, centro, 200, 0);//post_dx_ext;
    //if(avanzamento_c == ATTIVATO){
           //forward(FORWARD);
        //}
    //============================================
  } else {
    pwm.setPWM(0, 0, pulselen - 8); //-5
    pwm.setPWM(1, 0, pulselen + 80);
    ant_sx_speed = 0;
    ant_dx_speed = 0;//ant_int;
    post_dx_speed = 0;//post_int;
    post_sx_speed = 0;//post_ext;
    //if(avanzamento_c == ATTIVATO){
           //forward(FORWARD);
        //}
  }
  if(diagnostic == ATTIVATO)
  {
      Serial.print("ANT SX SPEED: ");
      Serial.println(ant_sx_speed);
      Serial.print("POST SX SPEED (Modded servo): ");
      Serial.println(post_sx_speed);
      Serial.print("ANT DX SPEED: ");
      Serial.println(ant_dx_speed);
      Serial.print("POST DX SPEED: ");
      Serial.println(post_dx_speed);
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
  pwm.setPWM(3, 0, pulselen - 30);

}

void sterzo(uint16_t angolo_serzata)
{
  uint16_t pulselen_ant = angolo_serzata + 6;
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
      webcam_X = 0;
      webcam_Y = 0;
    } else if (posizione_sterzo > DRITTO)
    {
      for (uint16_t i = posizione_sterzo; i >= DRITTO; i--)
      {
        posizione_sterzo = i;
        sterzo(posizione_sterzo);
        delay(2);
      }
      webcam_X = 0;
      webcam_Y = 0;
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

void panel_rear_da_chiuso_a_aperto()
{
  if (POSIZIONE_PANNELLO_REAR == CHIUSO)
  {
    for (uint16_t i = 520; i > 70; i--) {
      pwm.setPWM(11, 0, i/*i*/);
      panel_position_rear = i;
      //pwm.setPWM(11, 0, i/*i*/);
      delay(panel_speed);
    }
    pwm.setPWM(11, 0, 0/*i*/);
    POSIZIONE_PANNELLO_REAR = APERTO;
  }
}

void panel_rear_da_aperto_a_chiuso()
{
  if (POSIZIONE_PANNELLO_REAR == APERTO)
  {
    for (uint16_t i = 70; i < 520; i++) {
      pwm.setPWM(11, 0, i/*i*/);
      panel_position_rear = i;
      delay(panel_speed);
    }
    pwm.setPWM(11, 0, 0);
    POSIZIONE_PANNELLO_REAR = CHIUSO;
  }
}

void panel_dx_da_chiuso_a_meta()
{
  if (POSIZIONE_PANNELLO_DX == CHIUSO)
  {
    for (uint16_t i = 520; i > 300; i--) {
      pwm.setPWM(8, 0, i/*i*/);
      panel_position_dx = i;
      //pwm.setPWM(9, 0, i/*i*/);
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
      //pwm.setPWM(9, 0, i/*i*/);
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
      //pwm.setPWM(9, 0, i/*i*/);
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
    for (uint16_t i = 450; i < 760; i++/*int i=0;i<240;i++ 640*/)
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
    for (uint16_t i = 760; i > 450; i--)
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
  pwm.setPWM(11, 0, 0/*i*/);

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
      if (panel_position_sx < 700)//660)
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
  if (POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO && peso_sinistra == DISATTIVATO && peso_destra == DISATTIVATO && POSIZIONE_PANNELLO_REAR == CHIUSO)
  {
    pwm.setPWM(15, 0, 299);
    delay(500);
    panel_rear_da_chiuso_a_aperto();// NEWWWWWWWWWWWWWWWWWW
    panel_dx_da_chiuso_a_meta();
    Serial.println( F("Pannello destro aperto a metà"));
    panel_sx_da_chiuso_a_meta();
    webcam_control();
    Serial.println( F("Pannello sinistro aperto a metà"));
    panel_sx_dx_da_meta_a_aperto();
    //          POSIZIONE_PANNELLO_SX = 0;
    Serial.println( F("Entrambi i pannelli aperti"));
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
  //Serial.println( F("azioni Pannello sx effettuate"));
}

void close_panels()
{
  if (POSIZIONE_PANNELLO_DX == APERTO && POSIZIONE_PANNELLO_SX == APERTO && peso_sinistra == DISATTIVATO && peso_destra == DISATTIVATO  && POSIZIONE_PANNELLO_REAR == APERTO)
      {
            panel_sx_dx_da_apero_a_meta();
            Serial.println( F("Pannello sinistro aperto a metà"));
            Serial.println( F("Pannello destro aperto a metà"));
            pwm.setPWM(15, 0, 299);
            panel_sx_da_meta_a_chiuso();
            panel_dx_da_meta_a_chiuso();
            panel_rear_da_aperto_a_chiuso();// NEWWWWWWWWWWWWWWWWWW
            delay(200);
            webcam_control();
            //          POSIZIONE_PANNELLO_SX = 0;
            Serial.println( F("Entrambi i pannelli chiusi"));
            Serial.println( F("azioni Pannello sx effettuate"));
            POSIZIONE_PANNELLO_DX = CHIUSO;
            POSIZIONE_PANNELLO_SX = CHIUSO;
            POSIZIONE_PANNELLO_REAR = CHIUSO;
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
    Serial.println( F("Pannello sinistro aperto a metà"));
    panel_sx_da_meta_a_aperto();
    //POSIZIONE_PANNELLO_SX = APERTO ;
    Serial.println( F("Pannello sinistro aperto completamente"));
  } else if (POSIZIONE_PANNELLO_SX == APERTO )
  {
    panel_sx_da_aperto_a_meta();
    Serial.println( F("Pannello sinistro aperto a metà"));
    //panel_dx_da_chiuso_a_meta();

    panel_sx_da_meta_a_chiuso();
    //POSIZIONE_PANNELLO_SX = 0;
    if (POSIZIONE_PANNELLO_SX == 0)
      Serial.println( F("Pannello sinistro 0"));
  } else if (POSIZIONE_PANNELLO_SX == 2)
  {
    panel_sx_da_meta_a_chiuso();
    //POSIZIONE_PANNELLO_SX = 0;
    if (POSIZIONE_PANNELLO_SX == 0)
      Serial.println( F("Pannello sinistro 0"));
  }
  Serial.println( F("Azioni pannello sx effettuate"));
}

void right_panel()
{
  if (POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX != 0)
  {
    panel_dx_da_chiuso_a_meta();
    Serial.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_aperto();
    //POSIZIONE_PANNELLO_DX = APERTO ;
    Serial.println( F("Pannello destro completamente aperto"));
  } else if (POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX != 0)
  {
    panel_dx_da_aperto_a_meta();
    Serial.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_chiuso();
    if (POSIZIONE_PANNELLO_DX == 0)
      Serial.println( F("Pannello destro 0"));
  } else if (POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX == 0)
  {
    panel_sx_da_chiuso_a_meta();
    panel_dx_da_aperto_a_meta();
    Serial.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_chiuso();
    panel_sx_da_meta_a_chiuso();
    if (POSIZIONE_PANNELLO_DX == 0)
      Serial.println( F("Pannello destro 0"));
  } else if (POSIZIONE_PANNELLO_DX == 2 && POSIZIONE_PANNELLO_SX != 0)
  {
    //panel_dx_da_aperto_a_meta();
    Serial.println( F("Pannello destro aperto a metà"));
    panel_dx_da_meta_a_chiuso();
    if (POSIZIONE_PANNELLO_DX == 0)
      Serial.println( F("Pannello destro 0"));
  }
  Serial.println( F("azioni Pannello dx effettuate"));
}

void radar_motor(uint16_t angle_motor)
{
  pwm.setPWM(12, 0, angle_motor + 15);
}

void peso_a_sinistra()
{
  Serial.println( F("1"));
  if (peso_sinistra == 0)
  {
    //Serial.println( F("2"));
    if (POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO)
    {
      panel_sx_da_chiuso_a_meta();
      panel_sx_da_meta_a_aperto();
      peso_sinistra = ATTIVATO;
      //Serial.println( F("3"));
    } else if (POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX == APERTO)
    {
      //Serial.println( F("4"));
      panel_dx_da_aperto_a_meta();
      panel_dx_da_meta_a_chiuso();
      peso_sinistra = ATTIVATO;
    }

  } else if (peso_sinistra == ATTIVATO)
  {
    //Serial.println( F("2"));
    if (stato_precedente == 0)
    {
      panel_sx_da_aperto_a_meta();
      panel_sx_da_meta_a_chiuso();
      peso_sinistra = DISATTIVATO;
      //Serial.println( F("3"));
    } else if (stato_precedente == APERTO )
    {
      //Serial.println( F("4"));
      panel_dx_da_chiuso_a_meta();
      panel_dx_da_meta_a_aperto();
      peso_sinistra = DISATTIVATO;
    }
  }
  //Serial.println( F("5"));
}//*/

void peso_a_destra() // muove i pannelli solari in modo da ottenere maggior peso verso destra per evitare eventuali ribaltamenti dovuti alla pendenza
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
    Serial.println( F("peso destro attivo 2"));
    if (stato_precedente == CHIUSO)
    {
      //Serial.println(stato_precedente);
      panel_sx_da_chiuso_a_meta();
      panel_dx_da_aperto_a_meta();
      panel_dx_da_meta_a_chiuso();
      panel_sx_da_meta_a_chiuso();
      Serial.println( F("3"));
      peso_destra = DISATTIVATO;
    } else if (stato_precedente == APERTO)
    {
      Serial.println(stato_precedente);
      panel_sx_da_chiuso_a_meta();
      panel_sx_da_meta_a_aperto();
      peso_destra = DISATTIVATO;
    }
    //peso_sinistra = DISATTIVATO;
  }
}

void load_odo()
{
  //rp2040.idleOtherCore();
  int dist_perc = EEPROM.read(3);
  uint8_t divisore = EEPROM.read(4);
  //rp2040.resumeOtherCore();
  distanza_percorsa = dist_perc * divisore;
}

void load_odo_eeprom()
{
  //EEPROM.readFloat(2);
  float value = 0;
  //rp2040.idleOtherCore();
  EEPROM.get(3, value);
  //rp2040.resumeOtherCore();
  distanza_percorsa = value;
  //  Serial.print(F("DISTANZA CARICATA: "));
  //    Serial.println(distanza_percorsa);
  if (diagnostic == ATTIVATO)
  {
    Serial.print(F("DISTANZA CARICATA: "));
    Serial.println(distanza_percorsa);
  }

}

void save_odo(int distanza_percorsa_odo)
{
  //rp2040.idleOtherCore();
  EEPROM.put(3, distanza_percorsa_odo);
  //rp2040.resumeOtherCore();
  //EEPROM.put(4, divisore);

}

void save_odo_eeprom(float distanza_percorsa_odo)
{
  //rp2040.idleOtherCore();
  EEPROM.put(3, distanza_percorsa_odo);
  //rp2040.resumeOtherCore();
  if (diagnostic == ATTIVATO)
  {
    Serial.println(F("DISTANZA SALVATA!"));
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
  //save_odo(distanza_percorsa);
}

void avanzamento_continuo()
{
  avanzamento_c = ATTIVATO;
  forward(FORWARD);
}

void avanti(int run_delay_1)
{
  if (indietro_c == ATTIVATO)
  {
    forward(STOP);
    indietro_c = DISATTIVATO;
  } else if (indietro_c == DISATTIVATO)
  {
      if (posizione_sterzo == DRITTO)
      {
        //radar_scan_180[361];
        Serial.print( F("Distanza: "));
        Serial.println(radar_ant);
        if (/*distance_ant>520 && */run_delay_1 == 21000)
        {
          forward(FORWARD);
          Serial.println( F("Avanti per 5mt"));
          delay(run_delay_1);
          forward(STOP);
        } else if (/*distance_ant>55 && */run_delay_1 == 0)
        {
          forward(FORWARD);
          Serial.println( F("Avanti per 50cm "));
          delay(2100);
          forward(STOP);
        } else if (/*distance_ant>12 && */run_delay_1 == 52)
        {
          forward(FORWARD);
          Serial.println( F("Avanti per 1cm "));
          delay(run_delay_1);
          forward(STOP);
        }
        else if (/*distance_ant<55 && distance_ant>25 && */run_delay_1 == 0)
        {
          forward(FORWARD);
          Serial.println( F("Avanti per 25cm"));
          delay(1050);
          forward(STOP);
        }
      } else if (posizione_sterzo > DRITTO)
      {
        if (run_delay_1 != 52)
        {
          forward(FORWARD);
          Serial.println( F("Avanti"));
          delay(1050);// indietreggia di 25 cm verso sinistra
          forward(STOP);
        } else if (/*distance_ant>12 && */run_delay_1 == 52/* && distance_sx>=8*/)
        {
          forward(FORWARD);
          Serial.println( F("Avanti per 1cm "));
          delay(run_delay_1);
          forward(STOP);
        }
      } else if (posizione_sterzo < DRITTO)
      {           
        if (run_delay_1 != 52)
        {
          forward(FORWARD);
          Serial.println( F("Avanti"));
          delay(1050);// indietreggia di 25 cm verso sinistra
          forward(STOP);
        } else if (/*distance_ant>12 && */run_delay_1 == 52/* && distance_sx>=8*/)
        {
          forward(FORWARD);
          Serial.println( F("Avanti per 1cm "));
          delay(run_delay_1);
          forward(STOP);
        }
      }
  }
}

void indietro()
{
  if (avanzamento_c == ATTIVATO)
  {
    forward(STOP);
  } else if (avanzamento_c == DISATTIVATO)
  {
    int distance_post = 0;
    int distance_sx = 0;
    int distance_dx = 0;

    if (posizione_sterzo == DRITTO)
    {
      forward(BACK);
      Serial.println( F("Indietro"));
      delay(525);
      forward(STOP);
    } else if (posizione_sterzo > DRITTO)
    {
      forward(BACK);
      Serial.println( F("Indietro"));
      delay(525);// indietreggia di 12,5 cm verso destra
      forward(STOP);
    } else if (posizione_sterzo < DRITTO)
    {           
      forward(BACK);
      Serial.println( F("Indietro"));
      delay(1050);// indietreggia di 25 cm verso sinistra
      forward(STOP);
    }
  }
}

void indietro_continuo()
{
  indietro_c = ATTIVATO;
  forward(BACK);
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
      if(radar_sx_center<200)
      {
         return true;//AVANTI
      }//SINISTRA
      return false;
}

bool radar_dx_alarm()
{
      if(radar_dx_center<200)
      {
         return true;//AVANTI
      }//DESTRO
      return false;
}

int lunghezza_int(int input)
{
  int count = 0;
  while (input > 0) {
    input /= 10;
    count++;
  }
  return count;
}

void radar_s()
{
                int spazi = 0;
                Serial.println(F("+=============|RADAR|=====================+"));
                  Serial.print(F("|45:"));
                Serial.print(radar_sx_ant_45);
                Serial.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_sx_ant_45);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial.print(F(" "));
                  }
                Serial.print(F("A: "));
                Serial.print(radar_ant);
                Serial.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_ant);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial.print(F(" "));
                  }
                Serial.print(F("45: "));
                Serial.print(radar_dx_ant_45);
                Serial.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_dx_ant_45);
               for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial.print(F(" "));
                  }
                spazi = 6 - lunghezza_int(radar_dx_ant_45);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial.print(F(" "));
                  }
                Serial.println(F("|"));
                Serial.println(F("+=========================================+"));
                delay(radar_delay);
                Serial.print(F("|S: "));
                Serial.print(radar_sx_center);
                Serial.print(F("mm"));
                spazi = 10 - lunghezza_int(radar_sx_center);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial.print(F(" "));
                  }
                delay(radar_delay);
                //Serial.print(F("F               "));2000mm  45
                Serial.print(F("        D: "));    
                Serial.print(radar_dx_center);
                Serial.print(F("mm"));
                spazi = 6 - lunghezza_int(radar_dx_center);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial.print(F(" "));
                  }
                Serial.println(F("|"));
                Serial.println(F("+=========================================+"));
                //Serial.println(F("|"));
                delay(radar_delay);
                Serial.print(F("|"));
                Serial.print(F("            D: "));
                Serial.print(radar_post);
                Serial.print(F("mm"));
                spazi = 21 - lunghezza_int(radar_post);
                for(uint8_t i=0;i<spazi;i++)
                  {
                  Serial.print(F(" "));
                  }
                Serial.println(F("|"));
                Serial.println(F("+=========================================+"));
}

void stop_motors_wheel()
{
  forward(STOP);
  if(avanzamento_c == ATTIVATO)
    avanzamento_c = DISATTIVATO;
  /*
  for(uint8_t i=4; i<16; i++)
  {
    pwm.setPWM(i, 0, 0);
  }
  disabled_motors = true;
  */
}

void stampa_distanza_percorsa()
{
  Serial.print(F("Distanza totale percorsa: "));
  if (distanza_percorsa < 100) {
      Serial.print(distanza_percorsa);
      Serial.println(F(" cm"));
    } else {
      Serial.print(distanza_percorsa);
      Serial.println(F(" cm"));
    }
}

void rover_commands(uint8_t str)
{
  if (str == 97) //a
  {
    int p = 0;
    if (posizione_sterzo - valore_sterzata >= STERZO_SINISTRA_MAX)
    {
      posizione_sterzo = posizione_sterzo - valore_sterzata;
      sterzo(posizione_sterzo);
      Serial.print( F("Sterzo: "));
      Serial.println(posizione_sterzo/*posizione_sterzo*/);
      //delay(100);
    }
  } else if (str == 100) //d
  {
    int p = 0;
    if (posizione_sterzo + valore_sterzata <= STERZO_DESTRA_MAX)
    {
      posizione_sterzo = posizione_sterzo + valore_sterzata;
      sterzo(posizione_sterzo);
      Serial.print( F("Sterzo: "));
      Serial.println(posizione_sterzo/*posizione_sterzo*/);
      //delay(100);
    }
  } else if (str == 104) //h
  {
    int p = 0;
    if (posizione_sterzo + valore_sterzata <= STERZO_DESTRA_MAX)
    {
      posizione_sterzo = STERZO_DESTRA_MAX;
      sterzo(posizione_sterzo);
      Serial.print( F("Sterzo: "));
      Serial.println(posizione_sterzo/*posizione_sterzo*/);
      //delay(100);
    }
  } else if (str == 103) //g
  {
    int p = 0;
    if (posizione_sterzo - valore_sterzata >= STERZO_SINISTRA_MAX)
    {
      posizione_sterzo = STERZO_SINISTRA_MAX;
      sterzo(posizione_sterzo);
      Serial.print( F("Sterzo: "));
      Serial.println(posizione_sterzo/*posizione_sterzo*/);
      //delay(100);
    }
  } else if (str == 119) { //w
    avanti(0);
  } else if (str == 115) { //s
    indietro();
  } else if (str == 101) { //e
    //anticollisione = ATTIVATO;
    //Serial.print( F("Anticollisione attivata automaticamente"));
    avanzamento_continuo();

  } else if (str == 121) { //y
    indietro_continuo();
  } else if (str == 113) { //q
    avanti(52);
  }else if (str == 52) { //4
    if (ruote_sterzanti_post == ATTIVATO && ruote_sterzanti_ant == ATTIVATO)
    {
      ruote_sterzanti_post = DISATTIVATO;
      sterzo(STERZO_DRITTO);
      Serial.println( F("Sterzo posteriore disattivato"));
      Serial1.println( F("SYS_MSG,Sterzo posteriore disattivato,ES"));

    } else if (ruote_sterzanti_post == DISATTIVATO)
    {
      sterzo(STERZO_DRITTO);
      ruote_sterzanti_post = ATTIVATO;
      Serial.println( F("Sterzo posteriore attivato"));
      Serial1.println( F("SYS_MSG,Sterzo posteriore attivato,ES"));
    } else {
      Serial.println( F("Impossibile disattivare lo sterzo posteriore: attiva lo sterzo anteriore per disattivare il posteriore"));
    }
  }
  else if (str == 112) { //p
    if(POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO)
    {
    if (anticollisione == ATTIVATO)
    {
      //radar_update();
      if (radar_dx_center > 200 && radar_sx_center > 200)
      {
        
        open_panels();
        panels_security_distance = 100;
      }else {
      Serial.println(F("Possibili ostacoli ai lati del rover"));
      Serial1.println( F("SYS_MSG,Possibili ostacoli ai lati del rover,ES"));
    }
    } else open_panels();
  } else if (POSIZIONE_PANNELLO_DX == APERTO && POSIZIONE_PANNELLO_SX == APERTO) {
    close_panels();
    panels_security_distance = 0;
  }else {
      POSIZIONE_PANNELLO_DX = CHIUSO;
      POSIZIONE_PANNELLO_SX = CHIUSO;
      Serial.println(F("Valori della posizione dei pannelli anomali, valori consentiti ripristinati."));
    } 
  } else if (str == 114) { //r
    radar_s();
  } else if (str == 49) { //1
    if (webcam_sterzo == DISATTIVATO)
    {
      webcam_sterzo = ATTIVATO;
      Serial.println(F("Webcam Sterzo attivato"));
    }
    else if (webcam_sterzo = ATTIVATO)
    {
      webcam_sterzo = DISATTIVATO;
      Serial.println(F("Webcam Sterzo disattivato"));
    }
  } else if (str == 53) { //5
    if (anticollisione == DISATTIVATO)
    {
      anticollisione = ATTIVATO;
      turn_off_lidar_sensor = 0;
      //reset_lidar_sensor = 1;
      //Serial_radar.print('5');
      Serial.println(F("Anticollisione attivata"));
    }
    else if (anticollisione = ATTIVATO)
    {
      anticollisione = DISATTIVATO;
      turn_off_lidar_sensor = 1;
      Serial.println(F("Anticollisione disattivata"));
    }
  } else if (str == 81) { //
    if (diagnostic == DISATTIVATO)
    {
      diagnostic = ATTIVATO;
      Serial.println(F("Diagnostica attivata"));
    }
    else if (diagnostic = ATTIVATO)
    {
      diagnostic = DISATTIVATO;
      Serial.println(F("Diagnostica disattivata"));
    }
  } else if (str == 52) { //4
    _webcam_sterzo(330);
  } else if (str == 50) { //50
    sterzo_dritto();
  } else if (str == 54) { //6
    if (sterzo_automatico == DISATTIVATO)
    {
      sterzo_automatico = ATTIVATO;
      Serial.println(F("Sterzo automatico attivato"));
    }
    else if (sterzo_automatico = ATTIVATO)
    {
      sterzo_automatico = DISATTIVATO;
      Serial.println(F("Sterzo automatico disattivato"));
    }
  }/**/else if (str == 110) //n
  {
    //radar_update();
    if (radar_sx_ant > 200 && radar_sx_center > 200)
      peso_a_sinistra();
    else {
      Serial.println(F("Possibili ostacoli a sinistra del rover"));
      Serial1.println( F("SYS_MSG,Possibili ostacoli a sinistra del rover,ES"));
    }

  } else if (str == 109) //m
  {
    //radar_update();
    if (radar_dx_center > 200)
      peso_a_destra();
    else {
      Serial.println(F("Possibili ostacoli a sinistra del rover"));
      Serial1.println( F("SYS_MSG,Possibili ostacoli a sinistra del rover,ES"));
    }
  } else if (str == 105) //i
  {
    if (webcam_sinistra_destra > 235 && webcam_sinistra_destra < 355 && webcam_sopra_sotto + 15 > 435) {
      Serial.println(F("Movimento massimo cam raggiunto"));
    } else {
      webcam_sopra_sotto = webcam_sopra_sotto + 15;
      webcam_control();
      webcam_Y--;
      Serial.print(F("Posizione Cam ( "));
      Serial.print(webcam_X);
      Serial.print(F(", "));
      Serial.print(webcam_Y);
      Serial.println(F(" )"));
    }

  } else if (str == 107) //k
  {
    //                if(webcam_sopra_sotto_effettuato == 0 && webcam_sopra_sotto_old > 435)
    //                {
    //                  webcam_sopra_sotto_effettuato = 1;
    //}
    if(webcam_sopra_sotto - 15<=180)
    {
      Serial.println(F("Movimento massimo cam raggiunto"));
    }else{
        webcam_sopra_sotto = webcam_sopra_sotto - 15;
        webcam_control();
        webcam_Y++;
        Serial.print(F("Posizione Cam ( "));
        Serial.print(webcam_X); //webcam_sinistra_destra
        Serial.print(F(", ")); 
        Serial.print(webcam_Y); //webcam_sopra_sotto
        Serial.println(F(" )"));
    }

  } else if (str == 111) //o
  {
    if(webcam_sinistra_destra != 115)
    {
        webcam_sinistra_destra = 115;
        webcam_control();
        webcam_X = +13;
        Serial.print(F("Posizione Cam ( "));
        Serial.print(webcam_X); //webcam_sinistra_destra
        Serial.print(F(", "));
        Serial.print(webcam_Y); //webcam_sopra_sotto
        Serial.println(F(" )"));
    } else Serial.println(F("La CAM e' gia' in questa posizione"));

  } else if (str == 117) //u
  {
    if(webcam_sinistra_destra != 505)
    {
        webcam_sinistra_destra = 505;
        webcam_control();
        webcam_X = -13;
        Serial.print(F("Posizione Cam ( "));
        Serial.print(webcam_X); //webcam_sinistra_destra
        Serial.print(F(", "));
        Serial.print(webcam_Y); //webcam_sopra_sotto
        Serial.println(F(" )"));
    }else Serial.println(F("La CAM e' gia' in questa posizione"));
  } else if (str == 108) //l
  {
    if(webcam_sinistra_destra - 15>=85)//115)
    {
        webcam_sinistra_destra = webcam_sinistra_destra - 15;
        webcam_control();
        webcam_X++;
        Serial.print(F("Posizione Cam ( "));
        Serial.print(webcam_X); //webcam_sinistra_destra
        Serial.print(F(", "));
        Serial.print(webcam_Y); //webcam_sopra_sotto
        Serial.println(F(" )"));
    } else Serial.println(F("Limite rotazione CAM raggiunto"));

  } else if (str == 106) //j
  {
    if(webcam_sinistra_destra + 15<=535)//505)
    {
        webcam_sinistra_destra = webcam_sinistra_destra + 15;
        webcam_control();
        webcam_X--;
        Serial.print(F("Posizione Cam ( "));
        Serial.print(webcam_X); //webcam_sinistra_destra
        Serial.print(F(", "));
        Serial.print(webcam_Y); //webcam_sopra_sotto
        Serial.println(F(" )"));
    }else Serial.println(F("Limite rotazione CAM raggiunto"));
  } else if (str == 98) //b
  {
    //float perc_1 = map(Battery_1_Volt*1000, 3200, 4200, 0, 100);
    //float perc_2 = map(Battery_2_Volt*1000, 3200, 4200, 0, 100);

    Serial.println(F("+===============|STATO ROVER|================+"));
    Serial.print(F("|TEMP = C"));
    Serial.println(analogReadTemp());  
    Serial.println(F("+======================================|ENERGIA ELETTRICA|======================================+"));
    if(busvoltage1 == 0 && busvoltage2 == 0 && busvoltage3 == 0)
      {
          Serial.print(F("|Errore, possibili problemi col sensore:     |"));
      }else {
            Serial.print(F("|Battery:     |")); 
            if (diagnostic == ATTIVATO)
            {
                Serial.print(F("V_Shunt: "));
                Serial.print(shuntvoltage1);
                Serial.print(F("V| "));
            }
            if(busvoltage1 == 0 && current_mA1 == 0)
            {
                Serial.println(F("Cavi scollegati |"));
            }else {
                Serial.print(F("V: "));
                Serial.print(busvoltage1);
                Serial.print(F("V| "));
                Serial.print(F(""));
                int bat_perc = map(busvoltage1 * 1000, 3200, 4200, 0, 100);
                int mah_bat = map(busvoltage1 * 1000, 3200, 4200, 0, 10000);
                residual_time_battery = mah_bat / mah_battery;
                Serial.print(bat_perc);
                Serial.print(F("% |"));
                Serial.print(mah_bat);
                Serial.print(F("mA residual|"));
                Serial.print(residual_time_battery);
                Serial.println(F("h |"));

                //Serial.print(F("A_Current: "));
                //Serial.print(current_mA1);
                //Serial.print(F("mA| "));
                //Serial.print(F("V_Load: "));
                //Serial.print(loadvoltage1);
                //Serial.print(F("V|"));
                //Serial.print(F("Max: "));
                //Serial.print(max_current1);
                //Serial.print(F("mA| "));
                //Serial.print(F("Min: "));
                //Serial.print(min_current1);
                //Serial.println(F("mA| "));
                }
            Serial.println(F("+===============================================================================================+"));
          
            Serial.print(F("|Solar panel: |"));
            if (diagnostic == ATTIVATO)
            {
                Serial.print(F("V_Shunt: "));
                Serial.print(shuntvoltage2);
                Serial.print(F("V| "));
            }
            if(busvoltage2 == 0 && current_mA2 == 0 && POSIZIONE_PANNELLO_DX == APERTO && POSIZIONE_PANNELLO_SX == APERTO)
            {
                Serial.println(F("Problemi con i pannelli |"));
            }else if(POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO)
            {
                Serial.println(F("Pannelli chiusi |"));
            }else {
                Serial.print(F("V_Bus: "));
                Serial.print(busvoltage2);
                Serial.print(F("V| "));
                Serial.print(F("A_Current: "));
                Serial.print(current_mA2);
                Serial.print(F("mA| "));
                Serial.print(F("V_Load: "));
                Serial.print(loadvoltage2);
                Serial.print(F("V|"));
                Serial.print(F("Max: "));
                Serial.print(max_current2);
                Serial.print(F("mA| "));
                Serial.print(F("Min: "));
                Serial.print(min_current2);
                Serial.print(F("mA| "));
                Serial.print(F("Efficiency: "));
                Serial.print(solar_efficiency);
                Serial.println(F("% |"));
            }
            Serial.println(F("+===============================================================================================+"));
          
            Serial.print(F("|Rover:       |"));
            if (diagnostic == ATTIVATO)
            {
                Serial.print(F("V_Shunt: "));
                Serial.print(shuntvoltage3);
                Serial.print(F("V| "));
            }
            if(busvoltage3 == 0 && current_mA3 == 0)
            {
                Serial.println(F("Non disponibile"));
            }else {
                Serial.print(F("V_Bus: "));
                Serial.print(busvoltage3);
                Serial.print(F("V| "));
                Serial.print(F("A_Current: "));
                Serial.print(current_mA3);
                Serial.print(F("mA| "));
                Serial.print(F("V_Load: "));
                Serial.print(loadvoltage3);
                Serial.print(F("V| "));
                Serial.print(F("Power: "));
                Serial.print(power_load3);
                Serial.print(F("W| "));
                Serial.print(F("Max: "));
                Serial.print(max_current3);
                Serial.print(F("mA| "));
                Serial.print(F("Min: "));
                Serial.print(min_current3);
                Serial.println(F("mA| "));
            }
      }
    Serial.println(F("+===============================================================================================+"));
    /*
    Serial.println(F("+=================|BATTERIA|=================+"));
    Serial.print(F("|BATTERIA ROVER = "));
    Serial.print(Battery_2_Volt);//(Vin_1 * f, 3);
    Serial.print(F("V"));
    Serial.print(F(" | "));
    Serial.print(perc_2);
    Serial.println(F("%"));
    Serial.println(F("+===========================================+"));
    Serial.print(F("|BATTERIA RASPBERRY = "));
    Serial.print(Battery_1_Volt);//(Vin_2 * f, 3);
    Serial.print(F("V"));
    Serial.print(F(" | "));
    Serial.print(perc_1);
    Serial.println(F("%"));
    */
    //Serial.println(F("+===========================================+"));
  }else if (str == 48) { //0
    pca_on_off();
  }else if (str == 56) { //8
    stampa_distanza_percorsa();
  } else if (str == 122) { //z
    if(motor_speed + 100 <= 4050)//  && motor_speed_id >= 1)
    {
        motor_speed = motor_speed + 405;//100;
        mot_speed_perc = map( motor_speed, 0, 4050, 0, 100);
        //motor_speed_id--;
        Serial.print("Velocita': ");
        Serial.println(motor_speed);
        //Serial.println(motor_speed_id);
        if(avanzamento_c == ATTIVATO){
           forward(FORWARD);
        }
    }
  }else if (str == 120) { //x
    if(motor_speed - 100 >= 200)// && motor_speed_id <= 10)
    {
        motor_speed = motor_speed - 405;// 100;
        mot_speed_perc = map( motor_speed, 0, 4050, 0, 100);
        //motor_speed_id++;
        Serial.print("Velocita': ");
        Serial.println(motor_speed);
        //Serial.println(motor_speed_id);
        if(avanzamento_c == ATTIVATO){
           forward(FORWARD);
        }
    }
  }else if (str == 79) { //O
        Serial.println("Azzeramento ODO: ");
        delay(1000);
        //save_odo_eeprom(0);
        save_odo_eeprom(0);
  }else if (str == 68) { //D
        security_distance =  security_distance + 10;
        Serial.println(F("Distanza di sicurezza ai lati: "));
        Serial.println(security_distance);
  }else if (str == 65) { //A
        security_distance =  security_distance - 10;
        Serial.println(F("Distanza di sicurezza ai lati: "));    //moltiplicatore_sterzata_automatica
        Serial.println(security_distance);
  }else if (str == 67) { //C
        stop_motors_wheel();
        /*
        if(moltiplicatore_sterzata_automatica < 5)
        {
              moltiplicatore_sterzata_automatica++;
              Serial.print(F("Moltiplicatore sterzata automatica: "));
              Serial.println(moltiplicatore_sterzata_automatica);
        }
        */
  }/*
  else if (str == 86) { //V
        if(moltiplicatore_sterzata_automatica > 1)
        {
              moltiplicatore_sterzata_automatica--;
              Serial.println(F("Moltiplicatore sterzata automatica: "));    //moltiplicatore_sterzata_automatica
              Serial.println(moltiplicatore_sterzata_automatica);
        }
  }*/else if (str == 99) { //c
    if(trimmer>=-10)
        {
        trimmer--;
        Serial.print(F("Trimmer: "));
        Serial.println(trimmer);
        sterzo(posizione_sterzo);
        //rp2040.idleOtherCore();
        EEPROM.write(10,trimmer);
        //rp2040.resumeOtherCore();
        if(diagnostic == ATTIVATO)
          Serial.print(F("Trimmer salvato! "));
        }
  }else if (str == 118) { //v
        if(trimmer<=10)
        {
        trimmer++;
        Serial.print("Trimmer: ");
        Serial.println(trimmer);
        sterzo(posizione_sterzo);
        //rp2040.idleOtherCore();
        EEPROM.write(10,trimmer);
        ////rp2040.resumeOtherCore();
        if(diagnostic == ATTIVATO)
          Serial.print("Trimmer salvato! ");
        }
   }else if (str == 76) { //L
        reset_lidar_sensor = 1;
        Serial.println(F("Sensori Lidar resettati"));
  }else if (str == 77) { //M
    if(COMMAND_MODE == PYTHON_SERVER)
        COMMAND_MODE = KEYBOARD;
    else if(COMMAND_MODE == KEYBOARD)
        COMMAND_MODE = PYTHON_SERVER;
  }else{
    Serial.println( F("Comando errato..."));
  }
}

void anticollision()
{
  if (anticollisione == ATTIVATO)
  {
    //rp2040.idleOtherCore();
    //=================ANT_Procedure===============================
    if (radar_ant <= 200 && radar_ant != 0)
    {
        if(avviso_ant == DISATTIVATO){
            Serial.println( F("RADAR ANT ALARM!"));
            Serial1.println( F("SYS_MSG,RADAR ANT ALARM!,ES"));

            avviso_ant = ATTIVATO;
       }if (avanzamento_c == ATTIVATO)
        {
          fine_conteggio_distanza = millis();
          forward(STOP);
          calcolo_distanza_percorsa();
          //avanzamento_c = DISATTIVATO;
        }
        //forward(STOP);
    }else avviso_ant = DISATTIVATO;
    //=================SX_45_Procedure===============================
    if (radar_sx_ant_45  + panels_security_distance <= 180  && radar_sx_ant_45 != 0/*radar_sx_alarm()*/)
    {
      if(avviso_sx_ant_45 == DISATTIVATO){
          Serial.println( F("RADAR SX 45 ALARM!"));
          Serial1.println( F("SYS_MSG,RADAR SX 45 ALARM!,ES"));
          avviso_sx_ant_45 = ATTIVATO;
      }
      if (sterzo_automatico == ATTIVATO)
      {
        posizione_sterzo = 440;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial.print( F("1) "));
        Serial.print( F("Sterzo automatico: "));
        Serial.println(posizione_sterzo/*posizione_sterzo*/);
        //}
        delay(200);
        posizione_sterzo = DRITTO;
        sterzo(posizione_sterzo);
      }
    }else avviso_sx_ant_45 = DISATTIVATO;
    //=================DX_45_Procedure===============================
    if (radar_dx_ant_45 <= 180  + panels_security_distance && radar_dx_ant_45 != 0/*radar_dx_alarm()*/)
    {
     if(avviso_dx_ant_45 == DISATTIVATO){
      Serial.println( F("RADAR DX 45 ALARM!"));
      Serial1.println( F("SYS_MSG,RADAR DX 45 ALARM!,ES"));
      
      avviso_dx_ant_45 = ATTIVATO;
     }
      if(sterzo_automatico == ATTIVATO)
      {
        posizione_sterzo = limite_sinistro;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial.print( F("2) "));
        Serial.print( F("Sterzo automatico: "));
        Serial.println(posizione_sterzo/*posizione_sterzo*/);
        //}
        delay(200);
        posizione_sterzo = DRITTO;
        sterzo(posizione_sterzo);
      }
    }else avviso_dx_ant_45 = DISATTIVATO;
    //=================DX_Procedure===============================
  if (radar_dx_center <= security_distance + panels_security_distance && radar_sx_center >= radar_dx_center && radar_dx_center != 0/*security_distance + panels_security_distance radar_dx_alarm()*/)
    {
      if(avviso_dx_center == DISATTIVATO && sterzo_automatico == DISATTIVATO){
        Serial.println( F("RADAR DX ALARM!"));
        Serial1.println( F("SYS_MSG,RADAR DX ALARM!,ES"));
          avviso_dx_center = ATTIVATO;
      }
      if(sterzo_automatico == ATTIVATO)
      {
        if(posizione_sterzo != DRITTO - valore_sterzata * moltiplicatore_sterzata_automatica)
        {
            posizione_sterzo = DRITTO - valore_sterzata * moltiplicatore_sterzata_automatica;
            sterzo(posizione_sterzo);
            if(diagnostic == ATTIVATO)
              Serial.print( F("3) "));
            Serial.print( F("Sterzo automatico: "));
            Serial.println(posizione_sterzo/*posizione_sterzo*/);
        }
      }
    }else if(radar_dx_center >= security_distance  + panels_security_distance && radar_sx_center > radar_dx_center/*radar_dx_alarm()*/)
    {
      avviso_dx_center = DISATTIVATO;
      if (sterzo_automatico == ATTIVATO)
      {
        if(posizione_sterzo != DRITTO + valore_sterzata * moltiplicatore_sterzata_automatica)
        {
        posizione_sterzo = DRITTO + valore_sterzata * moltiplicatore_sterzata_automatica;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial.print( F("4) "));
        Serial.print( F("Sterzo automatico: "));
        Serial.println(posizione_sterzo/*posizione_sterzo*/);
        }
      }
    }
      //else //forward(STOP);
    //=================SX_Procedure===============================
    if (radar_sx_center <= security_distance + panels_security_distance && radar_dx_center >= radar_sx_center && radar_sx_center != 0/*security_distance + panels_security_distance *//*radar_dx_alarm()*/)
    {
      if(avviso_sx_center == DISATTIVATO && sterzo_automatico == DISATTIVATO){
        Serial.println( F("RADAR SX ALARM!"));
        Serial1.println( F("SYS_MSG,RADAR SX ALARM!,ES"));
          avviso_sx_center = ATTIVATO;
      }
      if (sterzo_automatico == ATTIVATO)
      {
        if(posizione_sterzo != DRITTO + valore_sterzata * moltiplicatore_sterzata_automatica)
        {
        posizione_sterzo = DRITTO + valore_sterzata * moltiplicatore_sterzata_automatica;
        sterzo(posizione_sterzo);
        if(diagnostic == ATTIVATO)
          Serial.print( F("5) "));
        Serial.print( F("Sterzo automatico: "));
        Serial.println(posizione_sterzo/*posizione_sterzo*/);
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
                    Serial.println( F("RIAVVICINAMENTO A SINISTRA!"));
                    avviso_sx_center = ATTIVATO;
                }
                if (sterzo_automatico == ATTIVATO)
                {
                  if(posizione_sterzo != DRITTO - valore_sterzata * moltiplicatore_sterzata_automatica)
                  {
                    posizione_sterzo = DRITTO - valore_sterzata * moltiplicatore_sterzata_automatica;
                    sterzo(posizione_sterzo);
                    if(diagnostic == ATTIVATO)
                      Serial.print( F("6) "));
                    Serial.print( F("Sterzo automatico: "));
                    Serial.println(posizione_sterzo/*posizione_sterzo*/);
                  }
                }
        if(radar_sx_center > 1500 && radar_dx_center > 1500 && posizione_sterzo != 330)
        {
          posizione_sterzo = 330;
          sterzo(posizione_sterzo);
          if(diagnostic == ATTIVATO)
              Serial.print( F("7) "));
          Serial.print( F("Sterzo automatico: "));
          Serial.println(posizione_sterzo/*posizione_sterzo*/);
          Serial.println( F("Nessun ostacolo di riferimento trovato."));
          sterzo_automatico = DISATTIVATO;
          Serial.println( F("Sterzo automatico disabilitato automaticamente."));
        }   
      }
      }
      }// DA CONTROLLARE
    //=================SX_DX_Procedure==================================
    else if(radar_sx_center <= security_distance + panels_security_distance && radar_dx_center <= security_distance + panels_security_distance/*radar_dx_alarm()*/)
        if(radar_sx_center < radar_dx_center)
          {
            if (sterzo_automatico == ATTIVATO)
                {
                  if(posizione_sterzo != DRITTO + valore_sterzata)
                  {
                      posizione_sterzo = DRITTO + valore_sterzata;
                      sterzo(posizione_sterzo);
                      Serial.print( F("9) Sterzo automatico: "));
                      Serial.println(posizione_sterzo/*posizione_sterzo*/);
                  }
                }
        }else if (radar_sx_center > radar_dx_center)
                {
                  if (sterzo_automatico == ATTIVATO)
                  {
                    if(posizione_sterzo != DRITTO - valore_sterzata)
                    {
                      posizione_sterzo = DRITTO - valore_sterzata;
                      sterzo(posizione_sterzo);
                      Serial.print( F("10) Sterzo automatico: "));
                      Serial.println(posizione_sterzo/*posizione_sterzo*/);
                    }
                  }
          }
}
        
void raspberry_serial_communication_diagnostic()
{
  while (Serial1.available() > 0 ) {
    char buff[128];
    size_t str_lenght = Serial1.readBytes(buff, str_lenght);
    Serial.println(buff[1]);
    //rover_commands(str);
  }
}

void SerialCommands() {
  if (Serial1.available()) {

    char line[128];
    size_t lineLength = Serial1.readBytesUntil('\n', line, 127);
    for(int i = 0; i < lineLength; i++)
    {
      Serial.print(line[i]);
    }
    Serial.println();
    line[lineLength] = '\0';
    
    char response[MyCommandParser::MAX_RESPONSE_SIZE];
    parser.processCommand(line, response);
    Serial.println(response);
  }
}

void raspberry_serial_communication_keyboard()
{
  while (Serial1.available() > 0 ) {
    uint8_t str = Serial1.read();
    //Serial.println(str);
    rover_commands(str);
  }
}

void loop() {
  //raspberry_serial_communication_keyboard();
  //raspberry_serial_communication_diagnostic();
  //cmd_Parser();
  SerialCommands();
  //cmdCallback.updateCmdProcessing(&myParser, &myBuffer, &Serial1);
  //message_from_python_server();
  anticollision();
}
