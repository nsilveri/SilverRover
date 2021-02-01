//#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "FaBoPWM_PCA9685.h"
#include <HCSR04.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
HCSR04 hc(5,new int[4]{3,2,4,6},4);
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
FaBoPWM faboPWM;  
const uint16_t STOP = 0;
uint16_t rover_speed = 400;
const uint16_t FORWARD = rover_speed;
const uint16_t BACK = 250;
const uint16_t run_delay = 2100; //2100 percorre circa 50cm
uint8_t POSIZIONE_PANNELLO_DX = CHIUSO;
uint8_t POSIZIONE_PANNELLO_SX = CHIUSO;
uint8_t peso_destra = 0;
uint8_t peso_sinistra = 0;
uint8_t stato_precedente = CHIUSO;
const uint8_t panel_speed = 5;
int radar_scan[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int radar_scan_motor[13] = {200, 230, 260, 290, 350, 370, 390, 420, 450, 480, 510, 540, 570};
uint8_t webcam_sterzo = ATTIVATO;
uint8_t radar_sterzo = DISATTIVATO;
//uint8_t pca_off = ATTIVATO;
uint16_t posizione_sterzo = STERZO_DRITTO;
uint16_t webcam_sopra_sotto = 250;
uint16_t webcam_sinistra_destra = 268;
uint16_t webcam_sopra_sotto_effettuato = DISATTIVATO;
uint8_t risparmio_energetico = DISATTIVATO;
uint8_t sterzo_automatico = DISATTIVATO;

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
//uint16_t
//uint16_t
int8_t avanzamento_c = DISATTIVATO;
int8_t anticollisione = ATTIVATO;

int radar_dx_ant = 0;
int radar_dx_center = 0;
int radar_sx_ant = 0;
int radar_sx_center = 0;
int radar_ant = 0;
int radar_post = 0;

//SoftwareSerial newPort(10, 11);


void setup() {
  Serial.begin(9600);
  //Serial.begin(9600);
  Serial.println( F("Welcome to the SilverRover"));
  Serial.println( F("Insert comand to send to the Rover."));
  Serial.println( F("scrivi 'help' per vedere i comandi disponibili"));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pinMode(7,OUTPUT);
//  pca_on_off();//attiva pca
  sterzo_dritto();
  ruote_sterzanti_post = DISATTIVATO;
  

  
  delay(200);
  
//pca_on_off();//disattiva pca
}

//void pca_on_off()
//{
//  if(pca_off == DISATTIVATO)2
//  {
//    pca_off = ATTIVATO;
//    digitalWrite(7,HIGH); 
//    Serial.println(F("Motori disattivati"));   
//  }else if(pca_off == ATTIVATO)
//  {
//    pca_off = DISATTIVATO;
//    digitalWrite(7,LOW);
//    Serial.println(F("Motori attivati"));    
//  }
//}
//void prec_sterzo()
//{
//  uint16_t ps = 0;
//  Serial.print( F("La precisione attuale dello sterzo è: "));
//  Serial.println(precisione_sterzo);
//  Serial.print( F("Inserisci il valore di precisione desiderato ( 2, 4, 6,...): "));
//  precisione_sterzo = Serial.read();
//  if(110 % precisione_sterzo != 0)
//    prec_sterzo();
//  //precisione_sterzo = ps;
//  valore_sterzata = 110/precisione_sterzo;
//}

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
  pwm.setPWM(n, 0, pulse);
}

void forward(uint16_t comand)
{
  if(comand == STOP)
  {
    faboPWM.set_channel_value(4, STOP); 
    faboPWM.set_channel_value(5, STOP);
    faboPWM.set_channel_value(6, STOP); 
    faboPWM.set_channel_value(7, STOP);
  }else if(comand == FORWARD)
  {
    faboPWM.set_channel_value(4, FORWARD); 
    faboPWM.set_channel_value(5, BACK);
    faboPWM.set_channel_value(6, BACK); 
    faboPWM.set_channel_value(7, FORWARD);
  }else if(comand == BACK)
  {
    faboPWM.set_channel_value(4, BACK); 
    faboPWM.set_channel_value(5, FORWARD);
    faboPWM.set_channel_value(6, FORWARD); 
    faboPWM.set_channel_value(7, BACK);
  }
}

bool zona_radar(uint16_t value)
{
  if(value > 230 && value < 360)
     return true;
  return false;
}

void webcam_control()
{
  uint16_t webcam_sinistra_destra_l = webcam_sinistra_destra;
  uint16_t webcam_sopra_sotto_l = webcam_sopra_sotto;
  if(zona_radar(webcam_sinistra_destra) && webcam_sopra_sotto >435)
        webcam_sopra_sotto_l = 435;
  pwm.setPWM(15, 0, webcam_sinistra_destra_l);
  pwm.setPWM(14, 0, webcam_sopra_sotto_l);
}

void _webcam_sterzo(uint16_t pulselen)
{
  webcam_sinistra_destra = pulselen-60;
  webcam_sopra_sotto = 315;
  pwm.setPWM(15, 0, webcam_sinistra_destra);
  pwm.setPWM(14, 0, webcam_sopra_sotto);
}

void sterzo_anteriore(uint16_t pulselen)
{
  pulselen = pulselen +11;
  int p_p= map(pulselen, 220, 440, 264 ,397);
  if(pulselen > 330)
  {
    pwm.setPWM(0, 0, pulselen-8);//-5
    pwm.setPWM(1, 0, p_p+68);
  }else if(pulselen < 330)
  {
    pwm.setPWM(0, 0, p_p-8);//-5
    pwm.setPWM(1, 0, pulselen+68);
  }else {
    pwm.setPWM(0, 0, pulselen-8);//-5
    pwm.setPWM(1, 0, pulselen+68);
  }
}


void sterzo_posteriore(uint16_t pulselen)
{
//  if(webcam_sterzo == ATTIVATO)
//    _webcam_sterzo(pulselen);
//  if(radar_sterzo ==ATTIVATO)
//    radar_motor(pulselen);
//    delay(200);
  pwm.setPWM(2, 0, pulselen+38);//40
  pwm.setPWM(3, 0, pulselen-5);
  
}
  
  void sterzo(uint16_t angolo_serzata)
  {
  uint16_t pulselen_ant = angolo_serzata;
  uint16_t pulselen_post = pulselen_ant + 25;
  int calc_angle = STERZO_DRITTO-pulselen_ant;
  uint16_t angle_post= STERZO_DRITTO + calc_angle + 25;
  if(ruote_sterzanti_ant == ATTIVATO);
     sterzo_anteriore(pulselen_ant);
  if(ruote_sterzanti_post == ATTIVATO)
     sterzo_posteriore(angle_post);
  if(webcam_sterzo == ATTIVATO)
    _webcam_sterzo(angle_post);
  if(radar_sterzo ==ATTIVATO)
    radar_motor(angle_post);
    //delay(200);
  }

  
  void sterzo_dritto()
  {
    if(posizione_sterzo != DRITTO)
    {
      if(posizione_sterzo < DRITTO){
         for(int i=posizione_sterzo; i <= DRITTO; i++)
           {
            posizione_sterzo = i;
            sterzo(posizione_sterzo);
            delay(5);
            
           }
      }else if(posizione_sterzo > DRITTO)
           {
             for(int i=posizione_sterzo; i >= DRITTO; i--)
               {
                posizione_sterzo = i;
                sterzo(posizione_sterzo);
                delay(5);
               }
           }
            
           
    }
    posizione_sterzo = DRITTO;
    sterzo(STERZO_DRITTO);
  }

  
  void panel_dx_da_meta_a_chiuso()
  {
    if(POSIZIONE_PANNELLO_DX == META)
      {
        for(int i=0;i<220;i++)
          {
              pwm.setPWM(8, 0, 328+i/*i*/);
              pwm.setPWM(9, 0, 328-i/*i*/);
              delay(panel_speed);
          }
          POSIZIONE_PANNELLO_DX = 0;
      }
  }
  
  void panel_dx_da_chiuso_a_meta()
  {
    if(POSIZIONE_PANNELLO_DX == 0)
      {
          for(int i=220;i>0;i--){
                  pwm.setPWM(8, 0, 328+i/*i*/);
                  pwm.setPWM(9, 0, 328+7-i/*i*/);
                  delay(panel_speed);
      }
    POSIZIONE_PANNELLO_DX = META;
      }
  }

  void panel_dx_da_meta_a_aperto()
  {
      if(POSIZIONE_PANNELLO_DX == META)
      {
            for(int i=0;i<250;i++)
            {
                pwm.setPWM(8, 0, 328-i/*i*/);
                pwm.setPWM(9, 0, 328+7+i/*i*/);
                delay(panel_speed);
            }
         POSIZIONE_PANNELLO_DX = APERTO ;   
      }
  }

  void panel_dx_da_aperto_a_meta()
  {
    if(POSIZIONE_PANNELLO_DX == APERTO )
    {
      for(int i=250;i>0;i--)
      {
              pwm.setPWM(8, 0, 328-i/*i*/);
              pwm.setPWM(9, 0, 328+7+i/*i*/);
              delay(panel_speed);
      }
      POSIZIONE_PANNELLO_DX = META;
    }
  }

  void panel_sx_da_meta_a_chiuso()
  {
    if(POSIZIONE_PANNELLO_SX == META)
            {
              for(int i=0;i<220;i++)
              {
                  pwm.setPWM(10, 0, 328+i/*i*/);
                  pwm.setPWM(11, 0, 328-i/*i*/);
                  delay(panel_speed);
              }
              POSIZIONE_PANNELLO_SX = 0;
            }
  }

  void panel_sx_da_chiuso_a_meta()
  {
        if(POSIZIONE_PANNELLO_SX == 0)
            {
              for(int i=220;i>0;i--)
              {
                  pwm.setPWM(10, 0, 328+i/*i*/);
                  pwm.setPWM(11, 0, 328-i/*i*/);
                  delay(panel_speed);
              }
               POSIZIONE_PANNELLO_SX = META;
            }
  }

  void panel_sx_da_meta_a_aperto()
  {
  if(POSIZIONE_PANNELLO_SX == META)
          {
            for(int i=0;i<240;i++)
            {
                pwm.setPWM(10, 0, 328-i/*i*/);
                pwm.setPWM(11, 0, 328+i/*i*/);
                delay(panel_speed);
            }
          }
       POSIZIONE_PANNELLO_SX = APERTO ;   
  }

  void panel_sx_da_aperto_a_meta()
  {
    if(POSIZIONE_PANNELLO_SX == APERTO )
          {
              for(int i=240;i>0;i--)
                  {
                      pwm.setPWM(10, 0, 328-i/*i*/);
                      pwm.setPWM(11, 0, 328+i/*i*/);
                      delay(panel_speed);
                  }
              POSIZIONE_PANNELLO_SX = META;
          }
  }

  void panel_sx_dx_da_meta_a_aperto()
  {
    if(POSIZIONE_PANNELLO_SX == META && POSIZIONE_PANNELLO_DX == META)
          {
            int i,j = 0;
            for(i=0;i<240;i++){
                  pwm.setPWM(10, 0, 328-i);
                  pwm.setPWM(11, 0, 328+i);
                  pwm.setPWM(8, 0, 328+10-i);
                  pwm.setPWM(9, 0, 328+i);
                  delay(panel_speed);
          }
            j=i-20;
            pwm.setPWM(10, 0, 328-j);
            pwm.setPWM(11, 0, 328+j);
            POSIZIONE_PANNELLO_SX = APERTO ;
            POSIZIONE_PANNELLO_DX = APERTO ;
          }
  }

  void panel_sx_dx_da_apero_a_meta()
  {
    if(POSIZIONE_PANNELLO_SX == APERTO && POSIZIONE_PANNELLO_DX == APERTO )
          {
              int i,j = 0;
              for(i=250;i>0;i--)
                  {
                      pwm.setPWM(8, 0, 328+10-i/*i*/);
                      pwm.setPWM(9, 0, 328+i/*i*/);
                      //for(int i=240;i>0;i--){
                      pwm.setPWM(10, 0, 328-i/*i*/);
                      pwm.setPWM(11, 0, 328+i/*i*/);
                      delay(panel_speed);
                  }
              POSIZIONE_PANNELLO_SX = META;
              POSIZIONE_PANNELLO_DX = META;
//    j=i-20;
//    pwm.setPWM(10, 0, 328-j);
//    pwm.setPWM(11, 0, 328+j);
            }
  }

  void open_panels()
  {
    if(POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO && peso_sinistra == DISATTIVATO && peso_destra == DISATTIVATO)
        {
          panel_sx_da_chiuso_a_meta();
          Serial.println( F("Pannello sinistro aperto a metà"));
          panel_dx_da_chiuso_a_meta();
          Serial.println( F("Pannello destro aperto a metà"));
          panel_sx_dx_da_meta_a_aperto();
//          POSIZIONE_PANNELLO_SX = 0;
          Serial.println( F("Entrambi i pannelli aperti"));
          POSIZIONE_PANNELLO_DX = APERTO ;
          POSIZIONE_PANNELLO_SX = APERTO ;
          stato_precedente = APERTO ;
        }else if(peso_sinistra == ATTIVATO)
                  {
                    stato_precedente = 1;
                    peso_a_sinistra();
        }else if(peso_destra == ATTIVATO)
                  {
                    stato_precedente = 1;
                    peso_a_destra();
                  }
        Serial.println( F("azioni Pannello sx effettuate"));
  }

  void close_panels()
  {
    if(POSIZIONE_PANNELLO_DX == APERTO && POSIZIONE_PANNELLO_SX == APERTO && peso_sinistra == DISATTIVATO && peso_destra == DISATTIVATO)
        {
          panel_sx_dx_da_apero_a_meta();
          Serial.println( F("Pannello sinistro aperto a metà"));
          Serial.println( F("Pannello destro aperto a metà"));
          panel_dx_da_meta_a_chiuso();
          panel_sx_da_meta_a_chiuso();
//          POSIZIONE_PANNELLO_SX = 0;
          Serial.println( F("Entrambi i pannelli chiusi"));
          Serial.println( F("azioni Pannello sx effettuate"));
          POSIZIONE_PANNELLO_DX = 0;
          POSIZIONE_PANNELLO_SX = 0;
          stato_precedente = 0;
        }else if(peso_sinistra == ATTIVATO)
                  {
                    stato_precedente = 0;
                    peso_a_sinistra();
        }else if(peso_destra == ATTIVATO)
                  {
                    stato_precedente = 0;
                    peso_a_destra();
                  }
  }

  void left_panel()
  {
    if(POSIZIONE_PANNELLO_SX == 0)
        {
          panel_sx_da_chiuso_a_meta();
          Serial.println( F("Pannello sinistro aperto a metà"));
          panel_sx_da_meta_a_aperto();
          //POSIZIONE_PANNELLO_SX = APERTO ;
          Serial.println( F("Pannello sinistro aperto completamente"));
        }else if(POSIZIONE_PANNELLO_SX == APERTO )
        {
          panel_sx_da_aperto_a_meta();
          Serial.println( F("Pannello sinistro aperto a metà"));
          //panel_dx_da_chiuso_a_meta();
          
          panel_sx_da_meta_a_chiuso();
          //POSIZIONE_PANNELLO_SX = 0;
          if(POSIZIONE_PANNELLO_SX == 0)
              Serial.println( F("Pannello sinistro 0"));
        }else if(POSIZIONE_PANNELLO_SX ==2)
        {
          panel_sx_da_meta_a_chiuso();
          //POSIZIONE_PANNELLO_SX = 0;
          if(POSIZIONE_PANNELLO_SX == 0)
              Serial.println( F("Pannello sinistro 0"));
        }
        Serial.println( F("azioni Pannello sx effettuate"));
  }

  void right_panel()
  {
    if(POSIZIONE_PANNELLO_DX ==CHIUSO&& POSIZIONE_PANNELLO_SX != 0)
        {
          panel_dx_da_chiuso_a_meta();
          Serial.println( F("Pannello destro aperto a metà"));
          panel_dx_da_meta_a_aperto();
          //POSIZIONE_PANNELLO_DX = APERTO ;
          Serial.println( F("Pannello destro completamente aperto"));
        }else if(POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX != 0)
        {
          panel_dx_da_aperto_a_meta();
          Serial.println( F("Pannello destro aperto a metà"));
          panel_dx_da_meta_a_chiuso();
          if(POSIZIONE_PANNELLO_DX == 0)
              Serial.println( F("Pannello destro 0"));
        }else if(POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX == 0)
        {
          panel_sx_da_chiuso_a_meta();
          panel_dx_da_aperto_a_meta();
          Serial.println( F("Pannello destro aperto a metà"));
          panel_dx_da_meta_a_chiuso();
          panel_sx_da_meta_a_chiuso();
          if(POSIZIONE_PANNELLO_DX == 0)
              Serial.println( F("Pannello destro 0"));
        }else if(POSIZIONE_PANNELLO_DX ==2 && POSIZIONE_PANNELLO_SX != 0)
        {
          //panel_dx_da_aperto_a_meta();
          Serial.println( F("Pannello destro aperto a metà"));
          panel_dx_da_meta_a_chiuso();
          if(POSIZIONE_PANNELLO_DX == 0)
              Serial.println( F("Pannello destro 0"));
        }
        Serial.println( F("azioni Pannello dx effettuate"));
  }


  void radar_motor(uint16_t angle_motor)
  {
    pwm.setPWM(12, 0, angle_motor+15);
  }

  /**/void peso_a_sinistra()
  {
    Serial.println( F("1"));
    if(peso_sinistra == 0)
          {
            //Serial.println( F("2"));
            if(POSIZIONE_PANNELLO_DX ==CHIUSO&& POSIZIONE_PANNELLO_SX == CHIUSO)
              {
                panel_sx_da_chiuso_a_meta();
                panel_sx_da_meta_a_aperto();
                peso_sinistra = ATTIVATO;
                //Serial.println( F("3"));
              }else if(POSIZIONE_PANNELLO_DX == APERTO  && POSIZIONE_PANNELLO_SX == APERTO)
              {
                //Serial.println( F("4"));
                panel_dx_da_aperto_a_meta();
                panel_dx_da_meta_a_chiuso();
                peso_sinistra = ATTIVATO;
              }
              
           }else if(peso_sinistra == ATTIVATO)
          {
            //Serial.println( F("2"));
            if(stato_precedente == 0)
              {
                panel_sx_da_aperto_a_meta();
                panel_sx_da_meta_a_chiuso();
                peso_sinistra = DISATTIVATO;
                //Serial.println( F("3"));
              }else if(stato_precedente == APERTO )
              {
                //Serial.println( F("4"));
                panel_dx_da_chiuso_a_meta();
                panel_dx_da_meta_a_aperto();
                peso_sinistra = DISATTIVATO;
              }
           }
           //Serial.println( F("5"));
  }//*/
  

  /**/void peso_a_destra() // muove i pannelli solari in modo da ottenere maggior peso verso destra per evitare eventuali ribaltamenti dovuti alla pendenza
  {
     if(peso_destra == DISATTIVATO) //controllo per verificare che la funzione non sia già attiva
          {
           if(POSIZIONE_PANNELLO_DX == CHIUSO && POSIZIONE_PANNELLO_SX == CHIUSO)
              {
                panel_sx_da_chiuso_a_meta();
                panel_dx_da_chiuso_a_meta();
                panel_dx_da_meta_a_aperto();
                panel_sx_da_meta_a_chiuso();
                peso_destra = ATTIVATO;
              }else if(POSIZIONE_PANNELLO_DX == APERTO && POSIZIONE_PANNELLO_SX == APERTO)
                  {
                    panel_sx_da_aperto_a_meta();
                    panel_sx_da_meta_a_chiuso();
                    peso_destra = ATTIVATO;
                  }
                                 
           }else if(peso_destra == ATTIVATO)
          {
            Serial.println( F("peso destro attivo 2"));
            if(stato_precedente == CHIUSO)
              {
                //Serial.println(stato_precedente);
                panel_sx_da_chiuso_a_meta();
                panel_dx_da_aperto_a_meta();
                panel_dx_da_meta_a_chiuso();
                panel_sx_da_meta_a_chiuso();
                Serial.println( F("3"));
                peso_destra = DISATTIVATO;
              }else if(stato_precedente == APERTO)
              {
                Serial.println(stato_precedente);
                panel_sx_da_chiuso_a_meta();
                panel_sx_da_meta_a_aperto();
                peso_destra = DISATTIVATO;
              }
              //peso_sinistra = DISATTIVATO;
           } 
  }

  void avanzamento_continuo()
  {
    avanzamento_c = ATTIVATO;
    forward(FORWARD);
  }
  
  void avanti(int run_delay_1)
  {
        int distance_ant ;
        int distance_sx ;
        int distance_dx ;
        distance_ant = hc.dist(0);
        delay(200);
        distance_sx = hc.dist(3);
        delay(200);
        distance_dx = hc.dist(2);
        delay(200);
        if(posizione_sterzo == DRITTO)
        {
              radar_motor(DRITTO);
              Serial.print( F("Distanza: "));
              Serial.println(distance_ant);
              if(/*distance_ant>520 && */run_delay_1 == 21000)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 5mt"));
                  delay(run_delay_1);
                  forward(STOP);
              }else if(/*distance_ant>55 && */run_delay_1 == 0)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 50cm "));
                  delay(2100);
                  forward(STOP);
              }else if(/*distance_ant>12 && */run_delay_1 == 525)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 10cm "));
                  delay(run_delay_1);
                  forward(STOP);
              }
              else if(/*distance_ant<55 && distance_ant>25 && */run_delay_1 == 0)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 25cm"));
                  delay(1050);
                  forward(STOP);
             }
        }else if(posizione_sterzo > DRITTO)
        {
              Serial.print( F("Distanza anteriore: "));
              Serial.println(distance_ant);
              Serial.print( F("Distanza a destra: "));
              Serial.println(distance_dx);
               if(distance_ant>=20 && distance_dx>=10)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti"));
                  delay(1050);// indietreggia di 25 cm verso destra
                  forward(STOP);
              }else if(distance_ant>12 && run_delay_1 == 525 && distance_dx>=8)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 10cm "));
                  delay(run_delay_1);
                  forward(STOP);
              }else if(distance_ant<10 && distance_dx<10)
              {
                  Serial.println( F("Ostacolo a destra troppo vicino"));                      
              }
        }else if(posizione_sterzo < DRITTO)
        {
              Serial.print( F("Distanza anteriore: "));
              Serial.println(distance_ant);
              Serial.print( F("Distanza a sinistra: "));
              Serial.println(distance_sx);
               if(distance_ant>=20 && distance_sx>=10)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti"));
                  delay(1050);// indietreggia di 25 cm verso sinistra
                  forward(STOP);
              }else if(distance_ant>12 && run_delay_1 == 525 && distance_sx>=8)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 10cm "));
                  delay(run_delay_1);
                  forward(STOP);
              }else if(distance_ant<10 && distance_sx<10)
              {
                  Serial.println( F("Ostacolo a sinistra troppo vicino"));                      
              }
  }
  }

  void indietro()
  {
        if(avanzamento_c == ATTIVATO)
        {
          forward(STOP);
          avanzamento_c = DISATTIVATO;
        }else if(avanzamento_c == DISATTIVATO)
        {
        int distance_post = 0;
        int distance_sx = 0;
        int distance_dx = 0;
        distance_post = hc.dist(1);
        delay(200);
        distance_sx = hc.dist(3);
        delay(200);
        distance_dx = hc.dist(2);
        delay(200);
        
        if(posizione_sterzo == DRITTO)
        {
              Serial.print( F("Distanza posteriore: "));
              Serial.println(distance_post);
              if(distance_post>=12 && distance_dx>=15 && distance_sx>=15)
              {
                  forward(BACK);
                  Serial.println( F("Indietro"));
                  delay(525);
                  forward(STOP);
              }
              else if(distance_post<55 && distance_post>25)
              {
                  forward(BACK);
                  Serial.println( F("Indietro per 25cm"));
                  delay(1050);
                  forward(STOP);
              }else if(distance_post<25)
              {
                Serial.print(F(" Sei troppo vicino ad un ostacolo, azione non consentita "));
              }
        }else if(posizione_sterzo > DRITTO)
        {
              Serial.print( F("Distanza posteriore: "));
              Serial.println(distance_post);
              Serial.print( F("Distanza a destra: "));
              Serial.println(distance_dx);
               if(distance_post>=15 && distance_dx>=25)
              {
                  forward(BACK);
                  Serial.println( F("Indietro"));
                  delay(525);// indietreggia di 12,5 cm verso destra
                  forward(STOP);
              }else if(distance_post<10 && distance_dx<10)
              {
                  Serial.println( F("Ostacolo a destra troppo vicino"));                      
              }
        }else if(posizione_sterzo < DRITTO)
        {
              Serial.print( F("Distanza posteriore: "));
              Serial.println(distance_post);
              Serial.print( F("Distanza a sinistra: "));
              Serial.println(distance_sx);
               if(distance_post>=20 && distance_sx>=10)
              {
                  forward(BACK);
                  Serial.println( F("Indietro"));
                  delay(1050);// indietreggia di 25 cm verso sinistra
                  forward(STOP);
              }else if(distance_post<10 && distance_sx<10)
              {
                  Serial.println( F("Ostacolo a destra troppo vicino"));                      
              }
        }
        }
  }


bool radar_alarm()
  {
    radar_ant = hc.dist(0);
            delay(radar_delay);
    radar_motor(DRITTO);
    radar_dx_center = hc.dist(2);
            delay(radar_delay);
    radar_sx_center = hc.dist(3);
            delay(radar_delay);
    radar_post = hc.dist(1);
            delay(radar_delay);
    if(radar_ant<20)
    {
       return true;//AVANTI
    }
    if(radar_dx_center<20)
    {
       return true;//AVANTI
    }//DESTRO
    if(radar_sx_center<20)
    {
       return true;//AVANTI
    }//SINISTRA
    if(radar_post<20)
    {
       return true;//AVANTI
    }//POSTERIORE
    return false;
  }

  bool radar_ant_alarm()
  {
    radar_ant = hc.dist(0);
            delay(radar_delay);
    radar_motor(DRITTO);
    if(radar_ant<20)
    {
       return true;//AVANTI
    }
    return false;
  }

  bool radar_sx_alarm()
  {
      uint16_t radar_sx_center_1 = hc.dist(3);
              delay(radar_delay);
      uint16_t radar_sx_center_2 = hc.dist(3);
              delay(radar_delay);
    if(radar_sx_center_1<20 && radar_sx_center_2 < radar_sx_center_1)
    {
       return true;//AVANTI
    }//SINISTRA
    return false;
  }
  
  bool radar_dx_alarm()
  {
      uint16_t radar_dx_center_1 = hc.dist(2);
              delay(radar_delay);
      uint16_t radar_dx_center_2 = hc.dist(2);
              delay(radar_delay);
    if(radar_dx_center_1<20 && radar_dx_center_2 < radar_dx_center_1)
    {
       return true;//AVANTI
    }//DESTRO
    return false;
  }
  
  bool radar_post_alarm()
  {
    radar_post = hc.dist(1);
            delay(radar_delay);
    if(radar_post<20)
    {
       return true;//AVANTI
    }//POSTERIORE
    return false;
  }

  int lunghezza_int(int input)
  {
    int count = 0;
    while (input > 0){
         input/=10;
         count++;
            }
     return count;
  }

  void radar_update()
  {
            radar_ant = hc.dist(0);
            delay(radar_delay);
            radar_motor(radar_dx_ant_position);
            delay(200);
            radar_dx_ant = hc.dist(0)-6; 
            delay(radar_delay);
            radar_motor(radar_sx_ant_position);
            delay(400);
            radar_sx_ant = hc.dist(0)-6;
            delay(radar_delay);
            radar_motor(DRITTO);
            radar_dx_center = hc.dist(2);
            delay(radar_delay);
            radar_sx_center = hc.dist(3);
            delay(radar_delay);
            radar_post = hc.dist(1);
            delay(radar_delay);
  }

  void radar_s()
  {
            radar_ant = hc.dist(0);
            delay(radar_delay);
            radar_motor(radar_dx_ant_position);
            delay(200);
            radar_dx_ant = hc.dist(0)-6; 
            delay(radar_delay);
            radar_motor(radar_sx_ant_position);
            delay(400);
            radar_sx_ant = hc.dist(0)-6;
            delay(radar_delay);
            radar_motor(DRITTO);
            radar_dx_center = hc.dist(2);
            delay(radar_delay);
            radar_sx_center = hc.dist(3);
            delay(radar_delay);
            radar_post = hc.dist(1);
            delay(radar_delay);
            int spazi = 0;        
            Serial.println(F("+=============|RADAR|=====================+"));
            Serial.print(F("|AS:"));
            Serial.print(radar_sx_ant);
            Serial.print(F("cm"));
            spazi = 10 - lunghezza_int(radar_sx_ant);
            for(int i=0;i<spazi;i++)
              {
              Serial.print(F(" "));
              }
            Serial.print(F("A: "));
            Serial.print(radar_ant);
            Serial.print(F("cm"));
            spazi = 10 - lunghezza_int(radar_ant);
            for(int i=0;i<spazi;i++)
              {
              Serial.print(F(" "));
              }
            Serial.print(F("AD:"));
            Serial.print(radar_dx_ant);
            Serial.print(F("cm"));
            spazi = 6 - lunghezza_int(radar_dx_ant);
            for(int i=0;i<spazi;i++)
              {
              Serial.print(F(" "));
              }
            Serial.println(F("|"));
            Serial.println(F("+=========================================+"));
            delay(radar_delay);
            Serial.print(F("|S: "));
            Serial.print(radar_sx_center);
            Serial.print(F("cm"));
            spazi = 10 - lunghezza_int(radar_sx_center);
            for(int i=0;i<spazi;i++)
              {
              Serial.print(F(" "));
              }
            delay(radar_delay);
            Serial.print(F("               "));
            Serial.print(F("D: "));
            Serial.print(radar_dx_center);
            Serial.print(F("cm"));
            spazi = 6 - lunghezza_int(radar_dx_center);
            for(int i=0;i<spazi;i++)
              {
              Serial.print(F(" "));
              }
            Serial.println(F("|"));
            Serial.println(F("+=========================================+"));
            //Serial.println(F("|"));
            delay(radar_delay);
            Serial.print(F("|"));
            Serial.print(F("               D: "));
            Serial.print(radar_post);
            Serial.print(F("cm"));
            spazi = 21 - lunghezza_int(radar_post);
            for(int i=0;i<spazi;i++)
              {
              Serial.print(F(" "));
              }
            Serial.println(F("|"));
            Serial.println(F("+=========================================+"));
  }
  
  void loop() {

    
      //pwm.setPWM(12, 4096, 0);
//    Serial.println( F("Posizione sterzo"));
//    Serial.println( posizione_sterzo);
    while (Serial.available() > 0 ) {
//     if(risparmio_energetico == ATTIVATO)
//        pca_on_off();
     //String str = Serial.readString();
     int str = Serial.read();
     //str.trim();
     if (str=='a') 
            {
              int p = 0;
              if(posizione_sterzo - valore_sterzata >= STERZO_SINISTRA_MAX)
              {
                posizione_sterzo = posizione_sterzo - valore_sterzata;
                sterzo(posizione_sterzo);
                Serial.print( F("Sterzo: "));
                Serial.println(posizione_sterzo/*posizione_sterzo*/);
                delay(100);
               }
        }else if (str=='d') 
            {
              int p = 0;
              if(posizione_sterzo + valore_sterzata <= STERZO_DESTRA_MAX)
              {
                posizione_sterzo = posizione_sterzo + valore_sterzata;
                sterzo(posizione_sterzo);
                Serial.print( F("Sterzo: "));
                Serial.println(posizione_sterzo/*posizione_sterzo*/);
                delay(100);
                }
            }else if (str=='w') {
        avanti(0);
     }else if (str=='s') {
        indietro();
     }else if (str=='e') {
        anticollisione = ATTIVATO;
        Serial.print( F("Anticollisione attivata automaticamente"));
        avanzamento_continuo();
        
     }else if (str=='q') {
        avanti(525);
     }else if (str=='z') {
        if(precisione_sterzo <10)
        {
        precisione_sterzo++;
        if(110 % precisione_sterzo != 0){
          precisione_sterzo++;
          valore_sterzata = 110/precisione_sterzo;
        }
        
        }else{
        Serial.println( F("Limiti di precisione raggiunti"));
     }
        Serial.print( F("Precisione sterzo: "));
        Serial.println(precisione_sterzo);
        Serial.print( F("Valore sterzata: "));
        Serial.println(valore_sterzata);
     }else if (str=='x') {
        if(precisione_sterzo>1)
        {
        precisione_sterzo--;
        if(110 % precisione_sterzo != 0){
          precisione_sterzo--;
          valore_sterzata = 110/precisione_sterzo;
        }
        
        }else{
        Serial.println( F("Limiti di precisione raggiunti"));
     }
        Serial.print( F("Precisione sterzo: "));
        Serial.println(precisione_sterzo);
        Serial.print( F("Valore sterzata: "));
        Serial.println(valore_sterzata);
     }else if (str=='pannello_dx') {
        right_panel();
     }else if (str=='4') {
        if(ruote_sterzanti_post == ATTIVATO && ruote_sterzanti_ant == ATTIVATO)
        {
            ruote_sterzanti_post = DISATTIVATO;
            sterzo(STERZO_DRITTO);
            Serial.println( F("Sterzo posteriore disattivato"));
            
        }else if(ruote_sterzanti_post == DISATTIVATO)
        {
            ruote_sterzanti_post = ATTIVATO;
            Serial.println( F("Sterzo posteriore attivato"));
        }else {Serial.println( F("Impossibile disattivare lo sterzo posteriore: attiva lo sterzo anteriore per disattivare il posteriore"));}
//     }else if (str=='5') {
//        if(ruote_sterzanti_ant == ATTIVATO && ruote_sterzanti_post == ATTIVATO)
//        {
//            ruote_sterzanti_ant = DISATTIVATO;
//            sterzo(STERZO_DRITTO);
//            Serial.println( F("Sterzo anteriore disattivato"));
//            
//        }
//     } else if(ruote_sterzanti_ant == DISATTIVATO)
//        {
//            ruote_sterzanti_ant = ATTIVATO;
//            Serial.println( F("Sterzo anteriore attivato"));
//        }else {Serial.println( F("Impossibile disattivare lo sterzo anteriore: attiva lo sterzo posteriore per disattivare l'anteriore"));}
     }else if (str=='pannello_sx') {
        left_panel();
     }else if (str=='0') {
        radar_position = radar_position - 10;
        radar_motor(radar_position);
        Serial.print( F("Posizione RADAR: "));
        Serial.println(radar_position);
     }else if (str=='9') {
        radar_position = radar_position + 10;
        radar_motor(radar_position);
        Serial.print( F("Posizione RADAR: "));
        Serial.println(radar_position);
     }else if (str=='p') {
        radar_update();
        if(radar_dx_ant > 20 && radar_dx_center > 20 && radar_sx_ant > 20 && radar_sx_center > 20)
            open_panels();
        else Serial.println(F("Possibili ostacoli ai lati del rover"));
     }else if (str=='o') {
        close_panels();
     }else if(str=='r') {
        radar_s();
       }else if(str=='t') {
      Serial.print(F("|   "));
      for(int i = 0; i<13; i++)
      {
          radar_motor(radar_scan_motor[i]);
          radar_scan[i] = hc.dist(0);
          delay(100);
      }
      for(int i = 0; i<13; i++)
      {
            
            Serial.print(radar_scan[i]);
            Serial.print(F("  |   "));
      }
      radar_motor(370);
      Serial.println(F("Scan eseguito"));
     }else if (str=='1') {
        if(webcam_sterzo == DISATTIVATO)
          {
              webcam_sterzo = ATTIVATO;
              Serial.println(F("Webcam Sterzo attivato"));
          }
        else if(webcam_sterzo = ATTIVATO)
         {
             webcam_sterzo = DISATTIVATO;
             Serial.println(F("Webcam Sterzo disattivato"));
         }
     }else if (str=='5') {
        if(anticollisione == DISATTIVATO)
          {
              anticollisione = ATTIVATO;
              Serial.println(F("Anticollisione attivata"));
          }
        else if(anticollisione = ATTIVATO)
         {
             anticollisione = DISATTIVATO;
             Serial.println(F("Anticollisione disattivata"));
         }
     }else if(str=='4') {
        _webcam_sterzo(330);
     }else if(str=='2') {
        sterzo_dritto();
     }else if (str=='6') {
        if(sterzo_automatico == DISATTIVATO)
          {
              sterzo_automatico = ATTIVATO;
              Serial.println(F("Sterzo automatico attivato"));
          }
        else if(sterzo_automatico = ATTIVATO)
         {
             sterzo_automatico = DISATTIVATO;
             Serial.println(F("Sterzo automatico disattivato"));
         }
     }/**/else if (str=='n') 
        {
          radar_update();
        if(radar_sx_ant > 20 && radar_sx_center > 20)
            peso_a_sinistra();
        else Serial.println(F("Possibili ostacoli a sinistra del rover"));
          
        }else if (str=='m') 
            {
        radar_update();
        if(radar_dx_ant > 20 && radar_dx_center > 20)
            peso_a_destra();
        else Serial.println(F("Possibili ostacoli a sinistra del rover"));
              
                
        }else if (str=='b') 
            {
                if(risparmio_energetico == DISATTIVATO)
                    {
                      risparmio_energetico = ATTIVATO;
                      Serial.println(F("Risparmio energetico attivato"));
                    }else if(risparmio_energetico ==ATTIVATO)
                    {
                      Serial.println(F("Risparmio energetico disattivato"));
                      risparmio_energetico = DISATTIVATO;
                    }
        }else if (str=='i') 
            {
                if(webcam_sinistra_destra > 235 && webcam_sinistra_destra < 355 && webcam_sopra_sotto + 15 > 435){
                    Serial.println(F("Movimento massimo camera raggiunto"));
                }else{
                webcam_sopra_sotto = webcam_sopra_sotto+15;
               // webcam_sopra_sotto_old = webcam_sopra_sotto;
                webcam_control();
                webcam_Y--;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_sinistra_destra);
                Serial.print(F(", "));
                Serial.print(webcam_sopra_sotto);
                Serial.println(F(" )"));
                }
                
        }else if (str=='k') 
            {
//                if(webcam_sopra_sotto_effettuato == 0 && webcam_sopra_sotto_old > 435)
//                {
//                  webcam_sopra_sotto_effettuato = 1;
//                }
                webcam_sopra_sotto = webcam_sopra_sotto-15;
                //webcam_sopra_sotto_old = webcam_sopra_sotto;
                webcam_control();
                webcam_Y++;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_sinistra_destra);
                Serial.print(F(", "));
                Serial.print(webcam_sopra_sotto);
                Serial.println(F(" )"));
            
        }else if (str=='l') 
            {
                webcam_sinistra_destra = webcam_sinistra_destra-15;
                webcam_control();
                webcam_X++;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_sinistra_destra);
                Serial.print(F(", "));
                Serial.print(webcam_sopra_sotto);
                Serial.println(F(" )"));
                
        }else if (str=='j') 
            {
                webcam_sinistra_destra = webcam_sinistra_destra+15;
                webcam_control();
                webcam_X--;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_sinistra_destra);
                Serial.print(F(", "));
                Serial.print(webcam_sopra_sotto);
                Serial.println(F(" )"));
                
            }else if (str=='h') {
          //DA AGGIORNARE
        Serial.println( F("SilverRover 1.2.0"));
        Serial.println( F("Ver. 1.1.0 (Sterzo Posteriore. disatt. di default, avanzamento continuo con E, modalità radar aggiornata)"));
        Serial.println( F("Ver. 1.2.0 (Aggiunto angolo di Ackermann, sterzo automatico per evitare gli ostacoli, controllo ostacoli prima di aprire i pannelli)"));
        Serial.println( F("COMANDI DISPONIBILI:"));
        Serial.println( F("1)  |w||a||s||d|: avanti, sterza a sinistra, sterza a destra, indietro"));
        Serial.println( F("2)  |i||j||k||l|: pan/til webcam"));
        Serial.println( F("3)  stop: se è in movimento, il rover si ferma"));
        Serial.println( F("4)  pannello_dx: apre il pannello solare destro (SCONSIGLIATO)"));
        Serial.println( F("5)  pannello_sx: apre il pannello solare sinistro (SCONSIGLIATO)"));
        Serial.println( F("6)  |p|: apre i pannelli solari"));
        Serial.println( F("7)  |o|: chiude i pannelli solari"));
        Serial.println( F("8)  |r|: effettua la scansione con tutti i sonar (avanti, dietro, sinistra, destra) riportando la distanza in cm di eventuali ostacoli rilevati"));
        Serial.println( F("9)  |t|: effettua la scansione con il sonar anteriore, questo sonar è motorizzato,"));
        Serial.println( F("10) |n|: se abilitato mantiene aperto soltanto il pannello solare sinistro per aumentare il peso verso sinistra"));
        Serial.println( F("11) |m|: se abilitato mantiene aperto soltanto il pannello solare destro per aumentare il peso verso destra"));
        Serial.println( F("12) |z|: dimunuisce la precisione dello sterzo diminuendo gli step di sterzata"));
        Serial.println( F("13) |x|: aumenta la precisione dello sterzo aumentando gli step di sterzata"));
        Serial.println( F("14) |1|: se abilitato, la webcam ruoterà insieme alle ruote anteriori"));
        Serial.println( F("15) |2|: posizione lo sterzo dritto (valore: 340)"));
//        Serial.println( F("16) |3|: attiva faro infrarosso"));
        Serial.println( F("17) |4|: attiva/disattiva lo sterzo anteriore (default: ATTIVATO)"));
        Serial.println( F("18) |5|: attiva/disattiva lo sterzo posteriore (default: ATTIVATO)"));
     }//else if(radar_alarm())
//     {  
//        Serial.println( F("RADAR ALARM!"));
//        forward(STOP);
//     }
      /**/else{
        Serial.println( F("Comando sconosciuto...riprova"));
     }
      //pwm.setPWM(0, 0, pulselen);
      //elay(200);
//      if(risparmio_energetico == ATTIVATO)
//        pca_on_off();
   }
   if(anticollisione == ATTIVATO)
   {
   if(radar_ant_alarm())
     {  
        Serial.println( F("RADAR ANT ALARM!"));
        forward(STOP);
     }
     if(radar_sx_alarm()==true)
     {  
        uint16_t pos_sterzo_aus;
        Serial.println( F("RADAR SX ALARM!"));
//        while(radar_sx_alarm())
//        { && posizione_sterzo < 330
        if(sterzo_automatico == ATTIVATO)
        {
        posizione_sterzo = 440;
                sterzo(posizione_sterzo);
                Serial.print( F("Sterzo: "));
                Serial.println(posizione_sterzo/*posizione_sterzo*/);
        //}
        delay(1500);
        posizione_sterzo = DRITTO;
        sterzo(posizione_sterzo);
        }else forward(STOP);
     }
     if(radar_dx_alarm())
     {  
        uint16_t pos_sterzo_aus = posizione_sterzo;
        Serial.println( F("RADAR DX ALARM!"));
//        while(radar_dx_alarm())
//         && posizione_sterzo >330
        if(sterzo_automatico == ATTIVATO)
        {
        posizione_sterzo = 220;
                sterzo(posizione_sterzo);
                Serial.print( F("Sterzo: "));
                Serial.println(posizione_sterzo/*posizione_sterzo*/);
        //}
        delay(1500);
        posizione_sterzo = DRITTO;
        sterzo(posizione_sterzo);
        }else forward(STOP);
     }
     if(radar_post_alarm())
     {  
        Serial.println( F("RADAR POST ALARM!"));
        forward(STOP);
     }
   }
   delay(200);
  }

     
  
