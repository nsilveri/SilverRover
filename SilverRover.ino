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
uint8_t radar_sterzo = ATTIVATO;
//uint8_t pca_off = ATTIVATO;
uint16_t posizione_sterzo = STERZO_DRITTO;
uint16_t webcam_sopra_sotto = 250;
uint16_t webcam_sinistra_destra = 268;
uint8_t risparmio_energetico = DISATTIVATO;
uint8_t faro = DISATTIVATO;
uint16_t radar_delay = 100;
uint16_t precisione_sterzo = 2;
uint16_t valore_sterzata = 55;
int8_t webcam_X = 0;
int8_t webcam_Y = 0;
int8_t ruote_sterzanti_post = ATTIVATO;
int8_t ruote_sterzanti_ant = ATTIVATO;


void setup() {
  Serial.begin(9600);
  Serial.println( F("Welcome to the SilverRover"));
  Serial.println( F("Insert comand to send to the Rover."));
  Serial.println( F("scrivi 'help' per vedere i comandi disponibili"));
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pinMode(7,OUTPUT);
//  pca_on_off();//attiva pca
  sterzo_dritto();
  

  
  delay(200);
  
//  pca_on_off();//disattiva pca
}

//void pca_on_off()
//{
//  if(pca_off == DISATTIVATO)
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

void webcam_control()
{
  pwm.setPWM(15, 0, webcam_sinistra_destra);
  pwm.setPWM(14, 0, webcam_sopra_sotto);
}

void _webcam_sterzo(uint16_t pulselen)
{
  webcam_sinistra_destra = pulselen-60;
  webcam_sopra_sotto = 250;
  pwm.setPWM(15, 0, webcam_sinistra_destra);
  pwm.setPWM(14, 0, webcam_sopra_sotto);
}

void sterzo_anteriore(uint16_t pulselen)
{
  pwm.setPWM(0, 0, pulselen-8);//-5
  pwm.setPWM(1, 0, pulselen+68);
}

void sterzo_posteriore(uint16_t pulselen)
{
  if(webcam_sterzo == ATTIVATO)
    _webcam_sterzo(pulselen);
  if(radar_sterzo ==ATTIVATO)
    radar_motor(pulselen);
    delay(200);
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
  }

  
//  void sterzo_sinistra_1()
//  {
//    posizione_sterzo = SINISTRA_1;
//    sterzo(STERZO_SINISTRA_1);
//    //_sterzo_sinistro = ATTIVATO;
//  }
//  
//  void sterzo_sinistra_2()
//  {
//    posizione_sterzo = SINISTRA_2;
//    sterzo(STERZO_SINISTRA_2);
//    //_sterzo_sinistro = ATTIVATO;
//  }
//
//  void sterzo_sinistra_3()
//  {
//    posizione_sterzo = SINISTRA_3;
//    sterzo(STERZO_SINISTRA_3);
//    //_sterzo_sinistro = ATTIVATO;
//  }
//
//  void sterzo_sinistra_4()
//  {
//    posizione_sterzo = SINISTRA_4;
//    sterzo(STERZO_SINISTRA_4);
//    //_sterzo_sinistro = ATTIVATO;
//  }
//
//  void sterzo_sinistra_5()
//  {
//    posizione_sterzo = SINISTRA_5;
//    sterzo(STERZO_SINISTRA_5);
//    //_sterzo_sinistro = ATTIVATO;
//  }
//
//  
  void sterzo_dritto()
  {
    posizione_sterzo = DRITTO;
    sterzo(STERZO_DRITTO);
  }
//
//    void sterzo_destra_1()
//  {
//    posizione_sterzo = DESTRA_1;
//    sterzo(STERZO_DESTRA_1);
//    //_sterzo_destro = ATTIVATO;
//  }
//
//  void sterzo_destra_2()
//  {
//    posizione_sterzo = DESTRA_2;
//    sterzo(STERZO_DESTRA_2);
//    //_sterzo_destro = ATTIVATO;
//  }
//
//  void sterzo_destra_3()
//  {
//    posizione_sterzo = DESTRA_3;
//    sterzo(STERZO_DESTRA_3);
//    //_sterzo_destro = ATTIVATO;
//  }
//
//  void sterzo_destra_4()
//  {
//    posizione_sterzo = DESTRA_4;
//    sterzo(STERZO_DESTRA_4);
//    //_sterzo_destro = ATTIVATO;
//  }
//
//  void sterzo_destra_5()
//  {
//    posizione_sterzo = DESTRA_5;
//    sterzo(STERZO_DESTRA_5);
//    //_sterzo_destro = ATTIVATO;
//  }

  


  
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
    pwm.setPWM(13, 0, angle_motor+15);
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
  }//*/
//  void delay_millis(int delay_time)
//  {
//    if(millis() >= time_now + 2100)
//                  {
//                    time_now += period;
//                    
//                    if(hc.dist(0) < 12)
//                       forward(STOP);
//                    
//                  }
//  }
  
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
              Serial.print( F("Distanza: "));
              Serial.println(distance_ant);
              if(distance_ant>520 && run_delay_1 == 21000)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 5mt"));
                  delay(run_delay_1);
                  forward(STOP);
              }else if(distance_ant>55 && run_delay_1 == 0)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 50cm "));
                  delay(2100);
//                  if(millis() >= time_now + 2100)
//                  {
//                    time_now += period;
//                    
//                    if(hc.dist(0) < 12)
//                       forward(STOP);
//                    
//                  }
                  forward(STOP);
              }else if(distance_ant>12 && run_delay_1 == 525)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 10cm "));
                  delay(run_delay_1);
                  forward(STOP);
              }
              else if(distance_ant<55 && distance_ant>25 && run_delay_1 == 0)
              {
                  forward(FORWARD);
                  Serial.println( F("Avanti per 25cm"));
                  delay(1050);
                  forward(STOP);
              }else if(distance_ant<25 && run_delay_1 == 0)
              {
                  Serial.println(F(" Sei troppo vicino ad un ostacolo, azione non consentita."));
                  if(hc.dist(1)>25){
                  forward(BACK);
                  Serial.println( F("Indietro per 25cm per consentirti di sterzare."));
                  delay(1050);
                  forward(STOP);
                  }else Serial.println( F("OSTACOLO SIA AVANTI CHE DIETRO...SEI BLOCCATO"));
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
              if(posizione_sterzo - valore_sterzata >= STERZO_SINISTRA_MAX)
              {
                posizione_sterzo = posizione_sterzo - valore_sterzata;
                sterzo(posizione_sterzo);
                Serial.print( F("Sterzo: "));
                Serial.println(posizione_sterzo);
                
//                if(posizione_sterzo != 0)
//                {
//                  posizione_sterzo--;
//                  dati_sterzo();  
               }
        }else if (str=='d') 
            {
              if(posizione_sterzo + valore_sterzata <= STERZO_DESTRA_MAX)
              {
                posizione_sterzo = posizione_sterzo + valore_sterzata;
                sterzo(posizione_sterzo);
                Serial.print( F("Sterzo: "));
                Serial.println(posizione_sterzo);
//                if(posizione_sterzo != 10)
//                {
//                  posizione_sterzo++;
//                  dati_sterzo();
//                }
                }
            }else if (str=='w') {
        avanti(0);
     }else if (str=='s') {
        indietro();
     }else if (str=='e') {
        avanti(12600);
     }else if (str=='q') {
        avanti(525);
//     }else if (str=='stop') {
//        forward(STOP);
//        Serial.println( F("Stop"));
     }else if(str=='3')
      {
        if(faro == DISATTIVATO){
          pwm.setPWM(12, 4096, 0);
          faro = ATTIVATO;
          Serial.println( F("Faro attivato"));
        }else if(faro == ATTIVATO){
          pwm.setPWM(12, 0, 4096);
          faro = DISATTIVATO;
          Serial.println( F("Faro disattivato"));
        }
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
            Serial.print( F("Sterzo posteriore disattivato"));
            
        }else if(ruote_sterzanti_post == DISATTIVATO)
        {
            ruote_sterzanti_post = ATTIVATO;
            Serial.print( F("Sterzo posteriore attivato"));
        }else {Serial.print( F("Impossibile disattivare lo sterzo posteriore: attiva lo sterzo anteriore per disattivare il posteriore"));}
     }else if (str=='5') {
        if(ruote_sterzanti_ant == ATTIVATO && ruote_sterzanti_post == ATTIVATO)
        {
            ruote_sterzanti_ant = DISATTIVATO;
            Serial.print( F("Sterzo posteriore disattivato"));
            
        }else if(ruote_sterzanti_ant == DISATTIVATO)
        {
            ruote_sterzanti_ant = ATTIVATO;
            Serial.print( F("Sterzo posteriore attivato"));
        }else {Serial.print( F("Impossibile disattivare lo sterzo anteriore: attiva lo sterzo posteriore per disattivare l'anteriore"));}
     }else if (str=='pannello_sx') {
        left_panel();
     }else if (str=='p') {
        open_panels();
     }else if (str=='o') {
        close_panels();
     }else if(str=='r') {
            //uint16_t radar_delay = 200;
            Serial.println(F("+=============|RADAR|================="));
            Serial.print(F("|          A: "));
            Serial.print(hc.dist(0));
            Serial.println(F("cm"));
            Serial.println(F("|"));
            delay(radar_delay);
            Serial.print(F("|S: "));
            Serial.print(hc.dist(3));
            Serial.print(F("cm    |   "));
            delay(radar_delay);
            Serial.print(F("D: "));
            Serial.print(hc.dist(2));
            Serial.println(F("cm"));
            Serial.println(F("|"));
            delay(radar_delay);
            Serial.print(F("|          D: "));
            Serial.print(hc.dist(1));
            Serial.println(F("cm"));
            Serial.println(F("+======================================"));
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
     }else if(str=='4') {
        _webcam_sterzo(330);
     }else if(str=='2') {
        sterzo_dritto();
     }/**/else if (str=='n') 
        {
          peso_a_sinistra();
        }else if (str=='m') 
            {
                peso_a_destra();
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
                webcam_sopra_sotto = webcam_sopra_sotto+15;
                webcam_control();
                webcam_Y--;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_X);
                Serial.print(F(", "));
                Serial.print(webcam_Y);
                Serial.println(F(" )"));
                
        }else if (str=='k') 
            {
                webcam_sopra_sotto = webcam_sopra_sotto-15;
                webcam_control();
                webcam_Y++;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_X);
                Serial.print(F(", "));
                Serial.print(webcam_Y);
                Serial.println(F(" )"));
            
        }else if (str=='l') 
            {
                webcam_sinistra_destra = webcam_sinistra_destra-20;
                webcam_control();
                webcam_X++;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_X);
                Serial.print(F(", "));
                Serial.print(webcam_Y);
                Serial.println(F(" )"));
                
        }else if (str=='j') 
            {
                webcam_sinistra_destra = webcam_sinistra_destra+20;
                webcam_control();
                webcam_X--;
                Serial.print(F("Posizione webcam ( "));
                Serial.print(webcam_X);
                Serial.print(F(", "));
                Serial.print(webcam_Y);
                Serial.println(F(" )"));
                
            }
//        }else if (str=='6') 
//            {
//               rover_speed = rover_speed + 10;
//               Serial.print(F("Velocità rover aumentata"));
//        
//        }else if (str=='5') 
//            {
//               rover_speed = rover_speed - 10; 
//               Serial.print(F("Velocità rover diminuita")); 
//        
//        }
          else if (str=='h') {
          //DA AGGIORNARE
        Serial.println( F("SilverRover"));
        Serial.println( F("Ver. 1.0.1 (Aggiunta possibilità di disattivare i singoli sterzi ant e post"));
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
        Serial.println( F("16) |3|: attiva faro infrarosso"));
        Serial.println( F("17) |4|: attiva/disattiva lo sterzo anteriore (default: ATTIVATO)"));
        Serial.println( F("18) |5|: attiva/disattiva lo sterzo posteriore (default: ATTIVATO)"));
     }/**/else{
        Serial.println( F("Comando sconosciuto...riprova"));
     }
      //pwm.setPWM(0, 0, pulselen);
      //elay(200);
//      if(risparmio_energetico == ATTIVATO)
//        pca_on_off();
   }
   delay(400);
  }

     
  
