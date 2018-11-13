/*Gestiona el parpadeo de un led
  Autor: Javier Vargas García
  https://creativecommons.org/licenses/by/4.0/
*/

#include "Arduino.h"

class BlinkLed {
 public:
  BlinkLed(int Pin, int TimeON, int TimeOFF); //Constructor
  BlinkLed(int Pin, int Period);
  void begin(); //Pinmode
  void Update(); //Acutaliza el estado del led en funcion del tiempo
  bool GetStatus(); //Devuelve el estado del led
  void Off(); //Apaga el led
  void On(); //Enciende el led
  void NoBlink(); //Desactiva el parpadeo
  void Blink(); //Activa el parpadeo
  void SetPeriod(int Period); //Ajusta el periodo del parpadeo
  void SetTime(int TimeON, int TimeOFF); //Ajusta el tiempo de on/off

private:
  int Pin;
  int TimeON;
  int TimeOFF;
  unsigned long m;
  unsigned long m2;
  int T;
  bool state;
  bool active;
  bool NoBlink_;
};
