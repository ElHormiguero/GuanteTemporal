/*Gestiona el parpadeo de un led
  Autor: Javier Vargas García
  https://creativecommons.org/licenses/by/4.0/
*/

#include "BlinkLed.h"

BlinkLed::BlinkLed(int Pin_, int TimeON_, int TimeOFF_){
  Pin = Pin_;
  TimeON = TimeON_;
  TimeOFF = TimeOFF_;
  T = TimeON + TimeOFF;
}

BlinkLed::BlinkLed(int Pin_, int Period){
  Pin = Pin_;
  TimeON = Period / 2;
  TimeOFF = Period / 2;
  T = Period;
}

void BlinkLed::begin() {
  pinMode(Pin, OUTPUT);
  digitalWrite(Pin, LOW);
  active = 0;
  state = 0;
  NoBlink_ = 0;
}

void BlinkLed::Update() {
  //Modo blink
  if (!NoBlink_) {
     if (active) {
        if (millis() > m) {
            //Apagado
            if (state) {
                state = 0;
                m = millis() + TimeOFF;
                digitalWrite(Pin, LOW);
            }
        //Encendido
            else {
                state = 1;
                m = millis() + TimeON;
                digitalWrite(Pin, HIGH);
            }
        }
    }
  }
}

bool BlinkLed::GetStatus() {
    return state;
}

void BlinkLed::Off() {
    if (state) digitalWrite(Pin, LOW);
    state = 0;
    active = 0;
}

void BlinkLed::On() {
    if (NoBlink_ && !state) digitalWrite(Pin, HIGH);
    state = 1;
    active = 1;
}

void BlinkLed::NoBlink() {
    NoBlink_ = 1;
}

void BlinkLed::Blink() {
    NoBlink_ = 0;
}

void BlinkLed::SetPeriod(int Period) {
    TimeON = Period / 2;
    TimeOFF = Period / 2;
    T = Period;
//    if (state) {
//        if (m2 > )
//        if (T = TimeON + TimeOFF;
//    }
}

void BlinkLed::SetTime(int TimeON_, int TimeOFF_) {
    TimeON = TimeON_;
    TimeOFF = TimeOFF_;
    T = TimeON_ + TimeOFF_;
}
