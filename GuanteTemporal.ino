/*
   Luz led estroboscópica regulada desde un potenciómetro con ajuste de la frecuencia en función del ángulo de inclinacion de un acelerómetro.
   Proyecto basado en TimeGlove de AlexGyver https://github.com/AlexGyver/EnglishProjects/tree/master/TimeGlove
   Autor: Javier Vargas. El Hormiguero.
   https://creativecommons.org/licenses/by/4.0/
*/
//PINES
#define PinMosfet 11
#define PinPot A0
#define PinLed 12

//CONFIGURACION
#define AccONOFF 50 //Variaciones de aceleracion para encender - apagar
#define KfiltComp 0.99f //Filtro complementario de la imu
#define OffsetAng 0
#define AngMax 45
#define Tled 1000 //(ms) periodo de parpadeo del led
#define Ton 200 //(us) tiempo de encendido
unsigned long Tmin = 1000; //(us) Periodo minimo de pulso regulado por potenciometro
unsigned long Tmax = 50000; //(us) Periodo maximo de pulso regulado por potenciometro
unsigned long Tinc = 5000; //(us) incremento del periodo regulado por el angulo

//IMU
#include <MPU6050_CompFilter.h>
MPU6050_CompFilter mpu(0x68);

//PARPADEO LED
#include "BlinkLed.h"
BlinkLed led(PinLed, Tled);

#include <digitalWriteFast.h>
float T0 = 0;
float T1 = 0;
boolean estado = 0;
unsigned long t = 0, m = 0;

void setup() {
  //PinMode
  pinMode(PinMosfet, OUTPUT);
  pinMode(PinPot, INPUT);
  digitalWrite(PinMosfet, LOW);

  //Inicio LED
  led.begin();
  led.Blink();

  //IniciO IMU
  mpu.Iniciar(0.05);
  mpu.setKcompFilter(KfiltComp);
  //Offset obtenido con MPU_Zero de jrowberg
  mpu.setXAccelOffset(-2388);
  mpu.setYAccelOffset(-263);
  mpu.setZAccelOffset(1231);
  mpu.setXGyroOffset(-45);
  mpu.setYGyroOffset(4);
  mpu.setZGyroOffset(-18);

  delay(2000);
  led.On();
}

//////////////////////////////////
//////////////LOOP////////////////
//////////////////////////////////

void loop() {


  //Lectura del periodo en funcion del potenciometro
  T0 = LecturaPotenciometro();
  //Lectura del periodo en funcion de la imu
  T1 = LecturaIMU();
  //encendido y apagado
  InterruptorIMU();

  //Encendido
  if (estado) {
    digitalWriteFast(PinMosfet, HIGH); //Led ON
    delayMicroseconds(Ton);
    digitalWriteFast(PinMosfet, LOW); //Led OFF
    delay(T0 / 1000); //Tiempo de apagado del potenciometro
    delayMicroseconds((int)T0 % 1000); //Tiempo de apagado del potenciometro (decimales)
    delayMicroseconds(T1); //Tiempo de apagado segun el ángulo
  }
  else delay(50);

  //Estado del led
  led.Update();

  //Tiempo de bucle
  t = millis() - m;
  m = millis();
}

//////////////////////////////////
//////////////////////////////////
//////////////////////////////////

//Lectura del potenciometro, devuelve el periodo de la señal
float LecturaPotenciometro() {
  static float val0 = 0;
  //Lectura potenciometro
  float val = map(analogRead(PinPot), 0, 1024, Tmax, Tmin);
  val = constrain(val, Tmin, Tmax);

  return val;
}

float LecturaIMU() {
  static float val0 = 0;
  static unsigned long m = 0;
  
  //Lectura IMU
  mpu.setT((float)t / 1000);
  mpu.Lectura(1, 1);
  float ang = -mpu.angX() + OffsetAng;
  ang = constrain(ang, -AngMax, AngMax);

  //Valor de incremento de periodo segun el angulo obtenido
  float val = map(ang * 10, -AngMax * 10, AngMax * 10, 0, Tinc);

  return val;
}

void InterruptorIMU() {
  static unsigned long m = 0;
  static float a0 = mpu.Yacc();
  float derAcc = mpu.Yacc() - a0;
  a0 = mpu.Yacc();
  
  if (abs(derAcc) > AccONOFF && millis() > m) {
    estado = !estado;
    m = millis() + 500;
    if (estado) {
      //Luz encendida
      led.NoBlink();
      led.On();
    }
    else {
      //Luz apagada
      digitalWrite(PinMosfet, LOW);
      led.Blink();
    }
  }
}



