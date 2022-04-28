/*
*  Project     Nintendo Extension Controller Library
*  @author     David Madison
*  @link       github.com/dmadison/NintendoExtensionCtrl
*  @license    LGPLv3 - Copyright (c) 2018 David Madison
*
*  This file is part of the Nintendo Extension Controller Library.
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*  Example:      Classic_Demo
*  Description:  Connect to a Classic Controller and demonstrate all of
*                the avaiable control data functions.
*/

#include <NintendoExtensionCtrl.h>
ClassicController classic;

//#include <Adafruit_NeoPixel.h>
//#define PIN 6
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, PIN, NEO_GRB + NEO_KHZ800);

#include<Servo.h> 
Servo rampa;
Servo torreta;

#define motFrenteA 5
#define motAtrasA 6
#define motFrenteB 9
#define motAtrasB 10

#define pinRampa 11
#define pinTorreta 12
#define rampaArriba 0
#define rampaAbajo 180
#define torretaSpeed 0
#define torretaReversa 200


bool autoStatus = false;
bool rampaFlag = false;
int normalVel= 255; //Velocidad default, que sea menor a 255
int topVel = 255; //Velocidad máxima
int posRamp = 0;
int posTorreta;

void setup() {
  Serial.begin(115200);
  Serial.println("Piloto abordando Sumobot.");
  classic.begin();
  delay(1000); //Por si las dudas
  
  pinMode(motFrenteA, OUTPUT); //Pin con PWM
  pinMode(motAtrasA, OUTPUT); //Pin con PWM
  pinMode(motFrenteB, OUTPUT); //Pin con PWM
  pinMode(motAtrasB, OUTPUT); //Pin con PWM

  rampa.attach(pinRampa);
  rampa.write(rampaAbajo);
  
//  strip.begin();
//  strip.show(); // Initialize all pixels to 'off'
  
  
  while (!classic.connect()) {
    Serial.println("Classic Controller not detected!");
    delay(1000);
  }
  Serial.println("Piloto listo. Arrancando Sumobot.");
}

void loop() { //////////////////////////////////////////////////////// LOOP
  
  boolean success = classic.update();  // Get new data from the controller

  if (!success) {  // Ruh roh
    Serial.println("Controller disconnected!");
    delay(1000);
  }
  else {
    
    if(classic.buttonPlus() && classic.buttonMinus())
    {
      toggleMode();
    }
    
    if(autoStatus == true) modoAutonomo() ;
    else control();

  }
}           //////////////////////////////////////////////////////// LOOP

void toggleMode()
{
  if(autoStatus == false)
  {
    Serial.println("Entrando a Modo Autonomo");
    autoStatus = true;
    delay(500);
  }
  else
  {
    Serial.println("Entrando a Modo Manual");
    autoStatus = false;
    delay(500);
  }
}

void modoAutonomo() 
{
  bool stay = true;
  while(stay)
  {
    boolean success = classic.update();
    Serial.println("Beep, boop");
    digitalWrite(motFrenteA, LOW);  //Atrás
    digitalWrite(motAtrasA, HIGH);  

    digitalWrite(motFrenteB, LOW);  //Atrás
    digitalWrite(motAtrasB, HIGH);  
    //delay(1000);
    
    
    if(classic.buttonPlus() && classic.buttonMinus()) //Salir del modo Automatico
    {
      stay = false;
      motorOFF();
      toggleMode();
    }
  }
  
}

void control() {
  // Read the DPAD (Up/Down/Left/Right) or read a button (ABXY, Minus, Plus)
  int vel;
  boolean padUp = classic.dpadUp();
  boolean padDown = classic.dpadDown();
  boolean padRight = classic.dpadRight();
  boolean padLeft = classic.dpadLeft();
  
  boolean aButton = classic.buttonA();
  boolean bButton = classic.buttonB();
  boolean plusButton = classic.buttonPlus();
  boolean minusButton = classic.buttonMinus();
  
  if(!padUp && !padDown && !padRight && !padLeft) //No hay movimiento
  {
    Serial.println("-----------");
    motorOFF();
  }
  
  //////////////// BOTONES
  if(bButton) //Si el botón B está oprimido, se mueve con turbo
  { vel = topVel; Serial.print("!");}
  else { vel = normalVel; }

  if(plusButton) //Alterna entre posiciones de rampa
  { 
    if(!rampaFlag)
    {
      rampa.write(rampaArriba);
      Serial.print("kya!");
      rampaFlag = true; 
      delay(400); 
    }else if (rampaFlag)
    {
      rampa.write(rampaAbajo);
      Serial.print("no kya!"); 
      rampaFlag = false;
      delay(400); 
    }
    
  }
  //else { rampa.write(rampaAbajo); delay(10);}
  
  if(aButton && !minusButton) //Disparar ligas
  { torreta.attach(pinTorreta); torreta.write(torretaSpeed); Serial.print("piu piu!"); delay(10);}
  else { torreta.detach(); delay(10); }
  if(aButton && minusButton) //Recargar ligas
  { torreta.attach(pinTorreta); torreta.write(torretaReversa); Serial.print("Reload torreta"); delay(10);}
  else { torreta.detach(); delay(10); }
  
  
  
  //////////////// DPAD
    if(padUp && padRight) //Para frente derecha
  {
    Serial.println("/°");
    frenteDer(vel);
  }
  else if(padUp && padLeft) //Para frente izquierda
  {
    Serial.println(" °\\ ");
    frenteIzq(vel);
  }
  else if(padDown && padRight) //Para atras derecha
  {
    Serial.println("\\.");
    atrasDer(vel);
  }
  else if(padDown && padLeft) //Para atras izquierda
  {
    Serial.println("./");
    atrasIzq(vel);
  }
  
  ////////////////
  
  else if(padUp) //Para adelante
  {
    Serial.println("^");
    adelante(vel);
  }
  else if(padDown) //Para atrás
  {
    Serial.println("v");
    atras(vel);
  }
  else if(padRight) //Para derecha
  {
    Serial.println(">");
    derecha(vel);
  }
  else if(padLeft) //Para izquierda
  {
    Serial.println("<");
    izquierda(vel);
  }

}

//----------------------------------------------------

void motorOFF()
{
  digitalWrite(motFrenteA, LOW);  
  digitalWrite(motAtrasA, LOW);  
  digitalWrite(motFrenteB, LOW);  
  digitalWrite(motAtrasB, LOW); 
}

void adelante(int velocidad)
{
  analogWrite(motFrenteA, velocidad);  
  analogWrite(motAtrasA, LOW);  
  analogWrite(motFrenteB, velocidad);  
  analogWrite(motAtrasB, LOW); 
}

void atras(int velocidad)
{
  analogWrite(motFrenteA, LOW);  
  analogWrite(motAtrasA, velocidad);  
  analogWrite(motFrenteB, LOW);  
  analogWrite(motAtrasB, velocidad); 
}

void izquierda(int velocidad)
{
  analogWrite(motFrenteA, velocidad);  
  analogWrite(motAtrasA, LOW);  
  analogWrite(motFrenteB, LOW);  
  analogWrite(motAtrasB, velocidad); 
}

void derecha(int velocidad)
{
  analogWrite(motFrenteB, velocidad);  
  analogWrite(motAtrasB, LOW);  
  analogWrite(motFrenteA, LOW);  
  analogWrite(motAtrasA, velocidad); 
}

void frenteIzq(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  analogWrite(motFrenteA, velocidad);  
  analogWrite(motAtrasA, LOW);  
  analogWrite(motFrenteB, midVelocidad);  
  analogWrite(motAtrasB, LOW); 
}

void frenteDer(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  analogWrite(motFrenteA, midVelocidad);  
  analogWrite(motAtrasA, LOW);  
  analogWrite(motFrenteB, velocidad);  
  analogWrite(motAtrasB, LOW); 
}

void atrasDer(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  analogWrite(motFrenteA, LOW);  
  analogWrite(motAtrasA, midVelocidad);  
  analogWrite(motFrenteB, LOW);  
  analogWrite(motAtrasB, velocidad); 
}

void atrasIzq(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  analogWrite(motFrenteA, LOW);  
  analogWrite(motAtrasA, velocidad);  
  analogWrite(motFrenteB, LOW);  
  analogWrite(motAtrasB, midVelocidad); 
}
