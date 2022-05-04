#include <Ps3Controller.h>
#include <ESP32Servo.h>

////// DEFINICIONES //////////

#define motFrenteA    0
#define motAtrasA     1     //canales de PWM no pines
#define motFrenteB    2
#define motAtrasB     3

#define motFrenteAP   25
#define motAtrasAP    26     //PINES
#define motFrenteBP   12
#define motAtrasBP    13

#define pinRampa        11
#define pinTorreta      5
#define rampaArriba     0
#define rampaAbajo      180
#define torretaSpeed    0
#define torretaReversa  200

//////////////////////////////
///////// ESTRUCTURAS /////////

typedef struct PWM{
  int freq = 500;
  int Channel = 0;
  const int resolution = 8;
};

typedef struct buttonsState {
  // Right side buttons data
  bool crossBtn = false;
  bool squareBtn = false;
  bool triangleBtn = false;
  bool circleBtn = false;
  // Left side buttons data
  bool upBtn = false;
  bool rightBtn = false;
  bool downBtn = false;
  bool leftBtn = false;
  // Right shoulder and trigger buttons
  bool rShoulderBtn = false;
  bool rShoulderTrigBtn = false;
  
  //Left shoulder and trigger buttnons
  bool lShoulderBtn = false;
  bool lShoulderTrigBtn = false;

  // Right Joy Stick
  int rightXAxis = 0;
  int rightYAxis = 0;
  bool rightJoyBtn = false;

  // Left Joy Stick
  int leftXAxis = 0;
  int leftYAxis = 0;
  bool leftJoyBtn = false;

  // Special buttons
  bool selectBtn = false;
  bool startBtn = false;
  bool ps3Btn = false;

  // Battery Level
  int batteryLevel = 0;
};

///////////////////////////////
//////// ENCABEZADOS ///////////////

void moveControl(int x, int y);
void notify();
void onConnect();
void toggleMode();
void modoAutonomo();
void control();
void motorOFF();
void adelante(int velocidad);
void atras(int velocidad);
void izquierda(int velocidad);
void derecha(int velocidad);
void frenteIzq(int velocidad);
void frenteDer(int velocidad);
void atrasDer(int velocidad);
void atrasIzq(int velocidad);

///////////////////////////////////
//////// VARIABLES GLOBALES ///////

Servo rampa;
Servo torreta;
bool autoStatus = false;
bool rampaFlag = false;
int normalVel= 255; //Velocidad default, que sea menor a 255
int topVel = 255; //Velocidad máxima
int posRamp = 0;
int posTorreta;
int speedSet = 100;
const int freq = 5000;
const int derChannel = 0;
const int izqChannel = 1;
const int resolution = 8;
struct buttonsState currentState;

//////////////////////////////////
////////// PROGRAMA /////////////////
void setup()
{
    Serial.begin(115200);
    Serial.println("Piloto abordando Sumobot.");
    // CONFIGURAMOS PWM PARA LOS MOTORES
    ledcSetup(motFrenteA, freq, resolution);
    ledcSetup(motAtrasA, freq, resolution);
    ledcSetup(motFrenteB, freq, resolution);
    ledcSetup(motAtrasB, freq, resolution);
  
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(motFrenteAP, motFrenteA);
    ledcAttachPin(motAtrasAP, motAtrasA);
    ledcAttachPin(motFrenteBP, motFrenteB);
    ledcAttachPin(motAtrasBP, motAtrasB);

    rampa.attach(pinRampa);
    rampa.write(rampaAbajo);

    ledcWrite(motFrenteA, 0);
    ledcWrite(motFrenteB, 0);
    ledcWrite(motAtrasA, 0);
    ledcWrite(motAtrasB, 0);

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("0c:ee:e6:27:91:49");

    Serial.print("Ready, Waiting for connection");
}

void loop()
{
    if(!Ps3.isConnected()) {
        Serial.print(".");
        delay(1000);
    }
    
}

//////////////////////////////////////////////////
//////// DEFINICIÓN DE FUNCIONES /////////////////
void notify()
{
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.analog_changed.button.cross > 200 ) currentState.crossBtn = true;
    if( Ps3.event.analog_changed.button.cross < 100 ) currentState.crossBtn = false;

    if( Ps3.event.analog_changed.button.square > 200 ) currentState.squareBtn = true;
    if( Ps3.event.analog_changed.button.square < 100 < 100 ) currentState.squareBtn = false;

    if( Ps3.event.analog_changed.button.triangle > 200 ) currentState.triangleBtn = true;
    if( Ps3.event.analog_changed.button.triangle < 100 ) currentState.triangleBtn = false;

    if( Ps3.event.analog_changed.button.circle > 200 ) currentState.circleBtn = true;
    if( Ps3.event.analog_changed.button.circle < 100 ) currentState.circleBtn = false;

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.analog_changed.button.up > 200 ) currentState.upBtn = true;
    if( Ps3.event.analog_changed.button.up < 100 ) currentState.upBtn = false;

    if( Ps3.event.analog_changed.button.right > 200 ) currentState.rightBtn = true;
    if( Ps3.event.analog_changed.button.right < 100 ) currentState.rightBtn = false;

    if( Ps3.event.analog_changed.button.down > 200 ) currentState.downBtn = true;
    if( Ps3.event.analog_changed.button.down < 100 ) currentState.downBtn = false;

    if( Ps3.event.analog_changed.button.left > 200 ) currentState.leftBtn = true;
    if( Ps3.event.analog_changed.button.left < 100 ) currentState.leftBtn = false;

    //------------- Digital shoulder button events -------------
    if( Ps3.event.analog_changed.button.l1 > 200 ) currentState.lShoulderBtn = true;
    if( Ps3.event.analog_changed.button.l1 < 100 ) currentState.lShoulderBtn = false;

    if( Ps3.event.analog_changed.button.r1 > 200 ) currentState.rShoulderBtn = true;
    if( Ps3.event.analog_changed.button.r1 < 100 ) currentState.rShoulderBtn = false;

    //-------------- Digital trigger button events -------------
    if( Ps3.event.analog_changed.button.l2 > 200 ) currentState.lShoulderTrigBtn = true;
    if( Ps3.event.analog_changed.button.l2 < 100 ) currentState.lShoulderTrigBtn = false;

    if( Ps3.event.analog_changed.button.r2 > 200 ) currentState.rShoulderTrigBtn = true;
    if( Ps3.event.analog_changed.button.r2 < 100 ) currentState.rShoulderTrigBtn = false;

    //--------------- Digital stick button events --------------
    if( Ps3.event.button_down.l3 ) currentState.leftJoyBtn = true;
    if( Ps3.event.button_up.l3 ) currentState.leftJoyBtn = false;

    if( Ps3.event.button_down.r3 ) currentState.rightJoyBtn = true;
    if( Ps3.event.button_up.r3 ) currentState.rightJoyBtn = false;


    //---------- Digital select/start/ps button events ---------
    if( Ps3.event.button_down.select ) currentState.selectBtn = true;
    if( Ps3.event.button_up.select ) currentState.selectBtn = false;

    if( Ps3.event.button_down.start ) currentState.startBtn = true;
    if( Ps3.event.button_up.start ) currentState.startBtn = false;

    if( Ps3.event.button_down.ps ) currentState.ps3Btn = true;
    if( Ps3.event.button_up.ps ) currentState.ps3Btn = false;

    //---------------- Analog stick value events ---------------
    // Left joy stick
   //if( abs(Ps3.event.analog_changed.stick.lx) > 0 || abs(Ps3.event.analog_changed.stick.ly) > 0 ) {
        currentState.leftXAxis = Ps3.data.analog.stick.lx;
        currentState.leftYAxis = Ps3.data.analog.stick.ly;
//        Serial.print("Left X: ");
//        Serial.println(currentState.leftXAxis);
//        Serial.print("Left Y: ");
//        Serial.println(currentState.leftYAxis);
    //}

    //Right joy stick
   //if( abs(Ps3.event.analog_changed.stick.rx) > 0 || abs(Ps3.event.analog_changed.stick.ry) > 0 ) {
       currentState.rightXAxis = Ps3.data.analog.stick.rx;
       currentState.rightYAxis = Ps3.data.analog.stick.ry;
       Serial.print("Right X: ");
       Serial.println(currentState.rightXAxis);
       Serial.print("Right Y: ");
       Serial.println(currentState.rightYAxis);
   //}

   //---------------------- Battery events ---------------------
    if( currentState.batteryLevel != Ps3.data.status.battery ){
        currentState.batteryLevel = Ps3.data.status.battery;
        Serial.print("The controller battery is ");
        if( currentState.batteryLevel == ps3_status_battery_charging )      Serial.println("charging");
        else if( currentState.batteryLevel == ps3_status_battery_full )     Serial.println("FULL");
        else if( currentState.batteryLevel == ps3_status_battery_high )     Serial.println("HIGH");
        else if( currentState.batteryLevel == ps3_status_battery_low)       Serial.println("LOW");
        else if( currentState.batteryLevel == ps3_status_battery_dying )    Serial.println("DYING");
        else if( currentState.batteryLevel == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
        else Serial.println("UNDEFINED");
    }
    

//    // printing the state of all the button
//    if(currentState.rShoulderBtn) {
//      speedSet += 10;
//      Serial.print("Speed is : ");
//      Serial.println(speedSet);
//      if (speedSet >= 255) speedSet = 255;
//    }
//    if(currentState.rShoulderTrigBtn) {
//      speedSet -= 10;
//      Serial.print("Speed is : ");
//      Serial.println(speedSet);
//      if (speedSet <= 100) speedSet = 100;
//    }
    
    control(); 
}

void onConnect(){
    Serial.println("Connected.");
    Ps3.setPlayer(1);
}

//////////// movimiento //////////

void control() {
  int vel;
  
  if(!currentState.rShoulderTrigBtn) //No hay movimiento
  {
    Serial.println("-----------");
    motorOFF();
  }
  
  //////////////// BOTONES
  if(currentState.circleBtn) //Si el botón B está oprimido, se mueve con turbo
  { vel = topVel; Serial.print("!");}
  else { vel = normalVel; }

  if(currentState.crossBtn) //Alterna entre posiciones de rampa
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
  
  if(currentState.lShoulderTrigBtn) //Disparar ligas
  { torreta.attach(pinTorreta); torreta.write(torretaSpeed); Serial.print("piu piu!");}
  else { torreta.detach(); delay(10); }
  if(currentState.lShoulderBtn) //Recargar ligas
  { torreta.attach(pinTorreta); torreta.write(torretaReversa); Serial.print("Reload torreta");}
  else { torreta.detach(); delay(10); }
  
  
  
  //////////////// DPAD
  if(currentState.rShoulderBtn){
     if(currentState.leftYAxis <= -10 && currentState.leftXAxis > 10) //Para frente derecha
     {
        Serial.println("/°");
        frenteDer(vel);
     }
     else if(currentState.leftYAxis <= -10 && currentState.leftXAxis <= -10) //Para frente izquierda
     {
       Serial.println(" °\\ ");
       frenteIzq(vel);
     }
     else if(currentState.leftYAxis >= 10 && currentState.leftXAxis >= 10) //Para atras derecha
     {
       Serial.println("\\.");
       atrasDer(vel);
     }
     else if(currentState.leftYAxis >= 10 && currentState.leftXAxis <= -10) //Para atras izquierda
     {
       Serial.println("./");
       atrasIzq(vel);
     }
  
  ////////////////
  
     else if(currentState.leftYAxis <= -10 && abs(currentState.leftXAxis) < 10) //Para adelante
     {
       Serial.println("^");
       adelante(vel);
     }
     else if(currentState.leftYAxis >= 10 && abs(currentState.leftXAxis) < 10) //Para atrás
     {
       Serial.println("v");
       atras(vel);
     }
     else if(currentState.leftXAxis >= 10 && abs(currentState.leftYAxis) < 10) //Para derecha
     {
       Serial.println(">");
       derecha(vel);
     }
     else if(currentState.leftXAxis <= -10 && abs(currentState.leftYAxis) < 10) //Para izquierda
     {
       Serial.println("<");
       izquierda(vel);
      }
  }
}

//----------------------------------------------------

void motorOFF()
{
  ledcWrite(motFrenteA, 0);
  ledcWrite(motFrenteB, 0);
  ledcWrite(motAtrasA, 0);
  ledcWrite(motAtrasB, 0);
}

void adelante(int velocidad)
{
  ledcWrite(motFrenteA, velocidad);  
  ledcWrite(motAtrasA, LOW);  
  ledcWrite(motFrenteB, velocidad);  
  ledcWrite(motAtrasB, LOW); 
}

void atras(int velocidad)
{
  ledcWrite(motFrenteA, LOW);  
  ledcWrite(motAtrasA, velocidad);  
  ledcWrite(motFrenteB, LOW);  
  ledcWrite(motAtrasB, velocidad); 
}

void izquierda(int velocidad)
{
  ledcWrite(motFrenteA, velocidad);  
  ledcWrite(motAtrasA, LOW);  
  ledcWrite(motFrenteB, LOW);  
  ledcWrite(motAtrasB, velocidad); 
}

void derecha(int velocidad)
{
  ledcWrite(motFrenteB, velocidad);  
  ledcWrite(motAtrasB, LOW);  
  ledcWrite(motFrenteA, LOW);  
  ledcWrite(motAtrasA, velocidad); 
}

void frenteIzq(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  ledcWrite(motFrenteA, velocidad);  
  ledcWrite(motAtrasA, LOW);  
  ledcWrite(motFrenteB, midVelocidad);  
  ledcWrite(motAtrasB, LOW); 
}

void frenteDer(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  ledcWrite(motFrenteA, midVelocidad);  
  ledcWrite(motAtrasA, LOW);  
  ledcWrite(motFrenteB, velocidad);  
  ledcWrite(motAtrasB, LOW); 
}

void atrasDer(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  ledcWrite(motFrenteA, LOW);  
  ledcWrite(motAtrasA, midVelocidad);  
  ledcWrite(motFrenteB, LOW);  
  ledcWrite(motAtrasB, velocidad); 
}

void atrasIzq(int velocidad)
{
  int midVelocidad = velocidad/3;
  
  ledcWrite(motFrenteA, LOW);  
  ledcWrite(motAtrasA, velocidad);  
  ledcWrite(motFrenteB, LOW);  
  ledcWrite(motAtrasB, midVelocidad); 
}
