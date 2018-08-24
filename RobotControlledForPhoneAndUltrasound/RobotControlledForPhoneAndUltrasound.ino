
#include <ArduinoSTL.h>

#define DEBUG(a) Serial.println(a);

//Constantes Motores
#define MAX_POWER_MOTOR 255
#define MIN_POWER_MOTOR 0

#define MOTOR_RIGHT_PWM      3
#define MOTOR_RIGHT_REARWARD 4
#define MOTOR_RIGHT_FORWARD	 5

#define MOTOR_LEFT_PWM       9
#define MOTOR_LEFT_REARWARD  10
#define MOTOR_LEFT_FORWARD	 11

//Constantes Angulos
#define MAX_PITCH         100
#define MIN_PITCH         0
#define HALF_PITCH        50
#define MAX_ROLL          50 
#define MIN_ROLL         -50
#define HALF_ROLL         0
#define PITCH_OFFSET_NULL 10
#define ROLL_OFFSET_NULL  10

//Constantes HC-SR04
#define LEFT_TRIG  22
#define LEFT_ECHO  23
#define FRONT_TRIG 24
#define FRONT_ECHO 25
#define RIGHT_TRIG 26
#define RIGHT_ECHO 27
#define BACK_TRIG  30
#define BACK_ECHO  31

#define STOP_DISTANCE 5
//ESP

#define GND_ESP8266  7


int overflowAngle(int rawAngle, int min, int max){
    if (rawAngle > max)
        return max;
    if (rawAngle <= min)
        return min;
    return rawAngle; 
}

int anglePitchPhone, angleRollPhone, state = 0,
    speed, powerMotorLeft, powerMotorRight, angle;


HardwareSerial &ESP8266  = Serial1;

long getDistance(int trigger, int echo) {
  pinMode(trigger, OUTPUT);
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  long duracion = pulseIn(echo, HIGH);
  long distancia = (duracion / 2) / 29;
  return distancia;
}


bool enableForward(){
  return  ( (getDistance(FRONT_TRIG, FRONT_ECHO) > STOP_DISTANCE)
          &&(getDistance(LEFT_TRIG,  LEFT_ECHO ) > STOP_DISTANCE)
          &&(getDistance(RIGHT_TRIG, RIGHT_ECHO) > STOP_DISTANCE)
          );
   
}


bool enableRearward(){
  return (getDistance(BACK_TRIG, BACK_ECHO) > STOP_DISTANCE );
}


std::vector<void (*)()>  stateMachine = {
  // -----------------Estado 0 - DETENIDO----------------------//
  []() { 
    if ( (anglePitchPhone < HALF_PITCH - PITCH_OFFSET_NULL) && (enableForward())){
      state = 1;
      //DEBUG("AVANZAR");
    }
    else if ((anglePitchPhone > HALF_PITCH + PITCH_OFFSET_NULL)&& (enableRearward())){
      state = 2;  
      //DEBUG("RETROCEDER");
    }
    else if ((angleRollPhone > HALF_ROLL + ROLL_OFFSET_NULL)&& (enableRearward())){
      state = 3;
      //DEBUG("DERECHA");
    }
    else if ((angleRollPhone < HALF_ROLL - ROLL_OFFSET_NULL) && (enableRearward())){
      state = 4;
      //DEBUG("IZQUIERDA");
    }
    speed = 0;
    digitalWrite(MOTOR_RIGHT_REARWARD,LOW);
    digitalWrite(MOTOR_LEFT_REARWARD, LOW);
    digitalWrite(MOTOR_LEFT_FORWARD , LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  },
  // ------------------Estado 1 - AVANZAR----------------------//
  []() {  
    if ((anglePitchPhone > HALF_PITCH - PITCH_OFFSET_NULL  ) || (!enableForward())){ 
      state = 0;
      DEBUG("DETENIDO");
      //DEBUG(speed);
    }
    angle = overflowAngle(anglePitchPhone,MIN_PITCH,MAX_PITCH);
    speed = map(angle,HALF_PITCH - PITCH_OFFSET_NULL,MIN_PITCH,MIN_POWER_MOTOR,MAX_POWER_MOTOR);
    digitalWrite(MOTOR_RIGHT_REARWARD,LOW);
    digitalWrite(MOTOR_LEFT_REARWARD, HIGH);
    digitalWrite(MOTOR_LEFT_FORWARD , LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  },
  // ------------------Estado 2 - RETROCEDER--------------------//
  []() { 
    if ((anglePitchPhone < HALF_PITCH + PITCH_OFFSET_NULL) || (!enableRearward())){
      state = 0;
      //DEBUG("DETENIDO");
      //DEBUG(speed);
    } 
    angle = overflowAngle(anglePitchPhone,MIN_PITCH,MAX_PITCH);
    speed = map(angle,HALF_PITCH + PITCH_OFFSET_NULL,MAX_PITCH,MIN_POWER_MOTOR,MAX_POWER_MOTOR);
    digitalWrite(MOTOR_RIGHT_REARWARD,HIGH);
    digitalWrite(MOTOR_LEFT_REARWARD, LOW);
    digitalWrite(MOTOR_LEFT_FORWARD , HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    
    
  },
  // ------------------Estado 3 - DERECHA-----------------------//
  []() { 
    if ((angleRollPhone < HALF_ROLL + ROLL_OFFSET_NULL) || (!enableForward())){
      state = 0;
      //DEBUG("DETENIDO");
      //DEBUG(speed);
    } 
    angle = overflowAngle(angleRollPhone,MIN_ROLL,MAX_ROLL);
    speed = map(angle,HALF_ROLL + ROLL_OFFSET_NULL,MAX_ROLL,MIN_POWER_MOTOR,MAX_POWER_MOTOR);
    digitalWrite(MOTOR_RIGHT_REARWARD,HIGH);
    digitalWrite(MOTOR_LEFT_REARWARD, HIGH);
    digitalWrite(MOTOR_LEFT_FORWARD , LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    
  },
  // ------------------Estado 4 - IZQUIERDA----------------------//
  []() { 
    if ((angleRollPhone > HALF_ROLL - ROLL_OFFSET_NULL) || (!enableForward())){
      state = 0;
      //DEBUG("DETENIDO");
      //DEBUG(speed);
    } 
    angle = overflowAngle(angleRollPhone,MIN_ROLL,MAX_ROLL);
    speed = map(angle,HALF_ROLL - ROLL_OFFSET_NULL,MIN_ROLL,MIN_POWER_MOTOR,MAX_POWER_MOTOR);
    digitalWrite(MOTOR_RIGHT_REARWARD,LOW);
    digitalWrite(MOTOR_LEFT_REARWARD, LOW);
    digitalWrite(MOTOR_LEFT_FORWARD , HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);

  }
};


void setup()
{
  Serial.begin(9600);
  ESP8266.begin(9600);
  
  pinMode(MOTOR_LEFT_PWM,     OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_REARWARD,OUTPUT);

  pinMode(MOTOR_RIGHT_PWM,     OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_REARWARD,OUTPUT);

  digitalWrite(MOTOR_RIGHT_REARWARD,LOW);
  digitalWrite(MOTOR_LEFT_REARWARD, LOW);
  digitalWrite(MOTOR_LEFT_FORWARD , LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);

  digitalWrite(GND_ESP8266,LOW);
}
String str;
void loop() {
    if (ESP8266.available()){
      str   =  ESP8266.readString();
      String pitch = str.substring(str.indexOf('i')+1,str.indexOf('m'));
      String roll  = str.substring(str.indexOf('m')+1,str.indexOf('f'));
      angleRollPhone  = roll.toInt();
      anglePitchPhone = pitch.toInt();
    }
    
    stateMachine[state]();
    analogWrite(MOTOR_RIGHT_PWM, speed);
    analogWrite(MOTOR_LEFT_PWM,  speed);
}
  





