#include <AccelStepper.h>
#include <Wire.h> 

#define E_X 9 
#define E_Y 10
#define E_Z 11

#define irPin A0 
#define relayPin 12  
#define relayPin1 13   

int StepX = 2; 
int DirX = 5;

int StepY = 3;
int DirY = 6;

int StepZ = 4;
int DirZ = 7;

int ena = 8;  
unsigned long startTime;
int flag = 0;

AccelStepper mystepperX(1, StepX, DirX, ena); 
AccelStepper mystepperY(1, StepY, DirY, ena);
AccelStepper mystepperZ(1, StepZ, DirZ, ena);

const int positionX[] = {-270, -200, -150, -270, -225, -185, -270, -230, -200}; // Các vị trí cần đến
const int positionY[] = {-100, -100, -150, -150, -150, -190, -210, -220, -250};
const int positionZ[] = {200, 200, 200, 150, 150, 150, 100, 100, 100};
 
void home_YZ()
{
  int homeY = 0;
  int homeZ = 0;

  mystepperY.setMaxSpeed(100);  
  mystepperY.setAcceleration(100);  
  mystepperY.enableOutputs();  

  mystepperZ.setMaxSpeed(100);
  mystepperZ.setAcceleration(100);
  mystepperZ.enableOutputs();

  while (digitalRead(E_Y) == 1 && digitalRead(E_Z) == 1)  
  {
    mystepperY.moveTo(homeY);
    mystepperZ.moveTo(homeZ);
    homeY++;
    homeZ--;
    mystepperY.run();
    mystepperZ.run();
  }

  while (digitalRead(E_Y) == 1)
  {
    mystepperY.moveTo(homeY);
    homeY++;
    mystepperY.run();
  }
  mystepperY.setCurrentPosition(0);

  while (digitalRead(E_Z) == 1)
  {
    mystepperZ.moveTo(homeZ);
    homeZ--;
    mystepperZ.run();
  }
  mystepperZ.setCurrentPosition(0);

  homeY = 0;
  homeZ = 0;
}

void home_X()
{
  int homeX = 0;
  mystepperX.setMaxSpeed(100);
  mystepperX.setAcceleration(100);
  mystepperX.enableOutputs();

  while (digitalRead(E_X) == 1)
  {
    mystepperX.moveTo(homeX);
    homeX++;
    mystepperX.run();
  }
  mystepperX.setCurrentPosition(0);

  homeX = 0;
}
void home()
{
  home_YZ();
  home_X();
}
void setup()
{
  Serial.begin(115200);

  pinMode(irPin, INPUT);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  pinMode(relayPin1, OUTPUT);
  digitalWrite(relayPin1, HIGH);

  pinMode(E_X, INPUT_PULLUP);
  pinMode(E_Y, INPUT_PULLUP);
  pinMode(E_Z, INPUT_PULLUP);

  home();

  mystepperX.setMaxSpeed(200);
  mystepperX.setAcceleration(200);
  mystepperY.setMaxSpeed(200);
  mystepperY.setAcceleration(200);
  mystepperZ.setMaxSpeed(200);
  mystepperZ.setAcceleration(200);
}

void Position(int X, int Y, int Z)
{
  mystepperX.moveTo(X); 
  mystepperY.moveTo(Y);
  mystepperZ.moveTo(Z);
  while (mystepperX.distanceToGo() != 0 || mystepperY.distanceToGo() != 0 || mystepperZ.distanceToGo() != 0) 
    mystepperX.run();
    mystepperY.run();
    mystepperZ.run();
}

void Position_home(int Y, int Z)
{
  mystepperY.moveTo(Y);
  mystepperZ.moveTo(Z);
  while (mystepperY.distanceToGo() != 0 || mystepperZ.distanceToGo() != 0)
  {
    mystepperY.run();
    mystepperZ.run();
  }
}

void loop()
{
  int i;
  String receivedChar = Serial.readString();
  i = receivedChar.toInt();
  if(i == 10){
    digitalWrite(relayPin, HIGH);
    Serial.println("i==10");
  }else
  if (digitalRead(irPin) == 0)                    
  {
    delay(80);
    Serial.println(digitalRead(irPin));
    digitalWrite(relayPin, HIGH);             
    delay(3000);
    if (Serial.available() > 0)                 
    {
      String receivedChar = Serial.readString();  
      i = receivedChar.toInt();                  
      while (i == 9 && digitalRead(irPin) == 0)
      { // pass
        digitalWrite(relayPin, LOW);
        delay(1000);
        String receivedChar = Serial.readString();
        i = receivedChar.toInt();
      }
      digitalWrite(relayPin, HIGH);
        while (i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 8)
        {
          digitalWrite(relayPin1, LOW);             
          Serial.println(digitalRead(irPin));
          Position(-65, -163, 133);                
          delay(500);
          home();
          if (digitalRead(irPin) == 0)              
          {
            Position(-60, -163, 133);
            delay(500);
            home();
          }
          if (digitalRead(irPin) == 0)
          {
            Position(-65, -160, 133);
            delay(500);
            home();
          }
          if (digitalRead(irPin) == 0)
          {
            Position(-60, -160, 133);
            delay(500);
            home();
          }
          if (digitalRead(irPin) == 1)
          {
            Position(positionX[i], positionY[i], positionZ[i]);
            digitalWrite(relayPin1, HIGH);
            delay(2500);
            home();
            if (Serial.available() > 0)
            {
              receivedChar = Serial.readString();
              i = receivedChar.toInt();
            }
            i = 10; 
          }
        }
    }
  }
  else
  {
    digitalWrite(relayPin, LOW);  
}
