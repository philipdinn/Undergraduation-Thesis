#include <AccelStepper.h>  // đk động cơ bước (vị trí, tốc độ,gia tốc)
#include <Wire.h>  // giao tiếp I2C
#include <LiquidCrystal_I2C.h>  // đk màn hình LCD qua I2C

#define E_X 9   // ct hành trình trục X _ chân 9
#define E_Y 10
#define E_Z 11

#define irPin A0  // chân cb hồng ngoại
#define relayPin 12  // relay đk băng chuyền
#define relayPin1 13   // realy đk hút

int StepX = 2; // khai báo chân
int DirX = 5; 

int StepY = 3; 
int DirY = 6; 

int StepZ = 4; 
int DirZ = 7; 

int ena = 8;  // Enable pin cho đc bước
unsigned long startTime;
int flag=0;
AccelStepper mystepperX(1, StepX, DirX, ena); //1 là chế độ dùng Driver, chân pin 2, chân pin 5, chân pin 8
AccelStepper mystepperY(1, StepY, DirY, ena); 
AccelStepper mystepperZ(1, StepZ, DirZ, ena); 

const int positionX[] = {-270, -200, -150, -270, -225, -185, -270, -230, -200}; // Các vị trí cần đến
const int positionY[] = {-100, -100, -150, -150, -150, -190, -210, -220, -250};
const int positionZ[] = {200, 200, 200, 150, 150, 150, 100, 100, 100};

void home_YZ()
{
  int homeY = 0;
  int homeZ = 0;

  mystepperY.setMaxSpeed(100);  // bước trên giây
  mystepperY.setAcceleration(100);  // bước trên giây bình phương
  mystepperY.enableOutputs();  //Nếu có chân enable (kích hoạt), nó cũng sẽ được thiết lập và kích hoạt để động cơ bước sẵn sàng hoạt động.

  mystepperZ.setMaxSpeed(100);
  mystepperZ.setAcceleration(100);
  mystepperZ.enableOutputs();

  while(digitalRead(E_Y)==1 && digitalRead(E_Z)==1)  // 1 là chưa chạm ct
  {
    mystepperY.moveTo(homeY);
    mystepperZ.moveTo(homeZ);
    homeY++;
    homeZ--;
    mystepperY.run();
    mystepperZ.run();
  }

  while(digitalRead(E_Y)==1)
  {
    mystepperY.moveTo(homeY);
    homeY++;
    mystepperY.run();
  }
  mystepperY.setCurrentPosition(0);

  while(digitalRead(E_Z)==1)
  {
    mystepperZ.moveTo(homeZ);
    homeZ--;
    mystepperZ.run();
  }
  mystepperZ.setCurrentPosition(0);
  
  homeY = 0; homeZ = 0;
}

void home_X()
{
  int homeX = 0;
  mystepperX.setMaxSpeed(100);
  mystepperX.setAcceleration(100);
  mystepperX.enableOutputs();

  while(digitalRead(E_X)==1)
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
  mystepperX.moveTo(X);  // vị trí đích đến là x
  mystepperY.moveTo(Y);
  mystepperZ.moveTo(Z);
  while(mystepperX.distanceToGo() != 0 || mystepperY.distanceToGo() != 0 || mystepperZ.distanceToGo() != 0)  // vòng lặp, nếu khoảng cách đến đích đến vẫn khác 0 thì tiếp tục di chuyển
  {
    mystepperX.run();
    mystepperY.run();
    mystepperZ.run();
  }
}

void Position_home(int Y, int Z)
{
  mystepperY.moveTo(Y);
  mystepperZ.moveTo(Z);
  while(mystepperY.distanceToGo() != 0 || mystepperZ.distanceToGo() != 0)
  {
    mystepperY.run();
    mystepperZ.run();
  }
}

void loop() 
{
  int i;
  if(digitalRead(irPin) == 0)                     // =0 là có vật cản
  {
    Serial.println(digitalRead(irPin));
    digitalWrite(relayPin, HIGH);                 //bc dừng
    if(Serial.available() > 0)                    // Kiểm tra xem có dữ liệu nào được gửi đến cổng Serial không.
      //Position(-50, -175, 128);
    {
      String receivedChar = Serial.readString();  // Đọc dữ liệu từ cổng Serial và lưu vào chuỗi receivedChar.
      i = receivedChar.toInt();                   // Chuyển đổi chuỗi receivedChar thành số nguyên và lưu vào biến i.
      while(i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 8)
      {
        digitalWrite(relayPin1, LOW);             // hút bật
        Serial.println(digitalRead(irPin));
        Position(-65, -163, 133);                 //vi tri tren bc
        delay(500);
        home();
        if(digitalRead(irPin) == 0)               // còn vat can, chua hut đc
        {
          Position(-60, -163, 133);
          delay(500);
          home();
        }
        if(digitalRead(irPin) == 0)
        {
          Position(-65, -160, 133);
          delay(500);
          home();
        }
        if(digitalRead(irPin) == 0)
        {
          Position(-60, -160, 133);
          delay(500);
          home();
        }
        //delay(500);
        if(digitalRead(irPin)==1)   
        {
        //Serial.println(digitalRead(irPin));
        //digitalWrite(relayPin, LOW);
        digitalWrite(relayPin1, HIGH);
        Position(positionX[i], positionY[i], positionZ[i]);
        delay(500);
      
        //delay(2500);
        home();
        if(Serial.available() > 0)
        {
          receivedChar = Serial.readString();
          i = receivedChar.toInt();
          i=9;
          
        }
        break;
        }
      }
      
    }
  }
  digitalWrite(relayPin, LOW);  // bc  chạy
}

