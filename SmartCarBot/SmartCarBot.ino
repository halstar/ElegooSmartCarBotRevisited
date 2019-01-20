#include <Servo.h>  

#include "IRremote.h"

#define DEBUG

#ifdef DEBUG
    #define LOG(string)       Serial.print(string)
    #define LOG_LINE(string)  Serial.println(string)
#else
    #define LOG(X)       
    #define LOG_LINE(X)  
#endif

#define BTN_FORWARD  16736925
#define BTN_BACKWARD 16754775
#define BTN_LEFT     16720605
#define BTN_RIGHT    16761405
#define BTN_STOP     16712445
#define BTN_1        16738455 // Line tracking mode
#define BTN_2        16750695 // Obstacles avoidance mode
#define BTN_3        16756815
#define BTN_4        16724175
#define BTN_5        16718055
#define BTN_6        16743045
#define BTN_7        16716015
#define BTN_8        16726215
#define BTN_9        16734885
#define BTN_0        16730805
#define BTN_STAR     16728765
#define BTN_HASH     16732845

#define RECV_PIN 12
#define LED_PIN  13
#define ECHO_PIN A4  
#define TRIG_PIN A5 

#define LEFT_MOTORS_VCC 11
#define LEFT_MOTORS_GND  9

#define RIGHT_MOTORS_VCC 7
#define RIGHT_MOTORS_GND 8

#define LEFT_MOTORS_POWER  5
#define RIGHT_MOTORS_POWER 6

#define LineTeacking_Pin_Right  10
#define LineTeacking_Pin_Middle 4
#define LineTeacking_Pin_Left   2
#define LineTeacking_Read_Right   !digitalRead(10)
#define LineTeacking_Read_Middle  !digitalRead(4)
#define LineTeacking_Read_Left    !digitalRead(2)

#define DEFAULT_CAR_SPEED 250

Servo         servo;
IRrecv        irRecv(RECV_PIN);
unsigned long irPreMillis;
unsigned long ltPreMillis;

enum MODE
{
  IDLE,
  LINE_TRACKING,
  OBSTACLES_AVOIDANCE,
  BT_CONTROL,
  IR_CONTROL
} mainMode = IDLE, oldMainMode = IDLE;

enum DIRECTION
{
  STOP,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT
} direction = STOP, oldDirection = STOP;

void delays(unsigned long t)
{
  for(unsigned long i = 0; i < t; i++)
  {
    getBtData();
    getIrData();
    delay(1);
  }
}

int getDistance()
{
  int distance;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
 
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
 
  digitalWrite(TRIG_PIN, LOW);
 
  distance = (int)pulseIn(ECHO_PIN, HIGH) / 58;

  LOG("Distance: ");
  LOG(distance);

  return distance;
}

void goForward(void)
{ 
  analogWrite(LEFT_MOTORS_POWER , DEFAULT_CAR_SPEED);
  analogWrite(RIGHT_MOTORS_POWER, DEFAULT_CAR_SPEED);

  digitalWrite(LEFT_MOTORS_VCC, HIGH);
  digitalWrite(LEFT_MOTORS_GND, LOW );

  digitalWrite(RIGHT_MOTORS_VCC, HIGH);
  digitalWrite(RIGHT_MOTORS_GND, LOW);

  LOG_LINE("Going forward");
}

void goBackward(void)
{
  analogWrite(LEFT_MOTORS_POWER , DEFAULT_CAR_SPEED);
  analogWrite(RIGHT_MOTORS_POWER, DEFAULT_CAR_SPEED);

  digitalWrite(LEFT_MOTORS_VCC, LOW );
  digitalWrite(LEFT_MOTORS_GND, HIGH);

  digitalWrite(RIGHT_MOTORS_VCC, LOW );
  digitalWrite(RIGHT_MOTORS_GND, HIGH);

  LOG_LINE("Going backward");
}

void turnLeft(void)
{
  analogWrite(LEFT_MOTORS_POWER, DEFAULT_CAR_SPEED);
  analogWrite(RIGHT_MOTORS_POWER, DEFAULT_CAR_SPEED);

  digitalWrite(LEFT_MOTORS_VCC, HIGH); 
  digitalWrite(LEFT_MOTORS_GND, LOW );

  digitalWrite(RIGHT_MOTORS_VCC, LOW );
  digitalWrite(RIGHT_MOTORS_GND, HIGH);
 
  LOG_LINE("Turning left");
}

void turnRight(void)
{
  analogWrite(LEFT_MOTORS_POWER, DEFAULT_CAR_SPEED);
  analogWrite(RIGHT_MOTORS_POWER, DEFAULT_CAR_SPEED);

  digitalWrite(LEFT_MOTORS_VCC, LOW );
  digitalWrite(LEFT_MOTORS_GND, HIGH);

  digitalWrite(RIGHT_MOTORS_VCC, HIGH);
  digitalWrite(RIGHT_MOTORS_GND, LOW );
 
  LOG_LINE("Turning right");
}

void stop(void)
{
  digitalWrite(LEFT_MOTORS_POWER , LOW);
  digitalWrite(RIGHT_MOTORS_POWER, LOW);
 
  LOG_LINE("Stop");
}

void getBtData()
{
  if (Serial.available() == true)
  {
    switch (Serial.read())
    {
      case 'f':
        mainMode  = BT_CONTROL;
        direction = FORWARD;
        break;
      case 'b':
        mainMode  = BT_CONTROL;
        direction = BACKWARD;     
        break;
      case 'l': 
        mainMode  = BT_CONTROL; 
        direction = TURN_LEFT;     
        break;
      case 'r': 
        mainMode  = BT_CONTROL; 
        direction = TURN_RIGHT;
        break;
      case 's':
        mainMode  = BT_CONTROL;
        direction = STOP;
        break;
      case '1':
        mainMode = LINE_TRACKING;
        break;
      case '2':
        mainMode = OBSTACLES_AVOIDANCE;
        break;
      default:
        break;
    } 
  }
}

void getIrData(void)
{
  decode_results results;

  if (irRecv.decode(&results) == true)
  { 
    irPreMillis = millis();

    switch (results.value)
    {
      case BTN_FORWARD:
        mainMode  = IR_CONTROL;
        direction = FORWARD; 
        break;
      case BTN_BACKWARD:
        mainMode  = IR_CONTROL;
        direction = BACKWARD;     
        break;
      case BTN_LEFT:
        mainMode  = IR_CONTROL;
        direction = TURN_LEFT;     
        break;
      case BTN_RIGHT:   
        mainMode  = IR_CONTROL;
        direction = TURN_RIGHT;   
        break;
      case BTN_STOP:
        mainMode  = IR_CONTROL;
        direction = STOP;    
        break;
      case BTN_1:  
        mainMode = LINE_TRACKING;                
        break;
      case BTN_2:
        mainMode = OBSTACLES_AVOIDANCE;          
        break;
      default:
        break;
    }
    irRecv.resume();
  }
}

void doBtControl() 
{
  if (mainMode == BT_CONTROL)
  {
    switch (direction)
    {
      case FORWARD:
        goForward();
        break;
      case BACKWARD:
        goBackward();
        break;
      case TURN_LEFT:
        turnLeft();
        break;
      case TURN_RIGHT:
        turnRight();
        break;
      case STOP:
        stop();
        break;
      default:
        break;
    }
  }
}

void doIrControl() 
{
  if (mainMode == IR_CONTROL)
  {
    switch (direction)
    {
      case FORWARD:
        goForward();
        break;
      case BACKWARD:
        goBackward();
        break;
      case TURN_LEFT:
        turnLeft();
        break;
      case TURN_RIGHT:
        turnRight();
        break;
      case STOP:
        stop();
        break;
      default:
        break;
    }
    if (millis() - irPreMillis > 500)
    {
      direction   = STOP;
      irPreMillis = millis();
    }
  }   
}

void doLineTracking() 
{
  if (mainMode == LINE_TRACKING)
  {
    if (LineTeacking_Read_Middle)
    {
      goForward();
      ltPreMillis = millis();
    }
    else if (LineTeacking_Read_Right)
    { 
      turnRight();
      while (LineTeacking_Read_Right)
      {
        getBtData();
        getIrData();
      }
      ltPreMillis = millis();
    }
    else if (LineTeacking_Read_Left)
    {
      turnLeft();
      while(LineTeacking_Read_Left)
      {
        getBtData();
        getIrData();
      }
      ltPreMillis = millis();
    }
    else
    {
      if (millis() - ltPreMillis > 150)
      {
        stop();
      }
    }
  }  
}

void doObstaclesAvoidance()
{
  int rightDistance  = 0;
  int leftDistance   = 0;
  int middleDistance = 0;
  
  if (mainMode == OBSTACLES_AVOIDANCE)
  {
    servo.write(90);
    delays(500);
    middleDistance = getDistance();
    if (middleDistance <= 40)
    {
      stop();
      delays(500);
      servo.write(10);
      delays(1000);
      rightDistance = getDistance();
      
      delays(500);
      servo.write(90);
      delays(1000);
      servo.write(170);
      delays(1000); 
      leftDistance = getDistance();
      
      delays(500);
      servo.write(90);
      delays(1000);
      if (rightDistance > leftDistance)
      {
        turnRight();
        delays(360);
      }
      else if (rightDistance < leftDistance)
      {
        turnLeft();
        delays(360);
      }
      else if ((rightDistance <= 40) || (leftDistance <= 40))
      {
        goBackward();
        delays(180);
      }
      else
      {
        goForward();
      }
    }
    else
    {
        goForward();
    }  
  }  
}

void setup()
{
  Serial.begin(9600);
  servo.attach(3, 500, 2400);// 500: 0 degree  2400: 180 degree
  servo.write(90);
  irRecv.enableIRIn();
  
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  pinMode(LEFT_MOTORS_VCC, OUTPUT);
  pinMode(LEFT_MOTORS_GND, OUTPUT);

  pinMode(RIGHT_MOTORS_VCC, OUTPUT);
  pinMode(RIGHT_MOTORS_GND, OUTPUT);

  pinMode(LEFT_MOTORS_POWER , OUTPUT);
  pinMode(RIGHT_MOTORS_POWER, OUTPUT);

  pinMode(LineTeacking_Pin_Right , INPUT);
  pinMode(LineTeacking_Pin_Middle, INPUT);
  pinMode(LineTeacking_Pin_Left  , INPUT);
}

void loop()
{
  getBtData();
  getIrData();

  if (mainMode != oldMainMode)
  {
    LOG("Mode: ");

    switch (mainMode)
    {
      case IDLE:
        LOG_LINE("IDLE");
        break;
      case LINE_TRACKING:
        LOG_LINE("LINE_TRACKING");
        break;
      case OBSTACLES_AVOIDANCE:
        LOG_LINE("OBSTACLES_AVOIDANCE");
        break;
      case BT_CONTROL:
        LOG_LINE("BT_CONTROL");
        break;
      case IR_CONTROL:
        LOG_LINE("IR_CONTROL");
        break;
      default:
        break;
    }

    oldMainMode = mainMode;
  }

  if (direction != oldDirection)
  {
    LOG("Direction: ");

    switch (direction)
    {
      case STOP:
        LOG_LINE("STOP");
        break;
      case FORWARD:
        LOG_LINE("FORWARD");
        break;
      case BACKWARD:
        LOG_LINE("BACKWARD");
        break;
      case TURN_LEFT:
        LOG_LINE("TURN_LEFT");
        break;
      case TURN_RIGHT:
        LOG_LINE("TURN_RIGHT");
        break;
      default:
        break;
    }

    oldDirection = direction;
  }

  doBtControl();
  doIrControl();
  doLineTracking();
  doObstaclesAvoidance();
}
