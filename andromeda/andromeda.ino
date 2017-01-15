#include <IRremote.h>
#include <NewPing.h>
#include <toneAC.h>

// ULTRASONIC SENSOR CONFIG
#define TRIGGER_PIN  7
#define ECHO_PIN     8
#define MAX_DISTANCE 200
#define SONAR_DISTANCE_TO_STOP    20
#define SONAR_DISTANCE_TO_GO      40
#define SONAR_MS_TURN_RIGHT       550
#define SONAR_MS_TURN_LEFT        500
#define SONAR_MS_CHECK_INTERVAL   50

// SERVO CONFIG
#define LSERVO_STOP 0 // 95
#define RSERVO_STOP 0 // 95
#define LSERVO_FORWARD 250
#define RSERVO_FORWARD 10
#define LSERVO_RIGHT 0
#define RSERVO_RIGHT 100
#define LSERVO_LEFT 0
#define RSERVO_LEFT 100
#define LSERVO_BACKWARD 1
#define RSERVO_BACKWARD 254

// PINS CONFIG
#define IR_PIN 11
#define BUTTON_PIN 2
#define LSERVO_PIN 6
#define RSERVO_PIN 5

// ROBOT MOVEMENTS
#define STOP        0
#define FORWARD     1
#define BACKWARD    2
#define TURN_RIGHT  3
#define TURN_LEFT   4

// BUTTON STATES
#define BTN_ON      0
#define BTN_OFF     1
#define BTN_CLICK   2

// DRIVE MODES
#define DRIVE_MENU            0
#define DRIVE_SONAR           1
#define DRIVE_REMOTE_ONEKEY   2
#define DRIVE_REMOTE_FULL     3
#define DRIVE_MODES           3

// IR REMOTE CODES
#define REMOTE_FORWARD    1033561079 //3838214000
#define REMOTE_BACKWARD   465573243 //4109411888
#define REMOTE_LEFT       2351064443 //2718175436
#define REMOTE_RIGHT      71952287 //274564916
#define REMOTE_STOP       1217346747 //4291178960


int robotMovements[][2]={
  {0, 0},     // Stop
  {254, 1},   // Forward
  {1, 254},   // Backward
  {254, 254}, // Turn Right
  {1, 1}      // Turn Left
};

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
IRrecv irrecv(IR_PIN);
decode_results results;

//void toneAC(int x, int y, int z) {delay(z);}

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LSERVO_PIN, OUTPUT);
  pinMode(RSERVO_PIN, OUTPUT);
  //moveRobot(FORWARD);
  //delay(250);
  //moveRobot(STOP);
  playBeverlyHillsCop();
  }

int buttonState=BTN_OFF;
int driveMode=DRIVE_MENU;
int menuOption=DRIVE_MENU;
int distance=0;
int distance_left=0;
int distance_right=0;
int ping_buffer[3]={MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};
int ping_buffer_index=0;
unsigned char robotCurrentMove=STOP;
unsigned long lastButtonClick=0;
unsigned long time_last_ping;
unsigned long time_ms;

void loop()
{

  // Obtain button status and capture click
  if (digitalRead(BUTTON_PIN)==LOW)
  {
    buttonState=BTN_OFF;
  }
  else if (buttonState==BTN_OFF)
  {
    buttonState=BTN_CLICK;
    Serial.println("CLICK");
  }
  else
    buttonState=BTN_ON;
  
  // Robot is in Main Menu
  if (driveMode==DRIVE_MENU)
  {
    if (buttonState==BTN_CLICK)
    {
      if (++menuOption>DRIVE_MODES) menuOption=DRIVE_MENU;
      // Play tones to know the option selected
      if (menuOption==DRIVE_MENU)
      {
        toneAC(250,10,1000);
        lastButtonClick=0;
        Serial.println("MENU PRINCIPAL");
      }
      else if (menuOption==DRIVE_SONAR)
      {
        Serial.println("MENU SONAR");
        toneAC(700,10,500);
        delay(250);
        lastButtonClick=millis();
      }
      else if (menuOption==DRIVE_REMOTE_ONEKEY)
      {
        Serial.println("MENU REMOTE ONEKEY");
        toneAC(700,10,500);
        delay(250);
        toneAC(700,10,500);
        delay(250);
        lastButtonClick=millis();
      }
      else
      {
        Serial.println("MENU REMOTE FULL");
        toneAC(700,10,500);
        delay(250);
        toneAC(700,10,500);
        delay(250);
        toneAC(700,10,500);
        delay(250);
        lastButtonClick=millis();
      }
      buttonState=BTN_OFF;
    }
    else if (lastButtonClick!=0)
    {
      if (millis()>lastButtonClick+2500)
      {
        Serial.println("MENU GO");
        toneAC(950,10,2000);
        lastButtonClick=0;
        driveMode=menuOption;
        if (driveMode==DRIVE_SONAR)
          moveRobot(FORWARD);
      }
    }
  }
  else // The robot is Moving
  {
    if (buttonState==BTN_CLICK)
    {
      driveMode=DRIVE_MENU;
      menuOption=DRIVE_MENU;
      toneAC(250,10,1000);
      moveRobot(STOP);
      lastButtonClick=0;
    }
    else
    {
      if (millis()>time_last_ping+SONAR_MS_CHECK_INTERVAL)
      {
        ping_buffer[ping_buffer_index++]=sonar.ping_cm();
        time_last_ping=millis();
        if (ping_buffer_index>2) ping_buffer_index=0;
        distance=(ping_buffer[0]+ping_buffer[1]+ping_buffer[2])/3;
      }
      
      if (driveMode==DRIVE_SONAR)
        driveModeSonar();
      else if (driveMode==DRIVE_REMOTE_ONEKEY)
        driveModeRemoteOneKey();
      else
        driveModeRemoteFull();
    }
  }
}

void driveModeSonar()
{
    if (distance<SONAR_DISTANCE_TO_STOP)
    {
      // Get right distance
      time_ms=millis();
      moveRobot(TURN_RIGHT);
      while (millis()<time_ms+SONAR_MS_TURN_RIGHT);
      moveRobot(STOP);
      distance_right=sonar.convert_cm(sonar.ping_median(5));
      delay(250);
      // Get left distance
      time_ms=millis();
      moveRobot(TURN_LEFT);
      while (millis()<time_ms+SONAR_MS_TURN_LEFT*2);
      moveRobot(STOP);
      distance_right=sonar.convert_cm(sonar.ping_median(5));
      delay(250);
      if (distance_left>=SONAR_DISTANCE_TO_GO || distance_right>=SONAR_DISTANCE_TO_GO)
      {
        if (distance_left<distance_right)
        {
          time_ms=millis();
          moveRobot(TURN_RIGHT);
          while (millis()<time_ms+SONAR_MS_TURN_RIGHT*2);
        }
      }
      else
      {
        time_ms=millis();
        moveRobot(TURN_LEFT);
        while (millis()<time_ms+SONAR_MS_TURN_LEFT);
      }
      moveRobot(FORWARD);
    }  
}

void driveModeRemoteOneKey()
{   
  if (irrecv.decode(&results)) 
  {
    moveRobot(TURN_LEFT);
    irrecv.resume(); 
    lastButtonClick=millis();
  }
  else if (millis()>lastButtonClick+250 && distance>=SONAR_DISTANCE_TO_STOP)
    moveRobot(FORWARD);  
  else if (millis()>lastButtonClick+250 && distance<SONAR_DISTANCE_TO_STOP)
    moveRobot(STOP);
}

void driveModeRemoteFull()
{
  
  if (irrecv.decode(&results)) 
  {    
    switch (results.value)
    {
      case REMOTE_FORWARD:
        moveRobot(FORWARD);
        break;
      case REMOTE_BACKWARD:
        moveRobot(BACKWARD);
        break;
      case REMOTE_LEFT:
        moveRobot(TURN_LEFT);
        break;
      case REMOTE_RIGHT:
        moveRobot(TURN_RIGHT);
        break;
      case REMOTE_STOP:
        moveRobot(STOP);
        break;  
    }
    
    irrecv.resume();
    lastButtonClick=millis();
  }  
}

void moveRobot(unsigned char direction)
{
  if (robotCurrentMove==direction)
    return;
    
  robotCurrentMove=direction;

  analogWrite(LSERVO_PIN,robotMovements[direction][0]);
  analogWrite(RSERVO_PIN,robotMovements[direction][1]);
}

void playBeverlyHillsCop() 
{
  toneAC(659,10,460); 
  //toneAC(784,10,340); toneAC(659,10,230); toneAC(659,10,110); toneAC(880,10,230); toneAC(659,10,230); toneAC(587,10,230); 
  //toneAC(659,10,460); toneAC(988,10,340); toneAC(659,10,230); toneAC(659,10,110); toneAC(1047, 10, 230); toneAC(988,10,230); toneAC(784,10,230); toneAC(659,10,230); toneAC(988,10,230); toneAC(1318,10,230); toneAC(659,10,110); toneAC(587,10,230); toneAC(587,10,110); toneAC(494,10,230); toneAC(740,10,230); toneAC(659,10,460);
  //toneAC(0); // Turn off toneAC, can also use noToneAC().
}
