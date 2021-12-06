#ifndef Excelsior_h
#define Excelsior_h

#include <Arduino.h>
#include <Adafruit_GFX.h>  // Include core graphics library for the display
#include <Adafruit_SSD1306.h>  // Include Adafruit_SSD1306 library to drive the display
#include <MPU6050_tockn.h>
#include <Wire.h>

#include <Fonts/FreeMonoBold12pt7b.h>  // Add a custom font
#include <Fonts/FreeMono9pt7b.h>  // Add a custom font

//define what color is which number for the switch cases and also what variables are used here
#define AUS         0
#define WEISS       1
#define ROT         2
#define GRUEN       3
#define BLAU        4
#define CYAN        5
#define MAGENTA     6
#define GELB        7
#define LICHT       8
#define LICHT_NXT   9
#define TAST_NXT   10
#define TAST_EV3   11
#define INFRAROT   12
#define MOTOR_A     1
#define MOTOR_B     2
#define MOTOR_C     3
#define MOTOR_D     4
#define GYRO_X      0
#define GYRO_Y      1
#define GYRO_Z      2

class Excelsior
{
  public:
    Adafruit_SSD1306 display;
    MPU6050 mpu6050;
    Excelsior();
    void SensorSetup(int port, int type);
    void LichtVerzoegerung(int delay);
    void Motor(int port, int dir);
    bool Knopf();
    int  SensorWert(int port);
    int  SensorWert(int port, int color);
    int  SensorWert(int port, int color, bool percent);
    int  GyroWert(int axis);
    void DisplayAktualisieren();
    void DisplayAktualisieren(int type);

  private:
    int  _LightSensorValue(int port, int color);
    long _LightSensorPercent(int port, int color);

    //Teensy 4.1 Pinout

    const int _pinout[13][4] =  {{ 7, 6, 2}       //---Motors        (3x PWM)
                                ,{ 8, 9, 3}       //      |
                                ,{11,10, 4}       //      |
                                ,{29,28, 5}       //      |
                                ,{24,25,12}       //---Internal I2C  (SCL2,SDA2) + Button
                                ,{13,14,15,23}    //---Sensors       (3x Digital, 1x Analog)  //Blue, Yellow ,Black ,White
                                ,{36,37,38,22}    //      |
                                ,{33,34,35,21}    //      |
                                ,{32,31,30,20}    //      |
                                ,{49,50,48,41}    //      |
                                ,{52,54,53,40}    //      |
                                ,{51,26,27,39}    //      |
                                ,{17,18,19,16}};  //---Open I2C      (SDA1,SDA,SCL,SCL1)

    //Tennsy 3.2 Pinout
    /*
    const int _pinout[3][4]={{ 3, 4, 5},
                             {29,30,32},
                             {25,26,27,28}};
    */
    static const int _sensShift = 4;//1;                        //number needed to be added to port for mapping to the pinout
    static const int _maxSensors = 8;//1;
    static const int _maxMotors = 4;//1;
    int _lightDelay = 1;                                  //not realy neccessary to have a higher number, as even 1 millisecond doesnt reduce the quality of the brightnesvalue

    int _sensors[_maxSensors];
    int _sensorValues[_maxSensors + 4];                   //stores the values of all sensors, the gyroscope and the button
    int _motorSpeeds[_maxMotors];                         //stores the speed / direction of each motor

    String _Display[10][4];                               //stores what is supposed to be shown on the display
};

#endif
