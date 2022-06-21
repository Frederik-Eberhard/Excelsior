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
#define AUS          0
#define WEISS        1
#define ROT          2
#define GRUEN        3
#define BLAU         4
#define CYAN         5
#define MAGENTA      6
#define GELB         7
#define LICHT        8
#define LICHT_NXT    9
#define TAST_NXT    10
#define TAST_EV3    11
#define INFRAROT    12
#define MOTOR_A     13
#define MOTOR_B     14
#define MOTOR_C     15
#define MOTOR_D     16
#define GYRO_X      17
#define GYRO_Y      18
#define GYRO_Z      19

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
    int  GyroWert(int axis, bool autoreset);
    void GyroReset();
    void GyroReset(int axis);
    void GyroReset(int axis, bool toOriginal);
    void GyroVerzoegerung(int delay);
    void GyroResetSpann(int a, int b);
    void DisplayAktualisieren();
    void DA();
    void DA(int type);
    void DisplayAktualisieren(int type);
    void DA(int (&layout)[8]);
    void DisplayAktualisieren(int (&layout)[8]);
    void DA(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8);
    void DisplayAktualisieren(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8);
    void DT(int x_, int y_, String s_);
    void DisplayText(int x_, int y_, String s_);
    void DR();
    void DisplayRand();
    void Wait(unsigned int delay);


  private:
    int  _LightSensorValue(int port, int color);
    long _LightSensorPercent(int port, int color);

    //Teensy 4.1 Pinout

    const int _pinout[13][4] =  {{ 7, 6, 2}       //---Motors        (3x PWM)
                                ,{ 8, 9, 3}       //      |
                                ,{11,10, 4}       //      |
                                ,{29,28, 5}       //      |
                                ,{24,25,12}       //---Internal I2C  (SCL2,SDA2) + Button
                                ,{13,14,15,23}    //1---Sensors       (3x Digital, 1x Analog)  //Blue, Yellow ,Black ,White
                                ,{36,37,38,22}    //2      |
                                ,{33,34,35,21}    //3      |
                                ,{32,31,30,20}    //4      |
                                ,{52,54,53,41}    //5      |
                                ,{50,48,51,40}    //6      |
                                ,{49,26,27,39}    //7      |
                                ,{17,18,19,16}};  //8---Open I2C      (SDA1,SDA,SCL,SCL1)
    //Tennsy 3.2 Pinout
    /*
    const int _pinout[3][4]={{ 3, 4, 5},
                             {29,30,32},
                             {25,26,27,28}};
    */
    static const int _sensShift = 4;//1;                        //number needed to be added to port for mapping to the pinout
    static const int _maxSensors = 8;//1;
    static const int _maxMotors = 4;//1;
    static const int _DisplayX  = 10;
    static const int _DisplayY  =  4;
    int _lightDelay = 1;                                  //not realy neccessary to have a higher number, as even 1 millisecond doesnt reduce the quality of the brightnesvalue
    int _gyroresetDelay = 100;
    int _gyroCalls = 0;
    int _gyroSpan[2] = {10, 200};                         //a Span, where if gyroValues fall inside of it, they wont get reset by autoreset

    int _sensors[_maxSensors];                            //stores the type of a sensor and if it hasn't been initialized it will be -1
    int _sensorValues[_maxSensors + 7];                   //stores the values of all sensors, the used gyroscope values the gyroscope reset values and the button
    int _motorSpeeds[_maxMotors];                         //stores the speed / direction of each motor

    String _Display[_DisplayX][_DisplayY];                //stores what is supposed to be shown on the display
    bool _displayOutline = false;
};

template<typename type> type absolute(type v){
  if(v < 0)
    return -1 * v;
  return v;
}

template double absolute(double);
template int absolute(int);
template float absolute(float);
template long absolute(long);

#endif
