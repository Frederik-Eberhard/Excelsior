#ifndef Excelsior_h
#define Excelsior_h

#include <Arduino.h>
#include <Array.h>              // Used for dynamic length array, as in the error messages
#include <Adafruit_GFX.h>       // Include core graphics library for the display
#include <Adafruit_SSD1306.h>   // Include Adafruit_SSD1306 library to drive the display
#include <Adafruit_Sensor.h>  	// Needed for the Adafruit_BNO055 Gyrosensor
#include <Adafruit_BNO055.h>    // Needed for the Adafruit_BNO055 Gyrosensor
#include <utility/imumaths.h>
#include <Wire.h>

#include <Fonts/FreeMonoBold9pt7b.h>  // Add a custom font -> "!" from error-Triangle
#include <Fonts/FreeMono9pt7b.h>      // Add a custom font -> other text

using namespace std;

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
#define MOTOR_A     1
#define MOTOR_B     2
#define MOTOR_C     3
#define MOTOR_D     4
#define GYRO_X      17
#define GYRO_Y      18
#define GYRO_Z      19

typedef Array<int,10> _VecInt10;                      //datatype similar to c++ vector with a maximum of 10 indecies

class Excelsior
{
  public:
    Adafruit_SSD1306 display;
    Adafruit_BNO055 bno055;
    Excelsior();
    void SensorSetup(int port, int type);
    void LichtVerzoegerung(int delay);
    void Motor(int port, int dir);
    bool Knopf();
    int  SensorWert(int port);
    int  SensorWert(int port, int color);
    int  SensorWert(int port, int color, bool percent);
    int  GyroWert(int axis);
    void GyroReset();
    void GyroReset(int axis);
    void GyroReset(int axis, bool toOriginal);
    void DisplayAktualisieren();
    void DA();
    void DA(int type);
    void DisplayAktualisieren(int type);
    void DA(int (&layout)[8], String errorMessage);
    void DisplayAktualisieren(int (&layout)[8], String errorMessage);
    void DA(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8);
    void DisplayAktualisieren(int layout1, int layout2, int layout3, int layout4, int layout5, int layout6, int layout7, int layout8);
    void DT(int x_, int y_, String s_);
    void DisplayText(int x_, int y_, String s_);
    void DR();
    void DisplayRand();

  private:
    int  _LightSensorValue(int port, int color);
    long _LightSensorPercent(int port, int color);
    void _getOrientation(double *vec);
    void _DisplayError(int error);
    void _DisplayError(int error, int input);
    void _DisplayError(int error, _VecInt10 & variables);
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
                                ,{48,50,49,41}    //5      |
                                ,{53,54,52,40}    //6      |
                                ,{27,26,51,39}    //7      |
                                ,{17,18,19,16}};  //8---Open I2C      (SDA1,SDA,SCL,SCL1)

    static const int _sensShift = 4;                       //number needed to be added to port for mapping to the pinout
    static const int _maxSensors = 8;
    static const int _maxMotors = 4;
    static const int _DisplayX  = 10;
    static const int _DisplayY  =  4;

    int _lightDelay = 1;                                  //not realy neccessary to have a higher number, as even 1 millisecond doesnt reduce the quality of the brightnesvalue

    int _sensors[_maxSensors];                            //stores the type of a sensor and if it hasn't been initialized it will be -1
    int _sensorValues[_maxSensors + 7];                   //stores the values of all sensors, the used gyroscope values the gyroscope offset values and the button
    int _motorSpeeds[_maxMotors];                         //stores the speed / direction of each motor

    String _Display[_DisplayX][_DisplayY];                //stores what is supposed to be shown on the display
    bool _displayOutline = false;                         //stores if the display-Outline is supposed to be displayed
    bool _errorTriangle = false;                          //stores if the error-Triangle is supposed to be displayed
    _VecInt10 _errorVariables;

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
