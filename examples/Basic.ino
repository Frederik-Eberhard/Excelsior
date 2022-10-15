#include <Excelsior.h>
Excelsior E;

void setup()
{
  E.SensorSetup(1,TAST_NXT);      //Initialisierung des Tastsensors auf SensorPort 1
  E.SensorSetup(2,LICHT);         //Initialisierung des lichtsensors auf SensorPort 2
}

void loop()
{
  if(E.SensorWert(1) == true)     //Wenn der Tastsensors auf Sensorport 1 gedrückt ist:
  {
    E.Motor(MOTOR_A,0);   	      //stoppen die beiden Motoren A und B
    E.Motor(MOTOR_B,0);
    delay(2000);                  //für 2 Sekunden
  }
  else
  {
    E.Motor(MOTOR_A,100);        //Sonst dreht sich der Roboter langsam um die eigene Achse --> A dreht vorwärts, B dreht rückwärts
    E.Motor(MOTOR_B,-100);
    delay(10);
  }
  if(E.GyroWert(GYRO_X) > 90)    //Wenn sich der Roboter um 90° von der Ausgangsposition gedreht hat
  {
    while(E.GyroWert(GYRO_X) > -90)   //Dreht er sich schnell um 180° in die andere Richtung
    {
      E.Motor(MOTOR_A,255);
      E.Motor(MOTOR_B,-255);
      E.DisplayAktualisieren(1);      //Das Display zeigt dabei die Motor- und Gyroskopwerte an
    }
  }

  E.DisplayAktualisieren(1);
}
