#include <Excelsior.h>
Excelsior E;

void setup()
{
  E.SensorSetup(1,TAST_NXT);      //Initialisierung des Tastsensors auf SensorPort 1
  E.SensorSetup(2,LICHT);         //Initialisierung des lichtsensors auf SensorPort 2
}

void loop()
{
  E.Motor(MOTOR_A, 255);
  E.Motor(MOTOR_B, 255);
  E.Motor(MOTOR_C, 10);
  if(E.SensorWert(1) == false)
  {
    E.SensorWert(2);          //Wird aufgerufen, damit die Werte
    E.GyroWert(GYRO_X);       //auf dem Display auch aktualisiert werden
    E.DisplayAktualisieren(1, MOTOR_A, MOTOR_B, MOTOR_C, 2, GYRO_X, GYRO_Y, GYRO_Z);     //Zeigt eine selbst definierte Ansicht an, bei der auf den 8 Stellen des Displays
                                                                                         //nun die Geschwindigkeiten von Motor A - C, die drei Gyroskopwerte
                                                                                         //und die Sensorwerte von Sensorport 1 und 2 angezeigt
  }
  else
  {
    E.DisplayText(1,1,"Tastsensor \n gedrueckt!");       //Zeigt anstelle der Übersicht benutzerdefinierten Text an. Die beiden Ganzzahlen geben die Startposition des Texts
    E.DisplayAktualisieren(2);                           //auf dem Display an. Das "\n" erzeugt einen Zeilenumbruch. Mit dem Parameter 2 wird der Text auf dem Display angezeigt
  }
  delay(100);
}
