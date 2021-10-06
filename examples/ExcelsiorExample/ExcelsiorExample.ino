#include <Excelsior.h>
Excelsior E;

void setup() 
{
  E.SensorSetup(1,TAST_NXT);
  E.SensorSetup(2,LICHT);
}

void loop() 
{
  if(E.SensorWert(1) == true){
    E.Motor(MOTOR_A,255);
    E.Motor(MOTOR_B,255);  
    delay(2000);
  }
  else{
    E.Motor(MOTOR_A,255);
    E.Motor(MOTOR_B,-255);  
    delay(10);
  }
  Serial.print("Lichtsensor: ");
  Serial.print(E.SensorWert(2,GRUEN,true));
  Serial.print("  Gyroskopsensor: ");
  Serial.println(E.GyroWert(GYRO_X));
  E.DisplayAktuallisieren();
}
