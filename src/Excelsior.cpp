#include <Arduino.h>
#include <Excelsior.h>
/*#include <Adafruit_GFX.h>  // Include core graphics library for the display
#include <Adafruit_SSD1306.h>  // Include Adafruit_SSD1306 library to drive the display
#include <MPU6050_tockn.h>
#include <Wire.h>

#include <Fonts/FreeMonoBold12pt7b.h>  // Add a custom font
#include <Fonts/FreeMono9pt7b.h>  // Add a custom font
*/

//------SETUP------------------
Excelsior::Excelsior() : display(128, 64, &Wire2), mpu6050(Wire2){
  Serial.begin(9600);
  delay(1000);
  Serial.println("Hello Excelsior");

  for(int i = 0; i < Excelsior::_maxSensors; i++){
    Excelsior::_sensors[i] = -1;                        //initiallises array as empty
  }

  pinMode(Excelsior::_pinout[Excelsior::_sensShift][2], INPUT);     //internal Button

  for(int i = 0; i < Excelsior::_maxMotors; i++){
    pinMode(Excelsior::_pinout[i][0],OUTPUT);  //functions as digital port   (direction)
    pinMode(Excelsior::_pinout[i][1],OUTPUT);  //functions as digital port   (direction)
    pinMode(Excelsior::_pinout[i][2],OUTPUT);  //functions as PWM port       (speed)
  }

  delay(100);  // This delay is needed to let the display to initialize
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize display with the I2C address of 0x3C
  display.clearDisplay();  // Clear the buffer
  display.setTextColor(WHITE);  // Set color of the text
  display.setRotation(0);  // Set orientation. Goes from 0, 1, 2 or 3
  display.setTextWrap(true);  // By default, long lines of text are set to automatically “wrap” back to the leftmost column.
                               // To override this behavior (so text will run off the right side of the display - useful for
                               // scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored
                               // with setTextWrap(true).
  display.dim(0);  //Set brightness (0 is maximun and 1 is a little dim)

  Wire2.begin();
  Excelsior::DisplayAktualisieren(-1);
  Excelsior::mpu6050.begin();
  Excelsior::mpu6050.calcGyroOffsets(true);
  Excelsior::DisplayAktualisieren(0);
}

//------SENSOR SETUP------------------
void Excelsior::SensorSetup(int port, int type){           //Digital- / Analog-Ports
  if(type >= LICHT && type <= INFRAROT && port > 0 && port <= Excelsior::_maxSensors){
      Excelsior::_sensors[port - 1] = type;                                                                                            //LIGHT       LIGHT_NXT       TOUCH_NXT       TOUCH_EV3      INFRAROT      KABLECOLOR  (GREEN -> 5V | RED -> GND)
      pinMode(Excelsior::_pinout[Excelsior::_sensShift + port][0],(type == TAST_EV3 || type == INFRAROT)? INPUT:OUTPUT);               //Red         NULL            NULL            Signal         Signal        BLUE
      pinMode(Excelsior::_pinout[Excelsior::_sensShift + port][1],OUTPUT);                                                             //Green       Led             NULL            NULL                         YELLOW
      pinMode(Excelsior::_pinout[Excelsior::_sensShift + port][2],OUTPUT);                                                             //Blue        GND             GND             NULL                         BLACK
      pinMode(Excelsior::_pinout[Excelsior::_sensShift + port][3],INPUT_PULLUP);                                                       //Signal      Signal          Signal          NULL                         WEIß

      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0],LOW);         //if defined as INPUT, this disables the internal pullup resistor
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1],LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2],LOW);
  }else{
    Serial.println((String) "Sensorart " + type + " oder Anschluss " + port + " ist nicht definiert");
  }
}

void Excelsior::LichtVerzoegerung(int delay){
  Excelsior::_lightDelay = delay;
}
//------DRIVING MOTORS------
void Excelsior::Motor(int port, int dir){
  if(port > 0 && port <= Excelsior::_maxMotors && dir > -256 && dir < 256){
  Excelsior::_motorSpeeds[port - 1] = dir;
  digitalWrite(Excelsior::_pinout[port - 1][0], dir < 0? HIGH:LOW);   //if dir == 0, then both go LOW (motor off)
  digitalWrite(Excelsior::_pinout[port - 1][1], dir > 0? HIGH:LOW);   //else if dir determines direction of rotation
  analogWrite (Excelsior::_pinout[port - 1][2], abs(dir));            //takes the absolute value to determine rotation speed
  }else{
    Serial.println((String)"Der Motoranschluss " + port + " oder die Geschwindigkeit " + dir + " ist außerhalb des vorgegebenen Intervalls");
  }
}

//------READING SENSORS------
bool Excelsior::Knopf(){
  Excelsior::_sensorValues[Excelsior::_maxSensors + 3] = !digitalRead(Excelsior::_pinout[Excelsior::_sensShift][2]);
  return Excelsior::_sensorValues[Excelsior::_maxSensors + 3];
}

int Excelsior::SensorWert(int port){
  if(port > 0 && port <= Excelsior::_maxSensors){       //looks if the desired port is part of the possible ports
    if(Excelsior::_sensors[port - 1] == TAST_EV3){
      Excelsior::_sensorValues[port - 1] =  map(digitalRead(Excelsior::_pinout[Excelsior::_sensShift + port][0]),0,2,1,0);       //0 and 1 have to be flipped because of different sensor funciton compared to the TOUCH_NXT
      return Excelsior::_sensorValues[port - 1];
    }else if(Excelsior::_sensors[port - 1] == TAST_NXT){
      Excelsior::_sensorValues[port - 1] = !digitalRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]);
      return Excelsior::_sensorValues[port - 1];
    }else if(Excelsior::_sensors[port - 1] == INFRAROT){
      int pulse = pulseIn(Excelsior::_pinout[Excelsior::_sensShift + port][0], HIGH, 20000);                                    //timeout in microseconds
      Excelsior::_sensorValues[port - 1] = min(2000, max(0, 2 * (pulse - 1000)));                                               //2000 mm is the maximum that will be returned, anything lower will be calculated
      return Excelsior::_sensorValues[port - 1];
    }
    return SensorWert(port, AUS);
  }else{
    Serial.println((String)"Der Sensoranschluss " + port + " ist außerhalb des vorgegebenen Intervalls");
    return -1;
  }
}

int Excelsior::SensorWert(int port, int color){
  if(port > 0 && port <= Excelsior::_maxSensors){
    if(Excelsior::_sensors[port - 1] == LICHT_NXT){
      if(color)
        digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], HIGH);
      else
        digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], LOW);
      Excelsior::_sensorValues[port - 1] = map(analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]),0,1024,1024,0);    //sensorrange gets flipped so that low values correspond to black
      return Excelsior::_sensorValues[port - 1];
    }
    return SensorWert(port, color, false);
  }else{
    Serial.println((String)"Der Sensoranschluss " + port + " ist außerhalb des vorgegebenen Intervalls");
    return -1;
  }
}

int Excelsior::SensorWert(int port, int color, bool percent){
  if(port > 0 && port <= Excelsior::_maxSensors){
    if(Excelsior::_sensors[port - 1] == LICHT){
      Excelsior::_sensorValues[port - 1] = percent? Excelsior::_LightSensorPercent(port,color) : Excelsior::_LightSensorValue(port,color);
      return Excelsior::_sensorValues[port - 1];
    }else if(Excelsior::_sensors[port - 1] == LICHT_NXT){
      Excelsior::_sensorValues[port - 1] = percent? map(Excelsior::SensorWert(port,color),0,1024,0,100) : Excelsior::SensorWert(port,color);
      return Excelsior::_sensorValues[port - 1];
    }
    Serial.println((String) "Sensorart " + port + " ist nicht definiert oder kann die Parameter nicht erfüllen");
    return -1;
  }else{
    Serial.println((String)"Der Sensoranschluss " + port + " ist außerhalb des vorgegebenen Intervalls");
    return -1;
  }
}

int Excelsior::_LightSensorValue(int port, int color){             //gets the "raw" sensor-value
  switch(color){                                       //defines what color the sensor glows
    case WEISS:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], HIGH);
      delay(_lightDelay);                               //small delay to make sure the colors have changed
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));       //subtracts sensor-value from the maximum value returned by analogRead + 1 --> before: WHITE(0 - 1023)BLACK ; after: WHITE(1024 - 1)BLACK
    case ROT:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));
    case GRUEN:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));
    case BLAU:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));
    case CYAN:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));
    case MAGENTA:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], HIGH);
      delay(_lightDelay);
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));
    case GELB:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], HIGH);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));
    case AUS:
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][0], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][1], LOW);
      digitalWrite(Excelsior::_pinout[Excelsior::_sensShift + port][2], LOW);
      delay(_lightDelay);
      return (1024 - analogRead(Excelsior::_pinout[Excelsior::_sensShift + port][3]));
    default: Serial.println((String) "Farbe " + color + " ist nicht definiert");   return -1;
  }
}

long Excelsior::_LightSensorPercent(int port, int color){
  int _red,_green,_blue,_cyan,_magenta,_yellow;             // "__" to avoid possible ROTefinitions in the main programm
  switch(color){
    case AUS:
      return map(Excelsior::_LightSensorValue(port,AUS),0,1024,0,100);
    case WEISS:
      return map(Excelsior::_LightSensorValue(port,WEISS),0,1024,0,100);
    case ROT:
      _red   = Excelsior::_LightSensorValue(port,ROT);
      _green = Excelsior::_LightSensorValue(port,GRUEN);
      _blue  = Excelsior::_LightSensorValue(port,BLAU);
      Excelsior::_LightSensorValue(port,AUS);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _red * 100L / (_red + _green + _blue);         //the L defines the output as the datatype long, allowing bigger values
    case GRUEN:
      _red   = Excelsior::_LightSensorValue(port,ROT);
      _green = Excelsior::_LightSensorValue(port,GRUEN);
      _blue  = Excelsior::_LightSensorValue(port,BLAU);
      Excelsior::_LightSensorValue(port,AUS);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _green * 100L / (_red + _green + _blue);
    case BLAU:
      _red   = Excelsior::_LightSensorValue(port,ROT);
      _green = Excelsior::_LightSensorValue(port,GRUEN);
      _blue  = Excelsior::_LightSensorValue(port,BLAU);
      Excelsior::_LightSensorValue(port,AUS);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _blue * 100L / (_red + _green + _blue);
    case CYAN:
      _cyan     = Excelsior::_LightSensorValue(port,CYAN);
      _magenta  = Excelsior::_LightSensorValue(port,MAGENTA);
      _yellow   = Excelsior::_LightSensorValue(port,GELB);
      Excelsior::_LightSensorValue(port,AUS);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _cyan * 100L / (_cyan + _magenta + _yellow);
    case MAGENTA:
      _cyan     = Excelsior::_LightSensorValue(port,CYAN);
      _magenta  = Excelsior::_LightSensorValue(port,MAGENTA);
      _yellow   = Excelsior::_LightSensorValue(port,GELB);
      Excelsior::_LightSensorValue(port,AUS);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _magenta * 100L / (_cyan + _magenta + _yellow);
    case GELB:
      _cyan     = Excelsior::_LightSensorValue(port,CYAN);
      _magenta  = Excelsior::_LightSensorValue(port,MAGENTA);
      _yellow   = Excelsior::_LightSensorValue(port,GELB);
      Excelsior::_LightSensorValue(port,AUS);                 //turns of the light, so that it doesn't interfere with neighbouring sensors
      return   _yellow * 100L / (_cyan + _magenta + _yellow);
    default: Serial.println((String) "Farbe " + color + " ist nicht definiert");   return -1;
  }
}

//------MPU 6050------
int Excelsior::GyroWert(int axis){
  Excelsior::mpu6050.update();
  Excelsior::_sensorValues[Excelsior::_maxSensors + 0] = Excelsior::mpu6050.getAngleX();
  Excelsior::_sensorValues[Excelsior::_maxSensors + 1] = Excelsior::mpu6050.getAngleY();
  Excelsior::_sensorValues[Excelsior::_maxSensors + 2] = Excelsior::mpu6050.getAngleZ();
  if(axis >= GYRO_X && axis <= GYRO_Z)                             //looks if axis is between X and Z
    return Excelsior::_sensorValues[Excelsior::_maxSensors + axis];
  Serial.println((String)"Die Gyroskopachse " + axis + " ist nicht definiert");
  return -1;
}

//------OLED DISPLAY------------------
void Excelsior::DisplayAktualisieren(){
  Excelsior::DisplayAktualisieren(0);
}

void Excelsior::DisplayAktualisieren(int type){
  Excelsior::display.clearDisplay();  // Clear the display so we can refresh
  Excelsior::display.setFont(&FreeMono9pt7b);  // Set a custom font

  if(      type == -1){                                          //bootup display
    Excelsior::display.setTextSize(1);
    Excelsior::display.setCursor(0,10);
    Excelsior::display.println("  Programm \n startet in \n 6 Sekunden");

  }else if(type == 0 || type == 1){                              //standard display (shows all 8 Sensors || shows all 4 Motors and the gyroscope values)
    for(int i = 0; i < 8; i++){                                  //shows all sensorvalues -> if less then 8 ports, then the last three are the gyroscope values
      Excelsior::display.setTextSize(0);
      Excelsior::display.drawRoundRect((i < 4)? 0:65,  0 + 16*(i % 4), 13, 15, 1, WHITE);
      Excelsior::display.setCursor((i < 4)? 1:66, 12 + 16*(i % 4));
      if(!type){                                                 //if it shows the sensors
        Excelsior::display.println(i+1);
      }else{                                                     //if it shows the motors
        if(i < Excelsior::_maxMotors + 3){
          char c = i < Excelsior::_maxMotors? 'A' + i:'X' + i - Excelsior::_maxMotors;
          Excelsior::display.println(c);
        }
      }
      Excelsior::display.setCursor((i < 4)? 17:82, 12 + 16*(i % 4));
      if(!type){                                                  //if it shows the sensors
        if(i < Excelsior::_maxSensors + 3 && ((i < Excelsior::_maxSensors)? Excelsior::_sensors[i] != -1:true)){        //if there is no sensor connected or if i is bigger than maxSensors + gyroscope, nothing will be displayed
          Excelsior::display.println(Excelsior::_sensorValues[i]);
        }
      }else{                                                      //if it shows the motors
        if(i < Excelsior::_maxMotors + 3)
          Excelsior::display.println(i < Excelsior::_maxMotors? Excelsior::_motorSpeeds[i]:Excelsior::_sensorValues[Excelsior::_maxSensors + i - Excelsior::_maxMotors]);
      }
    }
    if(Excelsior::_sensorValues[Excelsior::_maxSensors+3]){                               //displays a outline to show if the button is pressed
      Excelsior::display.drawRect(0, 0, 128, 64, WHITE);
    }
  }
  Excelsior::display.display();  // Print everything we set previously
}
