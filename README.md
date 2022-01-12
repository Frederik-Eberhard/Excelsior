# Excelsior
Arduino Library to enable easy use of the Excelsior-Brick thought as a replacement of the LEGO-EV3
### Version 1.0.1
- Added Infrared-Sensor compatability
- Screen now switches away from "Programm startet in 6 Sekunden" after the gyro finished calibrating
### Version 1.0.2
- DisplayAktuallisieren function is renamed to DisplayAktualisieren
### Version 1.0.3
- Added Adafruit_SSD1306 in dependencies
### Version 1.0.4
- Added function DisplayText for enabling custom Text to be displayed on the Display
- Added function DA as a shorthand for DisplayAktualisieren
### Version 1.0.5
- Removed unecessary "Excelsior::" from cpp file
### Version 1.0.6
- Added further features to Gyroscope:
 - Autoreset : enables that every fixed number of functioncalls all gyroValues are reset to 0. The fixed  number is stored in *gyroresetDelay*
 - GyroVerzoegerung(int) : sets the *gyroresetDelay* to the given parameter
 - GyroReset(int) : enables the resetting of gyroValues without enabling autoreset and also disabling the gyroOffset again
- Added features to Display:
 - Now possible to display the DisplayOutline with DisplayRand() or DR()
 - Default is still the button press if Knop() is called in main
- SensorValues is bigger, now storing displayed GyroValues and also the GyroOffset
