# Excelsior
Arduino Library to enable easy use of the Excelsior-Brick.
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
  - GyroReset(int,bool) : enables the resetting of gyroValues without enabling autoreset and also disabling the gyroOffset again
- Added features to Display:
  - Now possible to display the DisplayOutline with DisplayRand() or DR()
  - Default is still the button press if Knopf() is called in main
### Version 1.0.7
- Bug Fixes
### Version 1.0.8
- Added more function to autoreset of the gyro:
  - gyroSpan now sets an interval in which the autoreset doesn't reset: defaul interval is [10,200]. Note that this deals with the absolute gyroValues
  - Added function GyroResetSpann(int, int) to manually set this interval
- Added function absolute that is not part of the Excelsior class, which returns the absolute Value of any int, double, float, long
### Version 1.0.9
- Bug Fixes
### Version 1.1.0
- Added Wait function that acts like a regular delay while making sure, that the MPU continues to update to work reliably
### Version 1.1.1
-  Bug Fixes
### Version 1.1.2
- Sensorport change of the Infrarot-Sensor
### Version 1.1.3
- Refurbished the way that the Matrix on the Display is generated
- Custom order of the sensors and motors now able to be displayed in the Matrix-View
- Small Bug-Fixes to the pinout 
### Version 1.1.4
- Preparation for the Switch to BNO055
### Version 1.1.5
- MPU 6050 no longer supported, full switch to BNO055
- Rotations are now no longer limited to a range from 0 - 360°. This removes unwanted jumps in orientation readings and enables a simpler approach to many use cases
- Autoreset is no longer an option, it can still be done manually
### Version 1.1.6
- Small Bug Fix
### Version 1.1.7
- Added a new way to display Error Messages
  - If an Error occurs, a Message appears on the Display saying what went wrong
  - After the Error Message appeared, a warning triangle stays in the top right corner of the Display as a Warning
- Removed function Wait() since no longer necessary with the new Gyro
