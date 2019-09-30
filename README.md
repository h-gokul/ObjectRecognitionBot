A mono vision based Simultaneous Localization and Mapping Project.
#### Device Recommended : Raspberry Pi 3 Model B+
#### Software package requirements : 
- opencv 
- numpy 
- math 
- Rpi

# Repository Description:
Check the ReportFile for elaborate details about the work. 

## Test Run 
Contains a .mp4 video file with real time implementation of the work in it's earliest primitive stage.

##  Tests
Contains code for Unit level tests and a System-Integrated run time test. Units include :
- Speech to text conversion module
- Object (ball) detection in runtime.
- Target detection - (updated Ball detection)
- GPIO logic for motor driver control

## Main
Python notebook main.ipynb implements the integrated final system for on-device execution. 
- all function dependencies are contained in python script lib.py 
