# Object Detection Bot
A mono vision based terrestrial bot designed for deployment of ORB SLAM algorithm, courtesy of Gautham Ponnu & Jacob George who implemented this project for their MEngg Design Project @ Cornell. Find their project report [here](https://courses.ece.cornell.edu/ece6930/ECE6930_Spring16_Final_MEng_Reports/SLAM/Real-time%20ROSberryPi%20SLAM%20Robot.pdf).

#### Device Recommended : Raspberry Pi 3 Model B+
#### Software package requirements : 
- OpenCV 
- numpy 
- math 
- Rpi

It is recommended to follow [valarsundarlis/ORB-SLAM](https://github.com/valarsundarlis/ORB-SLAM) implementation to install the dependancies and required packages for the test run using Monocular examples.

## Repository Description:
Check the ReportFile.pdf for elaborate details about the research study on sensors and  work. 

### Test Run 
Contains a .mp4 video file with real time implementation of the work in it's earliest primitive stage.

###  Tests
Contains code for Unit level tests and a System-Integrated run time test. Units include :
- Speech to text conversion module
- Object (ball) detection in runtime.
- Target detection - (updated Ball detection)
- GPIO logic for motor driver control

### Main
Python notebook main.ipynb implements the integrated final system for on-device execution. 
- all function dependencies are contained in python script lib.py 

# Project Extension implementing ORB SLAM algorithm.
  The hardware system design of this project was targeted to implement the monocular ORB SLAM algorithm. 
## ORB SLAM- Description. 
  This is a feature based simultaneous localization and mapping system that can be implemented on a real-time monocular differential drive bot. The ORB features,which are oriented multiscale FAST cor-ners with a 256-bit descriptor associated, were implemented in this algorithm. These features are used for all the tasks tracking, mapping,relocalization, and loop closing. This facilitates good real time performance without the need for GPUs, which is a suitable option for our cost and hardware constraints. Further in-depth explanation of working of this State-of-the-art algorithm is provided in their research publication [ORB-SLAM: A Versatile and Accurate Monocular SLAM System](https://ieeexplore.ieee.org/document/7219438) published in IEEE transaction of robotics, Volume 31, Issue:5. The research publication DOI for ORB SLAM : 10.1109/TRO.2015.2463671

## ORB SLAM - Implementation

Clone the repository from github:
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```
or download the [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) repository to your home folder, rename the folder as 'ORB_SLAM2' and then execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```
This creates libORB_SLAM2.so at lib folder and the executables in Examples folder. The experimental simulation and testing of the algorithm can be done with TUM, KITTI and EuRoc datasets for monocular examples. The procedures have been further explained
by [raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

One such monocular test was done by [valarsundarlis/ORB-SLAM](https://github.com/valarsundarlis/ORB-SLAM)

One can process thier own sequences and deploy real-time after camera calibration by following step 8 : Processing your own sequences in the [git repository](https://github.com/raulmur/ORB_SLAM2) 

