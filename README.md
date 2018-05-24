# NeuroShield


- This script demonstrates the use of a regional pixel subsampling
 to learn and classify objects in the center of the frame
-  Display video frame and the region being monitored on the LCD
- Recognize continuously the center of the video frame    
Slow!! due to switch between modes CamToLCD/ CAMToFifo

- Region of interest= center rectangle of 121x121 pixels
 Feature vector= subsampling, by averaging internal blocks of 11x11 pixels

- When shutter button is depressed
    - if less than 2 seconds ==> learn category 1 and optionally increments
    - if more than 2 seconds ==> learn category 0 or background
    - optionally, but at the expense of the speed, save to the SD card:
    - the feature vectors (vectors.txt, 1 row per vector, also record the category taught)/        
    - the image (imgXcatY.dat, with X the img index, and Y the category taught)
    - the knowledge (neurons.knf)
    - The ArduCam_Console.exe allows to open these different files

## Hardware requirements

- Arduino/Genuino board & NeuroMem Shield board or NeuroMem shield board
- ArduCam Shield V2 with camera module

![Alt text](https://github.com/ArduCAM/NeuroShield/blob/master/image/image1.png)

![Alt text](https://github.com/ArduCAM/NeuroShield/blob/master/image/image2.png)

## Library requirements
 requires the following libraries

- ArduCAM at https://github.com/ArduCAM/Arduino
- UTFT4ArduCAM_SPI at https://github.com/ArduCAM/Arduino

## Video demo
### https://youtu.be/OWjTC9CULNI

![Alt text](https://github.com/ArduCAM/NeuroShield/blob/master/image/image3.png)
![Alt text](https://github.com/ArduCAM/NeuroShield/blob/master/image/image4.png)
![Alt text](https://github.com/ArduCAM/NeuroShield/blob/master/image/image5.png)

