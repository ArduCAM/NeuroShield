// NeuroMem_ArduCAM_Demo_1Feat
// Copyrights 2018, General Vision Inc.
//
//********************************************************************
//
// Warning! If you get an error when uploading this script
// you may have to disconnect the ArduCam Shield for the upload
//
//********************************************************************
//
// This script demonstrates the use of a regional pixel subsampling
// to learn and classify objects in the center of the frame
//
//  Display video frame and the region being monitored on the LCD
//
// Recognize continuously the center of the video frame
//    Slow!! due to switch between modes CamToLCD/ CAMToFifo
//
// Region of interest= center rectangle of 121x121 pixels
// Feature vector= subsampling, by averaging internal blocks of 11x11 pixels
//
// When shutter button is depressed
//    - if less than 2 seconds ==> learn category 1 and optionally increments
//    - if more than 2 seconds ==> learn category 0 or background
//    - optionally, but at the expense of the speed, save to the SD card:
//        - the feature vectors (vectors.txt, 1 row per vector, also record the category taught)
//        - the image (imgXcatY.dat, with X the img index, and Y the category taught)
//        - the knowledge (neurons.knf)
//    - The ArduCam_Console.exe allows to open these different files
//
// Hardware requirements
// ---------------------
// Arduino/Genuino board & NeuroMem Shield board
// or
// NeuroMem shield board
//
// ArduCam Shield V2 with camera module
//
// Library requirements
// ---------------------
// requires the following libraries
// ArduCAM at https://github.com/ArduCAM/Arduino
// UTFT4ArduCAM_SPI at https://github.com/ArduCAM/Arduino

#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include <UTFT_SPI.h>
#include <SD.h>
#include "memorysaver.h"

// NeuroMem platforms
#include <NeuroMemAI.h>
NeuroMemAI hNN;

int dist=0, cat=0, nid=0, ncount=0;
int catLearn=1, nextCat=1;
#define MAX_LEN 256 // memory of a neuron, maximum length of the input patterns
int neuron[MAX_LEN +8]; // to retrieve the content of an entire neuron
//
// variables for the feature extraction (relative to FIFO dimension)
// frame size read from the FIFO on ArduCAM Shield
//
int fw=320; 
int fh=240;
uint8_t fifo_burst_line[320*2];
//
// Definition of the region to monitor continuously
//
int rw = 128, rh = 128;
int rleft = (fw - rw)/2;
int rtop = (fh - rh) /2;
int rright = rleft + rw;
int rbottom= rtop + rh;

//
// parameters to extract feature#1: SubSample
// adjust rw, rh, bw, bh such that vlen < MAX_LEN
//
int bw = 8, bh = 8;
int hb = rw/bw, vb= rh/bh;
int vlen= hb*vb;
long subsample[256]; // buffer
int subsampleFeat[256]; // int array mapped to values [0-256] for the neurons
//
// Access to Camera
//
const int SPI_CS_CAM =10;
ArduCAM myCAM(OV2640, SPI_CS_CAM);
UTFT myGLCD(SPI_CS_CAM);
//
// Access to SD card
//
bool SD_detected=false;
File SDfile;
#define SPI_CS_SD 9
int sampleID=0; // to track the number of learned examples saved to SD card

void setup() {

  uint8_t vid = 0, pid = 0;
  uint8_t temp = 0;

  Wire.begin();
  Serial.begin(9600);
  while (!Serial);

  // initialize SPI:
  pinMode(SPI_CS_CAM, OUTPUT);
  SPI.begin();
  Serial.println("Welcome to the SMART_ArduCAM demo!");

  // Check if the ArduCAM SPI bus is OK
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if (temp != 0x55) {
    Serial.println("SPI interface Error!");
    Serial.println("Check your wiring, make sure using the correct SPI port and chipselect pin");
  }

  // Initialize the TFT LCD
  myGLCD.InitLCD();
  extern uint8_t BigFont[];
  myGLCD.setFont(BigFont);
  
  //Initialize SD Card 
  if (SD.begin(SPI_CS_SD))
  {
    SD_detected=true;
  }
  else
  {
      displayLCD_res("Warning: SD slot empty", 10,10);
      delay(100);
      Serial.println("Insert SD card if you wish to save vector data");
  }
  
  // Initialize the ArduCAM
  myCAM.InitCAM();
        
  // Check if the camera module type is OV2640
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
  if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) {
    Serial.println("Can't find OV2640 module!");
  } else {
    Serial.println("OV2640 detected.");
  }
  
  // Initialize the NeuroMem neural network
  int NMplatform=0;
  Serial.print("\nSelect your NeuroMem platform:\n\t1 - BrainCard\n\t2 - NeuroShield\n\t3 - NeuroTile\n");
  while(Serial.available() <1);
  NMplatform = Serial.read() - 48;      
  if (hNN.begin(NMplatform) == 0) 
  {
    Serial.print("\nYour NeuroMem_Smart device is initialized! ");
    Serial.print("\nThere are "); Serial.print(hNN.navail); Serial.print(" neurons\n");     
  }
  else 
  {
    Serial.print("\nYour NeuroMem_Smart device is NOT found!");
    Serial.print("\nCheck your device type and connection.\n");
    while (1);
  }
  
  Serial.print("Image width="); Serial.print(fw); Serial.print(", height="); Serial.println(fh);
  Serial.print("ROI width="); Serial.print(rw); Serial.print(", height="); Serial.println(rh);
  displayLCD_res("Ready", 10,10);
  delay(100);
}

void loop() {

  while (1) {
    // New Frame is coming
    if (!myCAM.get_bit(ARDUCHIP_TRIG, VSYNC_MASK)) 
    {
      myCAM.set_mode(MCU2LCD_MODE);
      myGLCD.resetXY();
      myCAM.set_mode(CAM2LCD_MODE);
      while (!myCAM.get_bit(ARDUCHIP_TRIG, VSYNC_MASK));      
      getFeatureVectors();
      recognize();
    } 
    else if (myCAM.get_bit(ARDUCHIP_TRIG, SHUTTER_MASK))
    {         
      long timer_start = millis();
      long tmp=0;
      while (myCAM.get_bit(ARDUCHIP_TRIG, SHUTTER_MASK))
      {
        tmp=millis() - timer_start;
        //Serial.print("time="); Serial.println(tmp);        
      }
      getFeatureVectors(); 
      if (tmp > 2000)
      {
        catLearn=0;
      }
      else 
      {
         catLearn=nextCat;
         nextCat++;
        // Option to comment the nextCat increment if you are teaching a single type of objects
        // but you may have to teach background examples (pressing the shutter more than 2 sec)
        // to avoid that the neurons overgeneralize
      }
      learn(catLearn);
      if (SD_detected==true)
      {
          // time consuming options...for debug or further analysis
          int error=hNN.saveKnowledge_SDcard("neurons.knf");
          if (error!=0) Serial.print("\n\nError saving knowledge to SD card\n");
          //saveVectors(catLearn); Serial.print("\n\nSaving vectors to SD card\n");
          //saveImage(catLearn);
          sampleID++;
      }
    }
  }
}

void getFeatureVectors() {
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  //Serial.println("Capture Done!");
  
  uint32_t length = 0;
  length = myCAM.read_fifo_length();
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();//Set fifo burst mode
  
  // Read captured image as BMP565 format (320x240x 2 bytes from FIFO)
  // extract the three features on the fly
  char VH, VL;
  int indexFeat1X, indexFeat1Y, index = 0;
  int color, r, g, b, greylevel, maxgrey=0, mingrey=255;
  for (int i = 0; i < vlen; i++) subsample[i] = 0; 
  for (int y = 0 ; y < fh ; y++)
  {
    SPI.transfer(fifo_burst_line, fw*2);//read one line from spi  
    for (int x = 0 ; x < fw ; x++)
    {
      VH = fifo_burst_line[x*2];
      VL = fifo_burst_line[x*2+1];
      color = (VH << 8) + VL;

      if (y >= rtop && y < rbottom)
      {       
        indexFeat1Y= (y - rtop) / bh;
        if (x >= rleft && x < rright)
        {             
          r = ((color >> 11) & 0x1F);
          g = ((color >> 5) & 0x1F);
          b = (color & 0x001F);
          greylevel= r+g+b; // byte value
          if (greylevel > maxgrey) maxgrey=greylevel;
          else if (greylevel < mingrey) mingrey=greylevel;

          indexFeat1X= (x - rleft) / bw;
          if ((indexFeat1X < hb) & (indexFeat1Y < vb))
          {
              index = (indexFeat1Y * hb) + indexFeat1X;
              subsample[index] = subsample[index] + greylevel;
          }
        }
      }
    }
  }
  
  myCAM.CS_HIGH();
  
  //Serial.print("\nBlock intensity Max="); Serial.print(maxgrey); Serial.print("\tMin="); Serial.println(mingrey);
  for (int i = 0; i < vlen; i++) subsampleFeat[i] = (byte)(subsample[i] / (bw * bh));
  // or to normalized
  // for (int i = 0; i < vlen; i++) subsampleFeat[i] = (byte)(((subsample[i] / (bw * bh))- mingrey)*255 / (maxgrey-mingrey));
}

void recognize() 
{
  // recognize feature vector #1 or subsample vector
  hNN.classify(subsampleFeat, vlen, &dist, &cat, &nid);
  // recognize feature vector #2 or histogram rgb
   
  char tmpStr[10];
  char Str[40] = {""};
  if (cat!=0xFFFF) 
  {
      strcat(Str,"Cat=");
      itoa (cat, tmpStr, 10);
      strcat(Str, tmpStr);
      strcat(Str," at dist=");
      itoa (dist, tmpStr, 10);
      strcat(Str, tmpStr);
      Serial.println(Str);    
  }
  else
  {
    strcat(Str,"Unknown");    
  }
  displayLCD_res(Str, 5,5);
  delay(10); // to sustain the display
}

void learn(int Category) 
{
  // learn feature vector #1 or subsample vector  
 ncount= hNN.learn(subsampleFeat, vlen, Category);
  
  char tmpStr[10];
  char Str[40] = {""};
  if (Category !=0)
  {
    strcat(Str, "Neurons=");
    itoa (ncount, tmpStr, 10);
    strcat(Str, tmpStr);
  }
  else
  {
    strcat(Str,"Forget");
  }
  Serial.println(Str);
  displayLCD_res(Str, 5, 220);
  delay(100); // to sustain the display
}

void saveVectors(int Category)
{
  // Save to a format compatible with the NeuroMem Knowledge Builder
  // Each feature type is assigned a different context
  char vectFilename[12] = "vectors.txt";
  if (sampleID==0)
  {
        if (SD.exists(vectFilename)) SD.remove(vectFilename);
  }
    SDfile = SD.open(vectFilename, FILE_WRITE);
    if(! SDfile)
    {
      Serial.print(vectFilename); Serial.println(" file open failed");
      return;
   }
    Serial.print("\nSaving to SD card vector#"); Serial.println(sampleID);
    // header for NeuroMem Knowledge Builder
    if (sampleID==0)SDfile.print("patternID, parentID, Context, GTcategory, v1\n");
    SDfile.print(sampleID); SDfile.print(","); // patternID
    SDfile.print("0,");  // parentID
    SDfile.print("1,"); // context
    SDfile.print(Category); SDfile.print(",");
    SDfile.print(vlen); SDfile.print(",");
    for (int i=0; i< vlen; i++) {SDfile.print(subsampleFeat[i]); SDfile.print(",");}
    SDfile.print("\n");
    SDfile.close();
    Serial.println("Vectors saved!");
}

void saveImage(int Category)
{
 //Construct a file name
 char imgFilename[20]="img";
 char strtmp[10];
 itoa(sampleID, strtmp, 10);
 strcat(imgFilename, strtmp);
 strcat(imgFilename, "cat");
 itoa(Category, strtmp, 10);
 strcat(imgFilename, strtmp);
 strcat(imgFilename, ".dat");
 
 if (SD.exists(imgFilename)) SD.remove(imgFilename);
 //SDfile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
 SDfile = SD.open(imgFilename, FILE_WRITE);
 if(! SDfile){
  Serial.print(imgFilename); Serial.println(" file open failed");
  return;
 }
   Serial.print("\nSaving to SD card "); Serial.print(imgFilename);

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
  //Serial.println("Capture Done!");
  
  myCAM.read_fifo(); // Read the first dummy byte
  
  byte pixeline[320]; // fw=320
  char VH, VL;
  int color, r, g, b, greylevel;
  for (int y = 0 ; y < fh ; y++)
  {
    for (int x = 0 ; x < fw ; x++)
    {
      VL = myCAM.read_fifo();
      VH = myCAM.read_fifo();
      color = (VH << 8) + VL;            
      r = ((color >> 11) & 0x1F);
      g = ((color >> 5) & 0x3F);
      b = (color & 0x001F);
      greylevel= r+g+b;
      pixeline[x] = (byte)(greylevel);        
    }
    SDfile.write(pixeline, fw);
  }
  SDfile.close();
  Serial.println("\nImage saved!");
 }

void displayLCD_res(char* Str, int x, int y)
{
  myCAM.set_mode(MCU2LCD_MODE);
  myGLCD.setColor(255, 0, 225);
  myGLCD.print(Str,x,y,0);
  myGLCD.setColor(255, 0, 225);
  myGLCD.drawRect(rleft, rtop, rright, rbottom);
}
