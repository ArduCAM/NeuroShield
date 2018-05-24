/************************************************************************/
/*																		
 *	NeuroMemAI.cpp	--	Class to use a NeuroMem network	
 
 *	Copyright (c) 2016, General Vision Inc, All rights reserved	
 *
 *  Created September 8, 2017
 *
 *
 * Updated 11/09/2017
 * In order to accommodate future models of NeuroMem chips with neuron memory size
 * different from a 256 byte array, the data type of vector, model and neuron has
 * been changed from unsigned char to int array. The upper bytes are always null in the case of the CM1K and NM500 chips.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/******************************************************************************/


/* ------------------------------------------------------------ */
/* www.general-vision.com/documentation/TM_NeuroMem_API.pdf		*/
/* ------------------------------------------------------------ */

#include <NeuroMemAI.h>
#include <NeuroMemSPI.h>

// Modules of the NeuroMem hardware (Byte #0 in map address)
static const int mod_NM=0x01;
	// possible additional modules depending on hardware
	// can be a sensor, cmos, flash memory, etc

// Registers of a NeuroMem network (Byte #3 in map address)
static const int NM_NCR=0x00;
static const int NM_COMP=0x01;
static const int NM_LCOMP=0x02;
static const int NM_DIST=0x03;
static const int NM_INDEXCOMP=0x03;
static const int NM_CAT=0x04;
static const int NM_AIF=0x05;
static const int NM_MINIF=0x06;
static const int NM_MAXIF=0x07;
static const int NM_TESTCOMP=0x08;
static const int NM_TESTCAT=0x09;
static const int NM_NID=0x0A;
static const int NM_GCR=0x0B;
static const int NM_RESETCHAIN=0x0C;
static const int NM_NSR=0x0D;
static const int NM_NCOUNT=0x0F	;
static const int NM_FORGET=0x0F;

#include <SD.h>

using namespace std;
extern "C" {
  #include <stdint.h>
}

NeuroMemSPI spi;

// ------------------------------------------------------------ //
//    Constructor to the class NeuroMemAI
// ------------------------------------------------------------ 
NeuroMemAI::NeuroMemAI(){	
}
// ------------------------------------------------------------ 
// Initialize the neural network
// ------------------------------------------------------------ 
#define HW_BRAINCARD 1 // neuron access through SPI_CS pin 10
#define HW_NEUROSHIELD 2 // neuron access through SPI_CS pin 7
#define HW_NEUROTILE 3 // neuron access through SPI_CS pin 10

// SD card chip select
#define SD_CS_BRAINCARD 9
#define SD_CS_NEUROSHIELD 6
#define SD_CS_NEUROTILE 0

int NeuroMemAI::begin(int Platform)
{
	int error=spi.connect(Platform);
	if (error==0) 
	{
		countNeuronsAvailable(); // update the global navail
		clearNeurons();
		switch(Platform)
		{
			case HW_BRAINCARD: SD_select=SD_CS_BRAINCARD; break;
			case HW_NEUROSHIELD: SD_select=SD_CS_NEUROSHIELD; break;
			case HW_NEUROTILE: SD_select=SD_CS_NEUROTILE; break;
		}
		SD_detected=SD.begin(SD_select);
	}
	return(error);
}
// ------------------------------------------------------------ 
// Un-commit all the neurons, so they all become ready to learn
// Reset the Maximum Influence Field to default value=0x4000
// ------------------------------------------------------------ 
void NeuroMemAI::forget()
{
	spi.write(mod_NM, NM_FORGET, 0);
}
// ------------------------------------------------------------ 
// Un-commit all the neurons, so they all become ready to learn,
// Set the Maximum Influence Field (default value=0x4000)
// ------------------------------------------------------------ 
void NeuroMemAI::forget(int Maxif)
{
	spi.write(mod_NM, NM_FORGET, 0);
	spi.write(mod_NM, NM_MAXIF, Maxif);
}
// --------------------------------------------------------------
// Clear the memory of the neurons to the value 0
// Reset default GCR=1, MINIF=2, MANIF=0x4000, CAT=0
// --------------------------------------------------------------
void NeuroMemAI::clearNeurons()
{
	spi.write(mod_NM, NM_NSR, 16);
	spi.write(mod_NM, NM_TESTCAT, 0x0001);
	spi.write(mod_NM, NM_NSR, 0);
	for (int i=0; i< NEURONSIZE; i++)
	{
		spi.write(mod_NM, NM_INDEXCOMP,i);
		spi.write(mod_NM, NM_TESTCOMP,0);
	}
	spi.write(mod_NM, NM_FORGET,0);
}
// ------------------------------------------------------------ 
// Detect the capacity of the NeuroMem network
// plugged on the board
// ------------------------------------------------------------ 
int NeuroMemAI::countNeuronsAvailable()
{
	spi.write(mod_NM, NM_FORGET, 0);
	spi.write(mod_NM, NM_NSR, 0x0010);
	spi.write(mod_NM, NM_TESTCAT, 0x0001);
	spi.write(mod_NM, NM_RESETCHAIN, 0);
	int read_cat;	
	navail = 0;
	while (1) {
		read_cat = spi.read(mod_NM, NM_CAT);
		if (read_cat == 0xFFFF)
			break;
		navail++;
	}
	spi.write(mod_NM, NM_NSR, 0x0000);
	spi.write(mod_NM, NM_FORGET, 0);
	return(navail);
}
// --------------------------------------------------------
// Broadcast a vector to the neurons and return the recognition status
// 0= unknown, 4=uncertain, 8=Identified
//---------------------------------------------------------
int NeuroMemAI::broadcast(int vector[], int length)
{
	//spi.writeAddr(0x01000001, length-1, vector);
	for (int i=0; i<length-1;i++) spi.write(mod_NM, NM_COMP, vector[i] & 0x00FF);
	spi.write(mod_NM, NM_LCOMP, vector[length-1]);
	return(spi.read(mod_NM, NM_NSR));
}
//-----------------------------------------------
// Learn a vector using the current context value
//----------------------------------------------
int NeuroMemAI::learn(int vector[], int length, int category)
{
	broadcast(vector, length);
	spi.write(mod_NM, NM_CAT,category);
	return(spi.read(mod_NM, NM_NCOUNT));
}
// ---------------------------------------------------------
// Classify a vector and return its classification status
// NSR=0, unknown
// NSR=8, identified
// NSR=4, uncertain
// ---------------------------------------------------------
int NeuroMemAI::classify(int vector[], int length)
{
	broadcast(vector, length);
	return(spi.read(mod_NM, NM_NSR));
}
//----------------------------------------------
// Recognize a vector and return the best match, or the 
// category, distance and identifier of the top firing neuron
//----------------------------------------------
int NeuroMemAI::classify(int vector[], int length, int* distance, int* category, int* nid)
{
	broadcast(vector, length);
	*distance = spi.read(mod_NM, NM_DIST);
	*category= spi.read(mod_NM, NM_CAT); //remark : Bit15 = degenerated flag, true value = bit[14:0]
	*nid =spi.read(mod_NM, NM_NID);
	return(spi.read(mod_NM, NM_NSR));
}
//----------------------------------------------
// Recognize a vector and return the response  of up to K top firing neurons
// The response includes the distance, category and identifier of the neuron
// The Degenerated flag of the category is masked rmask the degenerated response, use the current context value
// Return the number of firing neurons or K whichever is smaller
//----------------------------------------------
int NeuroMemAI::classify(int vector[], int length, int K, int distance[], int category[], int nid[])
{
	int recoNbr=0;
	broadcast(vector, length);
	for (int i=0; i<K; i++)
	{
		distance[i] = spi.read(mod_NM, NM_DIST);
		if (distance[i]==0xFFFF)
		{ 
			category[i]=0xFFFF;
			nid[i]=0xFFFF;
		}
		else
		{
			recoNbr++;
			category[i]= spi.read(mod_NM, NM_CAT); //remark : Bit15 = degenerated flag, true value = bit[14:0]
			nid[i] =spi.read(mod_NM, NM_NID);
		}
	}
return(recoNbr);
}
// ------------------------------------------------------------ 
// Set a context and associated minimum and maximum influence fields
// ------------------------------------------------------------ 
void NeuroMemAI::setContext(int context, int minif, int maxif)
{
	// context[15-8]= unused
	// context[7]= Norm (0 for L1; 1 for LSup)
	// context[6-0]= Active context value
	spi.write(mod_NM, NM_GCR, context);
	spi.write(mod_NM, NM_MINIF, minif);
	spi.write(mod_NM, NM_MAXIF, maxif);
}
// ------------------------------------------------------------ 
// Get a context and associated minimum and maximum influence fields
// ------------------------------------------------------------ 
void NeuroMemAI::getContext(int* context, int* minif, int* maxif)
{
	// context[15-8]= unused
	// context[7]= Norm (0 for L1; 1 for LSup)
	// context[6-0]= Active context value
	*context = spi.read(mod_NM, NM_GCR);
	*minif= spi.read(mod_NM, NM_MINIF); 
	*maxif =spi.read(mod_NM, NM_MAXIF);
}
// --------------------------------------------------------
// Set the neurons in Radial Basis Function mode (default)
//---------------------------------------------------------
void NeuroMemAI::setRBF()
{
	int tempNSR=spi.read(mod_NM, NM_NSR);
	spi.write(mod_NM, NM_NSR, tempNSR & 0xDF);
}
// --------------------------------------------------------
// Set the neurons in K-Nearest Neighbor mode
//---------------------------------------------------------
void NeuroMemAI::setKNN()
{
	int tempNSR=spi.read(mod_NM, NM_NSR);
	spi.write(mod_NM, NM_NSR, tempNSR | 0x20);
}
//-------------------------------------------------------------
// Read the contents of the neuron pointed by index in the chain of neurons
// starting at index 0
//-------------------------------------------------------------
void NeuroMemAI::readNeuron(int nid, int model[], int* context, int* aif, int* category)
{
	int TempNSR=spi.read(mod_NM, NM_NSR);
	spi.write(mod_NM, NM_NSR, 0x10);
	spi.write(mod_NM, NM_RESETCHAIN, 0);
	if (nid>0)
	{
		 // move to index in the chain of neurons
		 for (int i=0; i<nid; i++) spi.read(mod_NM, NM_CAT);
	}
	*context=spi.read(mod_NM, NM_NCR);
	for (int j=0; j<NEURONSIZE; j++) model[j]=spi.read(mod_NM, NM_COMP);
	*aif=spi.read(mod_NM, NM_AIF);
	*category=spi.read(mod_NM, NM_CAT);
	spi.write(mod_NM, NM_NSR, TempNSR); // set the NN back to its calling status
}
//-------------------------------------------------------------
// Read the contents of the neuron pointed by index in the chain of neurons
// starting index is 0
// Returns an array of integers of length NEURONSIZE + 4
// and with the following format
// NCR, NEURONSIZE * COMP, AIF, MINIF, CAT
//-------------------------------------------------------------
void NeuroMemAI::readNeuron(int nid, int neuron[])
{
	int TempNSR=spi.read(mod_NM, NM_NSR);
	spi.write(mod_NM, NM_NSR, 0x10);
	spi.write(mod_NM, NM_RESETCHAIN, 0);
	if (nid>0)
	{
		 // move to index in the chain of neurons
		 for (int i=0; i<nid; i++) spi.read(mod_NM, NM_CAT);
	}
	neuron[0]=spi.read(mod_NM, NM_NCR);
	for (int j=0; j<NEURONSIZE; j++) neuron[j+1]=spi.read(mod_NM, NM_COMP);
	neuron[NEURONSIZE+1]=spi.read(mod_NM, NM_AIF);
	neuron[NEURONSIZE+2]=spi.read(mod_NM, NM_MINIF);
	neuron[NEURONSIZE+3]=spi.read(mod_NM, NM_CAT);
	spi.write(mod_NM, NM_NSR, TempNSR); // set the NN back to its calling status
}
//----------------------------------------------------------------------------
// Read the contents of the committed neurons
// The output array has a dimension ncount * neurondata
// neurondata describes the content of a neuron and has a dimension (NEURONSIZE + 4)
// and with the following format
// NCR, NEURONSIZE * COMP, AIF, MINIF, CAT
//----------------------------------------------------------------------------
int NeuroMemAI::readNeurons(int neurons[])
{
	int ncount= spi.read(mod_NM, NM_NCOUNT);
	int TempNSR=spi.read(mod_NM, NM_NSR); // save value to restore upon exit
	spi.write(mod_NM, NM_NSR, 0x0010);
	spi.write(mod_NM, NM_RESETCHAIN, 0);
	int offset=0;
	int recLen=NEURONSIZE+4; // memory plus 4 int of neuron registers	
	for (int i=0; i< ncount; i++)
	{
		neurons[offset]=spi.read(mod_NM, NM_NCR);
		for (int j=0; j< NEURONSIZE; j++) neurons[offset + 1 + j]=spi.read(mod_NM, NM_COMP);
		neurons[offset + NEURONSIZE + 1]=spi.read(mod_NM, NM_AIF);
		neurons[offset + NEURONSIZE + 2]=spi.read(mod_NM, NM_MINIF);
		neurons[offset + NEURONSIZE + 3]=spi.read(mod_NM, NM_CAT);
		offset+=recLen;
	}
	spi.write(mod_NM, NM_NSR, TempNSR); // set the NN back to its calling status
	return(ncount);
}

//---------------------------------------------------------------------
// Clear the neurons and write their content from an input array
// The input array has a dimension ncount * neurondata
// neurondata describes the content of a neuron and has a dimension (NEURONSIZE + 4)
// and with the following format
// NCR, NEURONSIZE * COMP, AIF, MINIF, CAT
//---------------------------------------------------------------------
void NeuroMemAI::writeNeurons(int neurons[], int ncount)
{
	int TempNSR=spi.read(mod_NM, NM_NSR); // save value to restore NN upon exit
	int TempGCR=spi.read(mod_NM, NM_GCR);
	clearNeurons();
	spi.write(mod_NM, NM_NSR, 0x0010);
	spi.write(mod_NM, NM_RESETCHAIN, 0);		
	int offset=0;
	int recLen=NEURONSIZE+4;	
	for (int i=0; i< ncount; i++)
	{	
		spi.write(mod_NM, NM_NCR, neurons[offset]);
		for (int j=0; j<NEURONSIZE; j++) spi.write(mod_NM, NM_COMP, neurons[offset+1+j]);
		spi.write(mod_NM, NM_AIF,neurons[offset + NEURONSIZE + 1]);
		spi.write(mod_NM, NM_MINIF, neurons[offset + NEURONSIZE+ 2]);	
		spi.write(mod_NM, NM_CAT, neurons[offset + NEURONSIZE+ 3]);
		offset+=recLen;
	}
	spi.write(mod_NM, NM_NSR, TempNSR); // set the NN back to its calling status
	spi.write(mod_NM, NM_GCR, TempGCR);
}

// --------------------------------------------------------
// Read the number of committed neurons
//---------------------------------------------------------
int NeuroMemAI::NCOUNT()
{
	return(spi.read(mod_NM, NM_NCOUNT));
}
// --------------------------------------------------------
// Get/Set the Minimum Influence Field register
//---------------------------------------------------------
void NeuroMemAI::MINIF(int value)
{
	spi.write(mod_NM, NM_MINIF, value);
}
int NeuroMemAI::MINIF()
{
	return(spi.read(mod_NM, NM_MINIF));
}
// --------------------------------------------------------
// Get/Set the Maximum Influence Field register
//---------------------------------------------------------
void NeuroMemAI::MAXIF(int value)
{
	spi.write(mod_NM, NM_MAXIF, value);
}
int NeuroMemAI::MAXIF()
{
	return(spi.read(mod_NM, NM_MAXIF));
}
// --------------------------------------------------------
// Get/Set the Global Context register
//---------------------------------------------------------
void NeuroMemAI::GCR(int value)
{
	// GCR[15-8]= unused
	// GCR[7]= Norm (0 for L1; 1 for LSup)
	// GCR[6-0]= Active context value
	spi.write(mod_NM, NM_GCR, value);
}
int NeuroMemAI::GCR()
{
	return(spi.read(mod_NM, NM_GCR));
}
// --------------------------------------------------------
// Get/Set the Category register
//---------------------------------------------------------
void NeuroMemAI::CAT(int value)
{
	spi.write(mod_NM, NM_CAT, value);
}
int NeuroMemAI::CAT()
{
	return(spi.read(mod_NM, NM_CAT));
}
// --------------------------------------------------------
// Get the Distance register
//---------------------------------------------------------
int NeuroMemAI::DIST()
{	
	return(spi.read(mod_NM, NM_DIST));
}
// --------------------------------------------------------
// Set the Component Index register
//---------------------------------------------------------
void NeuroMemAI::NID(int value)
{
	spi.write(mod_NM, NM_NID, value);
}
// --------------------------------------------------------
// Get/Set the Network Status register
// bit 2 = UNC (read only)
// bit 3 = ID (read only)
// bit 4 = SR mode
// bit 5= KNN mode
//---------------------------------------------------------
void NeuroMemAI::NSR(int value)
{
	spi.write(mod_NM, NM_NSR, value);
}
int NeuroMemAI::NSR()
{
	return(spi.read(mod_NM, NM_NSR));
}
// --------------------------------------------------------
// Get/Set the AIF register
//---------------------------------------------------------
void NeuroMemAI::AIF(int value)
{
	spi.write(mod_NM, NM_AIF, value);
}
int NeuroMemAI::AIF()
{
	return(spi.read(mod_NM, NM_AIF));
}
// --------------------------------------------------------
// Reset the chain to first neuron in SR Mode
//---------------------------------------------------------
void NeuroMemAI::RESETCHAIN()
{
	spi.write(mod_NM, NM_RESETCHAIN, 0);
}
// --------------------------------------------------------
// Get/Set the NCR register
//---------------------------------------------------------
void NeuroMemAI::NCR(int value)
{
	spi.write(mod_NM, NM_NCR, value);
}
int NeuroMemAI::NCR()
{
	return(spi.read(mod_NM, NM_NCR));
}
// --------------------------------------------------------
// Get/Set the COMP register (component)
//---------------------------------------------------------
void NeuroMemAI::COMP(int value)
{
	spi.write(mod_NM, NM_COMP, value);
}
int NeuroMemAI::COMP()
{
	return(spi.read(mod_NM, NM_COMP));
}
// --------------------------------------------------------
// Get/Set the LCOMP register (last component)
//---------------------------------------------------------
void NeuroMemAI::LCOMP(int value)
{
	spi.write(mod_NM, NM_LCOMP, value);
}
int NeuroMemAI::LCOMP()
{
	return(spi.read(mod_NM, NM_LCOMP));
}

// --------------------------------------------------------
// Save the knowledge of the neurons to a knowledge file
// saved in a format compatible with the NeuroMem API
// --------------------------------------------------------
int NeuroMemAI::saveKnowledge_SDcard(char* filename)
{
	if (!SD_detected)
	{
		SD_detected=SD.begin(SD_select);
	}
	if (!SD_detected) return(1);
	if (SD.exists(filename)) SD.remove(filename);
    File SDfile = SD.open(filename, FILE_WRITE);
    if(! SDfile) return(3);

    int header[4]{ KN_FORMAT, 0,0,0 };
    header[1] = NEURONSIZE;
    int ncount = NCOUNT();
    header[2]=ncount;
    int* p_myheader = header;
    byte* b_myheader = (byte*)p_myheader;    
    SDfile.write(b_myheader, sizeof(int)*4);

    int neuron[NEURONSIZE + 4];
    int* p_myneuron = neuron;
    byte* b_myneuron = (byte*)p_myneuron;
	int TempNSR=spi.read(mod_NM, NM_NSR);
	spi.write(mod_NM, NM_NSR, 0x10);
	spi.write(mod_NM, NM_RESETCHAIN, 0);
	for (int i=0; i< ncount; i++)
	{
		neuron[0]=spi.read(mod_NM, NM_NCR);
		for (int j=0; j<NEURONSIZE; j++) neuron[j+1]=spi.read(mod_NM, NM_COMP);
		neuron[NEURONSIZE+1]=spi.read(mod_NM, NM_AIF);
		neuron[NEURONSIZE+2]=spi.read(mod_NM, NM_MINIF);
		neuron[NEURONSIZE+3]=spi.read(mod_NM, NM_CAT);		     
		SDfile.write(b_myneuron, sizeof(int)*(NEURONSIZE + 4));
	}
	spi.write(mod_NM, NM_NSR, TempNSR); // set the NN back to its calling status	
    SDfile.close();
	return(0); 
}
// --------------------------------------------------------
// Load the neurons with a knowledge stored in a knowledge file
// saved in a format compatible with the NeuroMem API
// --------------------------------------------------------
int NeuroMemAI::loadKnowledge_SDcard(char* filename)
{
	if (!SD_detected)
	{
		SD_detected=SD.begin(SD_select);
	}
	if (!SD_detected) return(1);
	if (!SD.exists(filename)) return(2); 
    File SDfile = SD.open(filename, FILE_READ);
    if (!SDfile) return(3);
	
    int header[4];
    int* p_myheader = header;
    byte* b_myheader = (byte*)p_myheader;    
    SDfile.read(b_myheader, sizeof(int)*4);  
    if (header[0] < KN_FORMAT) return(4);
    if (header[1] > NEURONSIZE) return(5);
	int ncount=0;    
    if (header[2] > navail)return(6); 
		else ncount=header[2]; // incompatible neuron size
		
    int neuron[NEURONSIZE + 4];
    int* p_myneuron = neuron;
    byte* b_myneuron = (byte*)p_myneuron; 
    while(SDfile.available())
	{
		int TempGCR=spi.read(mod_NM, NM_GCR);
		int TempNSR=spi.read(mod_NM, NM_NSR); // save value to restore NN upon exit	
		clearNeurons();
		spi.write(mod_NM, NM_NSR, 0x0010);
		spi.write(mod_NM, NM_RESETCHAIN, 0);		
		for (int i=0; i<ncount; i++)
		{
			SDfile.read(b_myneuron, sizeof(int)*(NEURONSIZE + 4));
			spi.write(mod_NM, NM_NCR, neuron[0]);
			for (int j=0; j<NEURONSIZE; j++) spi.write(mod_NM, NM_COMP, neuron[1+j]);
			spi.write(mod_NM, NM_AIF,neuron[NEURONSIZE + 1]);
			spi.write(mod_NM, NM_MINIF, neuron[NEURONSIZE+ 2]);	
			spi.write(mod_NM, NM_CAT, neuron[NEURONSIZE+ 3]);
		}
		spi.write(mod_NM, NM_NSR, TempNSR); // set the NN back to its calling status
		spi.write(mod_NM, NM_GCR, TempGCR);
	}		
	SDfile.close();
	return(0); 
}