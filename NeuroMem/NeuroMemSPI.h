/************************************************************************/
/*																		
 *	NeuroMemSPI.h	--	SPI driver for a NeuroMem hardware			        	
 *	Copyright (c) 2017, General Vision Inc, All rights reserved
 *
 */
/******************************************************************************/
#ifndef _NeuroMemSPI_h_
#define _NeuroMemSPI_h_

#include "SPI.h"

extern "C" {
  #include <stdint.h>
}

class NeuroMemSPI
{
	public:
			
		static const int mod_NM=0x01; // Addr[24:31] to access a NeuroMem chip
			
		NeuroMemSPI();
		int platform=0;		
		int connect(int Platform);		
		int FPGArev();			
		int read(unsigned char mod, unsigned char reg);
		void write(unsigned char mod, unsigned char reg, int data);
		void writeAddr(long addr, int length, int data[]);
		void readAddr(long addr, int length, int data[]);						
		
};
#endif
