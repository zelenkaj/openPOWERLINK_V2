/* ip_opt.h - IP Stack for FPGA MAC Controller */
/*
------------------------------------------------------------------------------

Copyright (c) 2009, B&R
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

- Neither the name of the B&R nor the names of
its contributors may be used to endorse or promote products derived
from this software without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

------------------------------------------------------------------------------
 Module:    ip
 File:      ip_opt.h
 Author:    Thomas Enzinger(enzingert)
 Created:   26.03.2004
------------------------------------------------------------------------------*/

#ifndef __IP_OPT_H__
#define __IP_OPT_H__

#include <oplk/oplkinc.h>

//-------------------------------------------------------------------------
// Number of entries in the ARP table
//
// Should be > number of connections from the local subnet
// For all connections from other subnets only 1 tab entry is required
//-------------------------------------------------------------------------
#define IP_ARP_TABSIZE			20		// size of ARP Table (12 Byte RAM / Entry)

//-------------------------------------------------------------------------
// MTU (Maximum Transmission Unit)
// 
// Buffers will be allocated with the following size: MTU + 14 + 4
// (14 byte ethernet header , 4 byte checksum)
//-------------------------------------------------------------------------
#define IP_MTU					1500

//-------------------------------------------------------------------------
// Number of IP buffers for the interface to the ethernet driver
//-------------------------------------------------------------------------
#define IP_RX_BUF_CNT			4	// number of rx buffers (allocated in mac layer)
#define IP_TX_BUF_CNT			4	// tx buffers

#ifndef IP_REASS_BUF_CNT
	#define IP_REASS_BUF_CNT		2	// number of buffers for reassembly
#endif

//-------------------------------------------------------------------------
// Number of reassembly buffers
// 
// Each buffer needs (MTU+18) bytes of RAM
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// If a reassembly buffer is not completed after this time the buffer
// will be deleted and reused when the next reassembly starts
// (Time in seconds)
//-------------------------------------------------------------------------
#define IP_REASS_MAXAGE			4

//-------------------------------------------------------------------------
// must be set to 1 if the host system is little endian
//-------------------------------------------------------------------------
#ifdef __NIOS2__
  #define LITTLE_ENDIAN			1
#elif(__MICROBLAZE__)
  #include "xparameters.h"

  #if XPAR_MICROBLAZE_ENDIANNESS == 1
    #define LITTLE_ENDIAN           1
  #else
    #define LITTLE_ENDIAN           0
  #endif
#else
  #error 'ERROR: Platform not found!'
#endif

//-------------------------------------------------------------------------
// defines if the system does provide a word-swap function
// (only relevant for little endian hosts)
//-------------------------------------------------------------------------
#define IP_SWAP_FUNCTION		0

//-------------------------------------------------------------------------
// Maximum number of listen ports    (RAM usage = x*12)
//-------------------------------------------------------------------------
#define IP_LISTEN_PORTS_UDP		5


//-------------------------------------------------------------------------
// maximum number of simultaneously open TCP connections
// (0 : tcpip disabled, less code)
//-------------------------------------------------------------------------
#define IP_TCP_SOCKETS			0

//-------------------------------------------------------------------------
// enable DHCP
// (0 : dhcp disabled, less code)
//-------------------------------------------------------------------------
#define IP_DHCP					0

//-------------------------------------------------------------------------
// statistics support
//
// The statistics is useful for debugging and to show the user but needs
// memory and runtime
//-------------------------------------------------------------------------
#define IP_STATISTICS			1

//  #define IP_SECONDARY_ADDRESS


//-------------------------------------------------------------------------
// interrupts support
//
// Enable disable global interrupts
// (Currently the whole IP stack is running in the same context so not
//  critical sections is required.)
//-------------------------------------------------------------------------
#define EnableGlobalInterrupt(x)

#endif

