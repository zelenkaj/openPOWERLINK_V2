/* hton.h - IP Stack for FPGA MAC Controller */
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
 File:      hton.h
 Author:    Thomas Enzinger(enzingert)
 Created:   26.03.2004
------------------------------------------------------------------------------*/

#ifndef __HTON_H__
#define __HTON_H__

#include "ip_opt.h"

#if LITTLE_ENDIAN==1
    // Host to Big Endian (headers)
    #define HTONS(x)    (unsigned short)((((unsigned short)((x) & 0xFF)) << 8) | (((x) & 0xFF00) >> 8))

        //-----------------------------------------------------------------------------
    // htons = Host TO Network Short
    // ... converts shorts (16 Bit) from host to network byte order
    // (for constants it is more efficient to use the macro HTONS()
    //
    #if IP_SWAP_FUNCTION == 1
        unsigned short _SwapWord(unsigned short x); // declaration of swap function
        unsigned long  _SwapLong(unsigned long x);  // declaration of swap function

        #define htons(x) _SwapWord(x)
        #define htonl(x) _SwapLong(x)
    #else
        unsigned short htons(unsigned short x);
        unsigned long  htonl(unsigned long x);
    #endif
#else
    // Host(Big Endian) to Network (Big Endian)
    #define HTONS(x)    (x)
    #define htons(x)    (x)
    #define htonl(x)    (x)
#endif  // LITTLE_ENDIAN == 1

#endif //__HTON_H__
