/* hton.c - IP Stack for FPGA MAC Controller */
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
 File:      hton.c
 Author:    Thomas Enzinger(enzingert)
 Created:   26.03.2004
------------------------------------------------------------------------------*/

#include "hton.h"


#if LITTLE_ENDIAN==1
    //-----------------------------------------------------------------------------
    // htons = Host TO Network Short
    // ... converts shorts (16 Bit) from host to network byte order
    // (for constants it is more efficient to use the macro HTONS()
    //
    #if IP_SWAP_FUNCTION == 0
        unsigned short htons(unsigned short x)
        {
            return (unsigned short)((x>>8)+(x<<8));
        }

        unsigned long htonl(unsigned long x)
        {
            volatile unsigned long ret = 0;
            //unsigned long *pbSrc = &x;

            ret |= (x & 0xFF000000) >> 24;
            ret |= (x & 0x00FF0000) >> 8;
            ret |= (x & 0x0000FF00) << 8;
            ret |= (x & 0x000000FF) << 24;

            return ret;
        }
    #endif

    // convert long to network byte order
    void htonlc(void *pDst, void* pSrc)
    {
        char tmp[2];    // temp is required in case pDst and pSrc are the same address

        tmp[0] = ((char*)pSrc)[0];
        tmp[1] = ((char*)pSrc)[1];

        ((char*)pDst)[0] = ((char*)pSrc)[3];
        ((char*)pDst)[1] = ((char*)pSrc)[2];

        ((char*)pDst)[2] = tmp[1];
        ((char*)pDst)[3] = tmp[0];
    }
#else
    // convert long to network byte order
    void htonlc(void *pDst, void* pSrc)
    {
        ((short*)pDst)[0] = ((short*)pSrc)[0];
        ((short*)pDst)[1] = ((short*)pSrc)[1];
    }
#endif
