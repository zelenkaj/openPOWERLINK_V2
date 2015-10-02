/* ip_name.h - Basic Name Service (NETBIOS) */ 
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
 Module:    ip_name
 File:      ip_name.c
 Author:    Thomas Enzinger(enzingert)
 Created:   26.03.2004
------------------------------------------------------------------------------
 History:
 26.03.2004	enzinger	created
 29.10.2004	enzinger	Function ipNetbiosInit() : Parameter pMacAddr not required anymore
 06.12.2005	enzinger	New: Funciton ipNetbiosSetCallback()
 28.11.2006	enzinger	New: Funciton ipNetbiosGetName()

------------------------------------------------------------------------------
*/

/*****************************************************************************
DESCRIPTION

  The function ipNetbiosInit() installs a service which answeres netbios name
  requests on port 137.
  The service will responde to a default name and to another name which can
  be set by the user with the function ipNetbiosSetName()

  The default name has the following format "BR001122334455", the numbers 0-5
  are replaced with the local MAC address.

  Example:

  Local MAC    = 0x00,0xBD,0x3B,0x33,0x45,0x10
  Default Name = BR00BD3B334510

  A name can have max 16 characters.

STATE:
 stabil

*/

#ifndef _IP_NAME_H_
#define _IP_NAME_H_

#include "ip.h"

/*****************************************************************************
* 
* ipNetbiosInit - initialize NetBIOS Name Service
* 
* The function must be called after the initialization of the IP stack
* to install the NetBIOS Name Service on Port 137
*
* The function will generate the default name 'BR001122334455' where the 
* numbers 0..5 represent the local MAC address
*
* RETURN: 0..ok / -1..error
*
*	The following reasons can cause an error:
*	- hIp or pLocalMac = 0
*
*/
int				ipNetbiosInit
(
 IP_STACK_H		hIp				/* handle to initialized IP stack				*/
);

/*****************************************************************************
* 
* ipNetbiosSetName - set host name
* 
* Sets the host name for the NetBIOS name service. The service will now respond
* to the initial name AND to the given name with this function
*
* RETURN: -
*
*/
void			ipNetbiosSetName
(
 char			*pName			/* ptr to host name (max 16 characters) */
);

/*****************************************************************************
* 
* ipNetbiosSetCallback - set a host name which causes a callback to the user
* 
* RETURN: -
*
*/
typedef void	NETBIOS_CBK_FCT(void);

void			ipNetbiosSetCallback
(
 char				*pName,		// ptr to host name (max 16 characters)
 NETBIOS_CBK_FCT	*pFct		// ptr to callback function to be called if name is requested
);

/*****************************************************************************
* 
* ipNetbiosGetName ... get local host name
*
* RETURN: 0..ok / -1..error
*
*/
int ipNetbiosGetName(char *pName, int nameLen);


#endif
