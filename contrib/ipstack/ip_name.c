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
 19.02.2006	enzinger	New: Request type 0x21 also answered (for some application this
								reduces the connection establishment time for ca. 2 sec)
 03.01.2007	enzinger	Response is sent to remote port, not fixed to 137

------------------------------------------------------------------------------
*/

/*****************************************************************************
DESCRIPTION

  see ip_name.h

STATE:
 stabil

*/

#include "ip.h"
#include "ip_name.h"

#include <string.h>
#include <stddef.h>

#define NETBIOS_NAME_PORT	137		// UDP port for name service
#define NETBIOS_NAME_TYPE	0x20	// standard name type with 32 bytes (max 16 characters)

#define MAX_NAME_LEN		15		// max number of characters

typedef struct
{
	// header
	unsigned short	nameTrnId;
	unsigned char	flags[2];
	unsigned short	questions;
	unsigned short	answers;
	unsigned short	authority;
	unsigned short	additional;

	// first question/answer
	unsigned char	name[34];
	unsigned short	type;
	unsigned short	cls;
	unsigned char	ttl[4];	// time to live
	unsigned short	rdLength;
	unsigned short	ownerFlags;
	unsigned short	ownerIp[2];
}netbios_typ;

static unsigned char hostName[MAX_NAME_LEN];		// host name (not coded to net format)
static unsigned char hostNameNet[MAX_NAME_LEN*2];	// host name can have maximum 16 characters

static unsigned char hostNameNet2[MAX_NAME_LEN*2];	// second host name (can be set by the user)

static unsigned char	 hostCallbackNameNet[MAX_NAME_LEN*2];	// callback will be called if this host-name is requested
static NETBIOS_CBK_FCT	*pHostCbkFct=0;					// callback function

void netbiosReceive(void *arg, ip_udp_info *pInfo)
{
	netbios_typ		*pFrame;
	unsigned short	c,cbk=0;
	unsigned char	*pChar;

	pFrame = pInfo->pData;

	// add to ARP table to avoid ARP requests (not necessary because we know the addresses already)
	ipArpUpdate(arg, pInfo->pRemoteMac , &pInfo->remoteHost);

	// check if user callback should be called
	if(pHostCbkFct)
	{
        if(memcmp(pFrame->name+1,hostCallbackNameNet,MAX_NAME_LEN*2) == 0) cbk=1;
	}

	if(pFrame->flags[0] & 0xF8)		return;	// no response if this was not an query-request
	if(pFrame->questions == 0)		return;	// no response if no questions are in the request
	if(pFrame->name[0] != 0x20)		return;	// no response if i can not interpret text format
	if(pFrame->cls != HTONS(0x01))	return;	// no response if type or class is wrong

	// the last 2 bytes must be 'AA' or 'CA' (. or blank)
	if(pFrame->name[MAX_NAME_LEN*2+2] != 'A') return;	// last must be a 'A'

	c = pFrame->name[MAX_NAME_LEN*2];
	if(c != 'A' && c != 'C') return;

	// prepare response
	pFrame->flags[0] = 0x84;	// response, authoritative answer
	pFrame->flags[1] = 0;

	pFrame->questions	= HTONS(0);
	pFrame->answers		= HTONS(1);	// 1 answer is coming
	pFrame->authority	= HTONS(0);
	pFrame->additional	= HTONS(0);

	//pInfo->remotePort	= HTONS(NETBIOS_NAME_PORT);	// respond always to netbios port

	// request type 0x21
	if( pFrame->type == HTONS(0x21) )
	{
		if ( pFrame->name[1]!=0x43 || pFrame->name[2]!=0x4B ) return;

		// create response type 0x21 (get names)
		pFrame->ttl[0] = 0;
		pFrame->ttl[1] = 0;
		pFrame->ttl[2] = 0;
		pFrame->ttl[3] = 2;

		pFrame->rdLength = 1 + 1 * (MAX_NAME_LEN+3);	// 1 byte for 'number of names' + n * (15 byte name + termination + flags + spacing)

		pChar = ((unsigned char*)&pFrame->rdLength)+2;
		*pChar++ = 1;	// number of names

		memset(pChar,' ', MAX_NAME_LEN);
		memcpy(pChar,hostName,strlen((char *)hostName));
		pChar += MAX_NAME_LEN;
		*pChar++ = 0;		// termination
		*pChar++ = 0x46;	// flags
		*pChar++ = 0;		// dummy

		ipGetAddress(arg, pChar,0);	// write mac address to unique id field after names

		pInfo->len = offsetof(netbios_typ, rdLength) + 2 + pFrame->rdLength + 6;
	}
	else if(pFrame->type == HTONS(0x20))
	{
		// compare name in question with the possible names
		if(	memcmp(pFrame->name+1, hostNameNet,MAX_NAME_LEN*2) != 0
			&&
			memcmp(pFrame->name+1, hostNameNet2,MAX_NAME_LEN*2) != 0
			&&
			cbk == 0)
		{
			return;	// no response if none of the available names matches with the name in the request
		}

		// call user callback
		if(cbk)
		{
			pHostCbkFct();
			return;
		}

		// ttl is default 300000 (0x000493E0)
		pFrame->ttl[0] = 0x00;
		pFrame->ttl[1] = 0x04;
		pFrame->ttl[2] = 0x93;
		pFrame->ttl[3] = 0xE0;

		pFrame->rdLength	= 6;
		pFrame->ownerFlags	= HTONS(0x6000);	// (so machts jedenfalls mein windows PC)

		ipGetAddress(arg, 0, pFrame->ownerIp);

		pInfo->len			= sizeof(netbios_typ);
	}
	else
	{
		return;	// unknown type
	}

	ipUdpSend(arg,pInfo);	// send reply
}

// FIRST LEVEL ENCODING of a character
unsigned char *firstLevelEncoding(unsigned char *pName, unsigned long c)
{
	if(c >= 'a' && c <= 'z') c = c-('a'-'A');	// make all letters capital

	pName[0] = (c >> 4  ) + 'A';
	pName[1] = (c & 0x0F) + 'A';
	return pName+2;
}

// hex table to convert local MAC to text
static char hexTab[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

// encode netbios name
void encodeName(unsigned char *pDst, unsigned char *pSrc)
{
	unsigned short i,c;

	for(i=0;i<MAX_NAME_LEN;i++)
	{
		if(pSrc) c = *pSrc;	// get character
		else     c = 0;		// reset name to 0

		if(c) pSrc++;	// switch to next character
		else  c=' ';	// use space if string end was reached

		pDst = firstLevelEncoding(pDst,c);
	}
}

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
 IP_STACK_H		hIp			/* handle to initialized IP stack				*/
)
{
	eth_addr		macAddr;
	unsigned char	*pCharNet,*pChar;
	unsigned char	c;
	int i;

	if(hIp==0) return -1;

	// get local MAC address
	ipGetAddress(hIp,&macAddr,0);

	// reset host names (fill with spaces)
	pCharNet = hostNameNet;
	for(i=0;i<MAX_NAME_LEN;i++) pCharNet = firstLevelEncoding(pCharNet,' ');

	memcpy(hostNameNet2,hostNameNet,sizeof(hostNameNet2));

	// generate host name "BR001122334455" (derived from local MAC)
	pChar = hostName;
	pChar[0] = 'B';
	pChar[1] = 'R';

	pCharNet = firstLevelEncoding(hostNameNet,*pChar++);
	pCharNet = firstLevelEncoding(pCharNet,   *pChar++);

	for(i=0;i<6;i++)
	{
		c = macAddr.addr[i];

		*pChar = hexTab[c >> 4];
		pCharNet = firstLevelEncoding(pCharNet, *pChar++);

		*pChar = hexTab[c & 0x0F];
		pCharNet = firstLevelEncoding(pCharNet, *pChar++);
	}

	*pChar = 0;	// termination

	// add udp receive function for port 137 to the ip stack
	i = ipUdpListen(hIp, NETBIOS_NAME_PORT, netbiosReceive, hIp);
	
	return i;	// return (-1 = Error)
}




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
)
{
	encodeName(hostNameNet2, (unsigned char*)pName);
}

/*****************************************************************************
* 
* ipNetbiosSetCallback - set callback on a certain netbios name request
* 
* RETURN: -
*
*/
void			ipNetbiosSetCallback
(
 char				*pName,		/* ptr to host name (max 16 characters) */
 NETBIOS_CBK_FCT	*pFct		// ptr to callback function to be called if name is requested
)
{
	encodeName(hostCallbackNameNet, (unsigned char*)pName);
	pHostCbkFct = pFct;
}

/*****************************************************************************
* 
* ipNetbiosGetName ... get local host name
*
* RETURN: 0..ok / -1..error
*
*/
int ipNetbiosGetName(char *pName, int nameLen)
{
	strncpy(pName, (const char*)hostName, nameLen);
	return 0;    
}
