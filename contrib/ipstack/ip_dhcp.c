/* ip_dhcp.c - DHCP client for FPGA MAC Controller*/
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
 Module:    ip_dhcp
 File:      ip_dhcp.c
 Author:    Thomas Enzinger(enzinger)
 Created:   12.03.2004
------------------------------------------------------------------------------
 History:
 25.03.2004	enzinger	stabile version
 09.04.2004 enzinger	After sending the discovery the next 4 offers will be
                        collected (max 1 second).
						The highest IP address of all collected offers will be 
						used to increase the chance to get always the same IP
						(if more dhcp servers are in the network we can not predict
						which one sends the first offer, so we wait 1 second before
						sending the request)
 28.02.2005 enzinger	Error corrected: if an 'infinite' lease was offered, the client
						made a new request every second
 07.03.2006 enzinger	DHCP_INTERVAL_MAX can be defined also in ip_opt.h
 28.06.2006	enzinger	Hostname in DHCP request included (for DHCP servers which are linked to name server)
 23.01.2007	enzinger	New function ipDhcpSetDnsName()
 24.01.2007	chkr        New function ipDhcpGetDnsName(...)
 23.05.2007	enzinger	Broadcast-Flag set (some DHCP Servers respond with unicast if this flag is not set)
 17.05.2011	enzinger	#ifdef for the case that this file is compiled, but IP_DHCP = 0
 22.06.2011	enzinger	IP-Address was not cleared, if dhcp-renew was unsuccessful (A&P 264875)
 06.09.2012 enzinger	Broadcast-Flag wird bei DHCP-RENEW nicht gesetzt (sonst Problem bei Einsatz von 
                         DHCP-Relay-Agents)
 07.09.2012 enzinger	Bei der letzten Verison habe ich leider eine Verschlimmbesserung eingebaut
                        (Fixe IP hat nicht mehr funktioniert, der Stack ist immer gleich in DHCP verfallen)

 Description:
 
   This file contains the functions ip_dhcp_handler(...) and ip_dhcp_receive(...)
   which are required by the file ip.c in case DHCP is enabled
------------------------------------------------------------------------------
*/

#include "ip.h"
#include "ip_internal.h"

#include <string.h>

#define DHCP_HOSTNAME_MAX_LEN	64
#define DHCP_PORT_CLIENT		68
#define DHCP_PORT_SERVER		67

#ifndef DHCP_INTERVAL_MAX
	#define DHCP_INTERVAL_MAX	120	// maximum discovery interval (seconds)
#endif

#define OPCODE_REQUEST			1
#define OPCODE_REPLY			2
#define ADDR_TYPE_ETH			1
#define OPT_MESSAGE_TYPE		53

#define MESSAGE_TYPE_DISCOVER	1
#define MESSAGE_TYPE_OFFER		2
#define MESSAGE_TYPE_REQUEST	3
#define MESSAGE_TYPE_ACK		5
#define MESSAGE_TYPE_NACK		6
#define MESSAGE_TYPE_INFORM		8

#define OPT_HOST_NAME			12
#define OPT_REQUEST_IP			50
#define OPT_LEASE_TIME			51
#define OPT_SERVER_IP			54
#define OPT_PAR_REQ_LIST		55
#define OPT_T1_RENEW			58
#define OPT_T2_REBIND			59
#define OPT_CLIENT_ID			61

#define PAR_ID_SUBNET			1
#define PAR_ID_ROUTERS			3
#define PAR_ID_HOSTNAME			12

#if IP_MTU < 350
	#error IP_MTU too small for DHCP
#endif

#if IP_DHCP==1
	static char hexTab[16] = "0123456789ABCDEF";	// hex table to convert local MAC to text
	static char hostName[DHCP_HOSTNAME_MAX_LEN+1];	// static var to store host name for DNS
#endif

typedef struct
{
	unsigned char	opcode;
	unsigned char	addrtype;
	unsigned char	addrlen;
	unsigned char	hops;
	unsigned short	transactionId[2];
	unsigned short	elapsedTime;
	unsigned short	flags;
	unsigned short	clientIpKnown[2];
	unsigned short	clientIpGiven[2];
	unsigned short	serverIp[2];
	unsigned short	gatewayIp[2];
	unsigned char	clientHw[16];
	unsigned char	hostName[64];
	unsigned char	bootFile[128];
	unsigned char	magicCookie[4];
}dhcp_typ;

/*********************************************************************************

  Function    : ip_dhcp_handler()
  Description : create dhcp discovery frame in first tx buffer

  Parameter:
	hIp			: handle of used interface

*********************************************************************************/
#if IP_DHCP==1

#define MAGIC_COOKIE	"\x63\x82\x53\x63"

void ip_dhcp_handler(IP_STACK_H hIpPar);

/*********************************************************************************

  Function    : ip_dhcp_receive()
  Description : DHCP frame received, parse it and send reply to offer with request

  Parameter:
	hIp			: handle of used interface

*********************************************************************************/

void ip_dhcp_receive(IP_STACK_H hIp, eth_frame *pFrame, unsigned short ipHdrLen)
{
	struct in_addr	subnet,gateway,server,given;
	unsigned char	*pData;
	unsigned short	i,len,type=0,code;
	unsigned long	leaseTime, t1=0, t2=0, lval;

	ipHdrLen =  ipHdrLen + sizeof(udp_hdr);	// add udp header to header variable

	pData = (unsigned char*)&pFrame->prot.ip + ipHdrLen;

	if(pData[0] != OPCODE_REPLY						// not a reply
		||
		pData[2] != 6								// not a 6 byte HW address
		||
		memcmp(pData+28, &hIp->local_eth_addr,6)	// compare hw address
		||
		pData[4] != hIp->state)						// compare transaction ID
	{
		return;
	}

	// get given ip address
	copy_ip_address(&given, pData + 16);

	if(given.S_un.S_addr == 0) return;		// no IP address given

	pData = pData + 236;
	if(memcmp(pData,MAGIC_COOKIE,4)) return;
	pData = pData + 4;	// access to option field

	i = htons(pFrame->prot.ip.len) - ipHdrLen - 236;
	
	while(i--)
	{
		code = pData[0];		// get option code
		if(code == 0xFF) break;	// exit loop if option end was found
		len = pData[1];			// get number of following bytes
		pData = pData + 2;		// set ptr to option data

		#if LITTLE_ENDIAN==1
			((char*)&lval)[0] = ((char*)pData)[3];
			((char*)&lval)[1] = ((char*)pData)[2];
			((char*)&lval)[2] = ((char*)pData)[1];
			((char*)&lval)[3] = ((char*)pData)[0];
		#else
			((char*)&lval)[0] = ((char*)pData)[0];
			((char*)&lval)[1] = ((char*)pData)[1];
			((char*)&lval)[2] = ((char*)pData)[2];
			((char*)&lval)[3] = ((char*)pData)[3];
		#endif

		if(code==OPT_MESSAGE_TYPE && len==1) type = pData[0];				// get message type
		if(code==OPT_SERVER_IP    && len==4) memcpy(&server,pData,4);
		if(code==PAR_ID_ROUTERS   && len>=4) memcpy(&gateway,pData,4);
		
		if(type==MESSAGE_TYPE_ACK && len==4)	// overtake subnet and gateway from ACK
		{
			if(code==PAR_ID_SUBNET)		memcpy(&subnet,pData,4);		// subnet mask
			if(code==OPT_LEASE_TIME)	leaseTime = lval;
			if(code==OPT_T1_RENEW)		t1 = lval;
			if(code==OPT_T2_REBIND)		t2 = lval;
		}

		pData = pData + len;	// set ptr to next option
	}

	if(type == MESSAGE_TYPE_NACK)
	{
		hIp->local_ip_addr.S_un.S_addr = 0;
		hIp->dhcp_given.S_un.S_addr    = 0;
		hIp->state = IP_STATE_DHCP_INIT;
	}
	// offer received
	else if(hIp->state == IP_STATE_DHCP_DISCOVER && type == MESSAGE_TYPE_OFFER)
	{
		// store given address and server address if the ip address is higher than the ones received until now
		if(given.S_un.S_addr > hIp->dhcp_given.S_un.S_addr)
		{
			hIp->dhcp_given.S_un.S_addr  = given.S_un.S_addr;
			hIp->dhcp_server.S_un.S_addr = server.S_un.S_addr;
		}
	}
	// ack received	
	else if((hIp->state == IP_STATE_DHCP_REQUEST || hIp->state == IP_STATE_DHCP_RENEWING || hIp->state == IP_STATE_DHCP_REBIND)
		&& type == MESSAGE_TYPE_ACK)
	{
		// overtake ip address, subnetmask and gateway
		hIp->local_ip_addr.S_un.S_addr = hIp->dhcp_given.S_un.S_addr;
		ipSetGateway(hIp, &subnet,&gateway);
		
		if(t1 == 0) t1 = leaseTime / 2;
		if(t2 == 0) t2 = (leaseTime / 4)*3;

		hIp->dhcp_t1	= hIp->time_s + t1;
		hIp->dhcp_t2	= hIp->time_s + t2;
		hIp->dhcp_lease	= hIp->time_s + leaseTime;

		// check for timer overflows (very large values for r1,t2 and lease (infinite))
		if(hIp->dhcp_t1    < hIp->time_s) hIp->dhcp_t1    = 0xFFFFFFFF;
		if(hIp->dhcp_t2    < hIp->time_s) hIp->dhcp_t2    = 0xFFFFFFFF;
		if(hIp->dhcp_lease < hIp->time_s) hIp->dhcp_lease = 0xFFFFFFFF;

		hIp->state		= IP_STATE_OK;
	}

}


void ip_dhcp_handler(IP_STACK_H hIpPar)
{
	ip_buf_type	*pBuf;
	IP_STACK_H	hIp		=	hIpPar;	// use local variable as ptr, faster and smaller code

	udp_hdr			*pUDP;
	char			*pData;
	unsigned long	i,len;
 
	switch(hIp->state)
	{
		case IP_STATE_DHCP_INIT:
			// convert MAC to hostname if no other hostname is set
			pData = hostName;
			if(pData[0]==0)
			{
				pData[0] = 'B';
				pData[1] = 'R';
				pData = pData + 2;

				for(i=0 ; i<12 ; i +=2 )
				{
					// get address byte and convert to 2 ascii chars
					pData[1] = hIp->local_eth_addr.addr[i/2];
					pData[0] = hexTab[pData[1] >> 4];
					pData[1] = hexTab[pData[1] & 0x0F];
					pData = pData + 2;
				}
				pData[0] = 0;	// string termination
			}

			// skip discovery if IP address is given by the user
			if(hIp->local_ip_addr.S_un.S_addr)
			{
				hIp->dhcp_lease = hIp->dhcp_t2 = hIp->dhcp_t1 = 0xFFFFFFFF;	// set renewal time to endless (136 Years) if ip is fixed
				hIp->state = IP_STATE_INIT_ARP;
				break;								
			}

			// calc start interval (a little bit random to avoid DHCP 'storm' of booting devices, at least 1 sec)
			hIp->dhcp_interval	= ((hIp->local_eth_addr.addr[4] + hIp->local_eth_addr.addr[5]) & 0x03) + 1;
			hIp->dhcp_timer		= hIp->time_s + hIp->dhcp_interval;

			hIp->dhcp_given.S_un.S_addr = 0;

			hIp->state = IP_STATE_DHCP_DISCOVER;
			break;

		case IP_STATE_DHCP_REBIND:
			// timeout check in next switch()
			break;

		case IP_STATE_DHCP_RENEWING:
			// timeout check in next switch()
			break;

		case IP_STATE_DHCP_DISCOVER:
			// at least 1 offer was received
			if(hIp->dhcp_given.S_un.S_addr != 0)
			{
				hIp->dhcp_timer		= hIp->time_s;
				hIp->state = IP_STATE_DHCP_REQUEST;
				hIp->dhcp_interval	= 1;			// if no tx buffer available: wait and try later
			}
			break;

		case IP_STATE_DHCP_REQUEST:
			// transmit will be triggered in next switch()
			break;

		case IP_STATE_OK:
			// timeout check in next switch()
			break;
	}

	// DHCP-Timeout check
	switch(hIp->state)
	{
		case IP_STATE_OK:
		case IP_STATE_DHCP_RENEWING:
		case IP_STATE_DHCP_REBIND:
			if(hIp->time_s >= hIp->dhcp_t1)
			{
				if(hIp->state != IP_STATE_DHCP_RENEWING) hIp->dhcp_interval = 1;

				hIp->state = IP_STATE_DHCP_RENEWING;
			}

			// check if renewing time is over and change to rebind (causes sending of ip broadcasts)
			if(hIp->time_s >= hIp->dhcp_t2)
			{
				hIp->state = IP_STATE_DHCP_REBIND;
			}

			if(hIp->time_s >= hIp->dhcp_lease)		// initialize dhcp if lease could not be renewed
			{
				hIp->local_ip_addr.S_un.S_addr = 0;
				hIp->state = IP_STATE_DHCP_INIT;
			}
			break;
	}

	// DHCP-Transmit
	switch(hIp->state)
	{
		case IP_STATE_DHCP_DISCOVER:
		case IP_STATE_DHCP_REQUEST:
		case IP_STATE_DHCP_RENEWING:
		case IP_STATE_DHCP_REBIND:

			// wait until timer is reached
			if(hIp->time_s < hIp->dhcp_timer) break;

			// try to get tx buffer and leave state without timer reset if no buffer was available
			pBuf = ip_alloc_tx_buffer(hIp);
			if(pBuf==0) break;

			// set next send time
			hIp->dhcp_timer = hIp->time_s + hIp->dhcp_interval;

			// double interval for next try
			hIp->dhcp_interval = hIp->dhcp_interval*2;
			// calc max dhcp interval (add last 2 bits of mac to randomize value)
			i = DHCP_INTERVAL_MAX + (hIp->local_eth_addr.addr[5]&0x03);

			if(hIp->dhcp_interval >= i) hIp->dhcp_interval = i;

			// transmit discovery / request
			pUDP  = (udp_hdr*)((char*)&pBuf->data.frame.prot.ip + sizeof(ip_hdr));
			
			pData = (((char*)pUDP) + sizeof(udp_hdr));
			
			memset(pData,0,sizeof(dhcp_typ));

			pData[0] = OPCODE_REQUEST;
			pData[1] = ADDR_TYPE_ETH;
			pData[2] = 6;
			pData[3] = 0;

			// use current state as transaction ID (1 byte used)
			pData[4] = (char)hIp->state;

			pData[8] = (char)(hIp->time_s >> 8);	// elapsed time since boot
			pData[9] = (char)(hIp->time_s & 0xFF);
			
			if(hIp->state != IP_STATE_DHCP_RENEWING)
			{
				pData[10]= 0x80;	// set broadcast flag to tell DHCP server to respond with broadcast
			}

			// set known IP-Address (in case of renew or rebind)
			memcpy(pData+12, &hIp->local_ip_addr.S_un.S_addr,4);

			memcpy(pData+28, &hIp->local_eth_addr, 6);

			pData = pData + 236;	// set ptr to magic cookie

			strcpy(pData,MAGIC_COOKIE);
	
			pData[4] = OPT_MESSAGE_TYPE;
			pData[5] = 1;
			pData = pData + 6;
			
			if(hIp->state == IP_STATE_DHCP_DISCOVER)
			{
				*(pData++) = MESSAGE_TYPE_DISCOVER;
			}
			else
			{
				*(pData++) = MESSAGE_TYPE_REQUEST;
				
				// requested IP
				pData[0] = OPT_REQUEST_IP;
				pData[1] = 4;
				memcpy(pData+2, &hIp->dhcp_given,4);
				pData = pData + 6;
			}

			if(hIp->state == IP_STATE_DHCP_RENEWING)
			{
				// add server address (only for renew)
				pData[0] = OPT_SERVER_IP;
				pData[1] = 4;
				memcpy(pData+2, &hIp->dhcp_server,4);
				pData = pData + 6;
			}

			pData[0] = OPT_CLIENT_ID;
			pData[1] = 7;
			pData[2] = ADDR_TYPE_ETH;
			pData = pData + 3;
			memcpy(pData, &hIp->local_eth_addr, 6);
			pData = pData + 6;

			pData[0] = OPT_PAR_REQ_LIST;
			pData[1] = 2;
			pData[2] = PAR_ID_SUBNET;
			pData[3] = PAR_ID_ROUTERS;
			pData = pData + 4;
			
			i = (size_t)pData - (size_t)&pBuf->data.frame.prot.ip;
			len = strlen(hostName);

			// add host name if the frame is big enough
			if( i+len+2 < IP_MTU )
			{
				pData[0] = OPT_HOST_NAME;
				pData[1] = len;
				memcpy(pData+2,hostName,len);
				pData = pData + 2 + len;
			}
			
			pData[0] = 0xFF;
			pData = pData + 1;


			// IP header
			if(hIp->state == IP_STATE_DHCP_RENEWING)
			{
				memcpy(pBuf->data.frame.prot.ip.dst_ip , &hIp->dhcp_server , 4);
			}
			else
			{
				memset(pBuf->data.frame.prot.ip.dst_ip , -1, 4);	// destination ip is 0xFFFFFFFF (broadcast)
			}

			pBuf->data.frame.prot.ip.len	= (size_t)pData - (size_t)&pBuf->data.frame.prot.ip;
			pBuf->data.frame.prot.ip.proto	= IPPROTO_UDP;

			// UDP header
			pUDP->src_port	= HTONS(DHCP_PORT_CLIENT);
			pUDP->dst_port	= HTONS(DHCP_PORT_SERVER);
			pUDP->chksum	= 0;	// no UDP checksum
			pUDP->len		= HTONS((size_t)pData-(size_t)pUDP);

			ip_buf_send(hIp, pBuf, TX_IP_HEADER);
			break;
	}

}

// set DNS name for DHCP (max 64 characters)
void ipDhcpSetDnsName(char *pHostName)
{
	strncpy(hostName,pHostName,DHCP_HOSTNAME_MAX_LEN);
}


// get DNS name (max 64 characters)
// return 0 : OK
// return SOCKET_ERROR
int ipDhcpGetDnsName(char *pName, int nameLen)
{
    if (nameLen > DHCP_HOSTNAME_MAX_LEN) return SOCKET_ERROR;
    strncpy(pName, (const char*)hostName, nameLen);
    return 0; // OK
}




#endif
