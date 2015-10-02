/* ip.c - IP Stack for FPGA MAC Controller */
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
 File:      ip.c
 Author:    Thomas Enzinger(enzingert)
 Created:   12.03.2004
------------------------------------------------------------------------------
 History:
 25.03.2004	enzinger	stabile version
 30.03.2004 enzinger	- ipGetAddress() changed, now also the eth-address
						  can be retrieved
 05.04.2004 enzinger	- minor changes in internal structures
						- ip stack instance is now allocated with malloc
						- new dependency : stdlib.c

 16.06.2004	enzinger	- Interface of some functions changed
							(data interface to ethernet driver now with packets
							instead of ptr and length)
						 --> The interface to the application has not changed 
 17.09.2004 enzinger	- new function : ipDisableInitArp()
						- error correction in icmp_in function
 13.10.2004	enzinger	- udp info type also contains the local host
 29.10.2004	enzinger	- error in ipGetAddress() corrected (returned mac address was wrong)
 02.11.2004	enzinger	- IP_ETHERTYPE... defines moved from IP.c to IP.h
 28.12.2004	enzinger	- new function : ipUdpClose()
 28.02.2005 enzinger	- Generation of second-counter improved (old version could loose some ms every
							 second, depending on the cycle time of the function calling ipPeriodic)
 14.03.2005	enzinger	- ip_alloc_tx_buffer() .. initialization of new flag (push)
 06.04.2005	enzinger	- ipUdpSend() .. ein broadcast kann auch gesendet werden, wenn der stack noch nicht READY ist
 28.07.2005	Zwi			- length of ARP response has not been set (got the length of the packet that used the same buffer before)
 10.10.2005	enzinger	- new function : ipPowerOn();
 12.10.2005	enzinger	- new fuunction: ipDhcpActivate(  );
 31.10.2005	enzinger	- ipStats can be called with hIp=0 and will return statistics of first installed instance
 07.03.2006	enzinger	- gratuitous ARP after check if MAC is duplicated to update other stations in network about my MAC address
						- check for duplicte MAC address with MAC-broadcast as source MAC in ARP request
							(to avoid that other stations overtake my MAC while searching for duplicates)
 28.04.2006	enzinger	- error at deleting arp entries caused problems (after 18 hours operation)
 09.08.2006	enzinger	- gratuitous ARP is also sent when calling ipDisableInitArp();
                        - Number of INIT-ARPs reduced to 2
 28.11.2006	enzinger	- new: htonl, ipArpQuery()
 28.11.2006	enzinger	- ipGetAddress() can be called with first parameter=0
							.. first instance will be used
 15.11.2006	enzinger	- new: ipGetState()
						- Functions and types renamed to windows standard function names
						- Support for IP Multicast added, support for DGRAM-Sockets added
 24.01.2007	enzinger	- new: ipChangeAddress()
 22.02.2007	enzinger	- TCP/UDP Packets will not be dropped anymore, if the socket is occupied with old data
							the queue processing will be stopped for the current cycle and the frame will be
							processed at the next cycle.
 04.05.2007 chkr        - New function: ipGetGateway(...)
 31.05.2007	enzinger	- ipArpQuery() does not copy MAC if pMac is 0
 31.05.2007	enzinger	- New function: ipArpRequest(...)
 13.09.2007	enzinger	- If the time given to ipPeriodic() was not 'monotonically increasing' the retransmission
						mechanism caused an error
 13.09.2007	enzinger	- New function: ipSetMtu(...)
 24.10.2007	enzinger	- Some elements of the IP-structure are allocated and not static anymore (no functional change)
 12.02.2008	enzinger	additions for IP_LOOPBACK and IP_ACTIVE_OPEN
 26.05.2008	enzinger	IRQ lock for ip_alloc_tx_buffer() to make it reentrant
 13.06.2008	enzinger	IP_LOCK_LEVEL eingeführt
 30.07.2008	enzinger	Improved handling of socket handling (for very high tcp-ip load from web-clients)
 07.10.2008	enzinger	Bei Verwendung von ipDisableInitArp() wurde trotzdem noch ein ARP rausgeschickt (unkritisch)
 16.03.2010 prenninger	move defines from ip.c and ip_internal.h to ip.h
 31.03.2010	enzinger	An den Gateway wird ein ARP gesendet, wenn in den letzten Minute kein Update empfangen wurde
 27.04.2010	enzinger	Interval des ARP an Gateway auf 10 minuten erhöht
                        ARP an Gateway wird nur durchgeführt, wenn der Gateway schon in der ARP-Tabelle ist
						(ARP ist nämlich nicht notwendig wenn noch niemand mit dem Gateway kommuniziert hat)
 04.01.2011	enzinger	support of VLAN-Tags (all Tx-Frames will be tagged with 0x8100 0000)
 09.05.2011	enzinger	user can define value for VLAN TAG by defining IP_VLAN_TAG_VALUE in ip_opt.h
						(if not defined, value 0 will be used)
 20.09.2011	enzinger	new function : ipDhcpDeactivate() ... no DHCP-Request even if IP=0.0.0.0
						(for Profinet BC)
 10.10.2011	enzinger	ipDhcpActivate() und ipDhcpDeactivate() werden nur compiliert wenn IP_DHCP = 1
						In manchen Fällen wurde IP_IN_USE anzegeigt obwohl das gar nicht der Fall war

 05.01.2012	enzinger	Correction of ARP-Probe and ARP-Announcement format according to RFC5227
 12.01.2012	enzinger	Erweiterungen für 2te IP Adresse
 07.02.2012	enzinger	Korrektur bei Auswertung von ARP-Announcements nach RFC5227
 24.02.2012	enzinger	Korrektur bei Empfang von fragmentierten IP-Frames (Wurden bei belegtem Socket verworfen)
 02.03.2012	enzinger	Korrektur für Module mit IP_LOOPBACK==1 (z.Bsp. X20BC0088)
 17.07.2012	enzinger	Korrektur von DHCP-Client (wurde am 5.1.2012 verschlimmbessert)
                        Neue Funktion ipRestartArpProbe()
 05.09.2012 enzinger	ARP-Requests wurden während DHCP RENEW und REBIND nicht beantwortet
                        (Problem bei Verbindungsaufbau, wenn sich der BC in RENEW oder REBIND befindet)
 ------------------------------------------------------------------------------
*/

/*****************************************************************************
DESCRIPTION

   IP Stack includes:
    ARP  : Request+Response
	ICMP : Ping
	UDP  : Callback Interface

  See ip.h for interface functions.

*/

#include "ip.h"
#include "ip_internal.h"
#include "ip_opt.h"

#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#ifdef IP_LOCK_LEVEL
	#define IP_LOCK_LEVEL_VAR	unsigned short	level;
	#define IP_LOCK_LEVEL_ON	level = px32intLevelLock(PX32INT_LEVEL(IP_LOCK_LEVEL));
	#define IP_LOCK_LEVEL_OFF	px32intLevelRestore(level);
#else
	#define IP_LOCK_LEVEL_VAR	;
	#define IP_LOCK_LEVEL_ON	;
	#define IP_LOCK_LEVEL_OFF	;
#endif

//check if all options are defined correctly in ip_opt.h
#ifndef IP_STATISTICS
	#error "Constant not defined in 'ip_opt.h' !"
#endif
#ifndef IP_TCP_SOCKETS
	#error "Constant not defined in 'ip_opt.h' !"
#endif
#ifndef IP_SWAP_FUNCTION
	#error "Constant not defined in 'ip_opt.h' !"
#endif
#ifndef IP_DHCP
	#error "Constant not defined in 'ip_opt.h' !"
#endif
#ifndef LITTLE_ENDIAN
	#error "Constant not defined in 'ip_opt.h' !"
#endif
#if IP_TX_BUF_CNT == 0
	#error "at least 1 tx buffer required !"
#endif

static struct IP_IF	*hIpList=0;		// ptr to first ip stack handle

// user can enter fixed value for VLAN TAG in ip_opt.h
#ifndef IP_VLAN_TAG_VALUE
	#define IP_VLAN_TAG_VALUE 0
#endif


typedef struct
{
	eth_addr		local_eth_addr;	// local ethernet address (6 Byte)
	unsigned short	state;			// stack state machine state
	struct in_addr	local_ip_addr;	// local ip addr
	unsigned long	cntRx;
	unsigned long	cntTx;
}udp_diag_data;

#define	FRM(p)		((eth_frame*)p)
#define	ARP(p)		((arp_hdr*)p)
#define ARP_TAB(p)	((arp_table_entry*)p)

#define ICMP_ECHO_REPLY 0
#define ICMP_ECHO       8     

#define ICMP_ECHO_REPLY 0
#define ICMP_ECHO       8     


#define ARP_REQUEST 1
#define ARP_REPLY   2
#define ARP_ANNOUNCEMENT   0xFFFF	// opcode is only used to mark packet interally as announcement packet
#define ARP_HWTYPE_ETH 1

#define IP_DHCP_PORT_CLIENT	68		// dhcp port

#define IP_DIAG_INFO		'i'		// info request / response

#define IP_ARP_REFRESH_S	10		// arp table refresh every 10 sec
#define IP_ARP_MAXAGE		1200	// maxium age of ARP table entries measured in seconds
									// 1200 corresponds to 20 minutes ... BSD default
#define IP_ARP_PROBE_COUNT		3		// send 3 arp requests at startup (ARP Probe as in RFC5227)
#define IP_ARP_ANNOUNCE_COUNT	2		// send 2 arp announcements (ARP Probe as in RFC5227)
#define IP_INIT_ARP_DISABLE		255


/*
*  The IP TTL (time to live) of IP packets sent
*
*  This should normally not be changed.
*/
#ifdef IP_TTL
#undef IP_TTL	// undef for win32 test (IP_TTL is already defined in ws2tcpip.h
#endif
#define IP_TTL				255

// ip_dhcp.c
extern void ip_dhcp_handler(IP_STACK_H hIp);
extern void ip_dhcp_receive(IP_STACK_H hIp, eth_frame *pFrame, unsigned short ipHdrLen);

static void	copy_eth_address(void *pDst,void *pSrc);								// copy ethernet address
static void	ip_arp_in(IP_STACK_H hIp, eth_frame *pFrame);							// ARP processing
static void ip_icmp_in(IP_STACK_H hIp, eth_frame *pFrame, unsigned short ipHdrLen);	// ICMP
static void ip_udp_in(IP_STACK_H hIp, eth_frame *pFrame, unsigned short ipHdrLen);	// UDP
static reass_buf_type *ip_reass(IP_STACK_H hIp, eth_frame *pFrame);					// reassembly
static void sendArpRequest(IP_STACK_H hIp, struct in_addr *pIp);

#if IP_TCP_SOCKETS > 0
// clear all internal variables after 'power up'
void ipPowerOn(void)
{
	hIpList = 0;
	sock_set_mtu(IP_MTU);
}
#endif


/*********************************************************************************

  Function    : ipInit
  Description : allocate and initialize ip interface structure

  Parameter:
	pEthAddr	: ptr to structure with local ethernet address
	pIpAddr		: ptr to structure with local ip address
    pEthSend    : ptr to edrv send callback

  Return:
	Handle of initialized interface
	(0=error)

*********************************************************************************/

IP_STACK_H ipInit(eth_addr *pEthAddr, struct in_addr* pIpAddr, IP_ETHSEND *pEthSend, void *hEth)
{
	IP_STACK_H	hIp;
	int			i;

	if(pEthSend==0) return 0;

	hIp = calloc(sizeof(struct IP_IF),1);	// generate handle to interface data type

	if(hIp==0) return 0;


	if(hIpList==0)	// this is the first ip stack
	{
		hIpList = hIp;
	}
	else			// add to the end of the list
	{
		while(hIpList->pNext) hIpList = hIpList->pNext;

		hIpList->pNext = hIp;
	}

	hIp->pTxBuffer = calloc(IP_TX_BUF_CNT, sizeof(ip_buf_type));
	hIp->pReassBuffer = calloc(IP_REASS_BUF_CNT, sizeof(reass_buf_type));

	// overtake local ethernet and ip address
	copy_eth_address(hIp->local_eth_addr.addr, pEthAddr);
	copy_ip_address(&hIp->local_ip_addr, pIpAddr);

	hIp->pEthSend	= pEthSend;
	hIp->hEth		= hEth;

	hIp->state = IP_STATE_INIT_ARP;

	#if IP_DHCP == 1
	{
		// start with DHCP discovery
		hIp->state = IP_STATE_DHCP_INIT;
	}
	#else
	{
		// ip address invalid
		if(pIpAddr->S_un.S_addr == 0 || pIpAddr->S_un.S_addr==0xFFFFFFFF)
		{
			hIp->state = IP_STATE_ERROR_INVALID_IP;
		}
	}
	#endif

	// initialize read queue next-pointers
	hIp->pRxRead = hIp->pRxWrite = hIp->rxQueue;	// set start pointers
	
	// set pointers of rx queue structures (point to next rx queue element)
	for(i=0;i<IP_RX_BUF_CNT;i++) hIp->rxQueue[i].pNext = hIp->rxQueue+i+1;

	// ptr in last queue element points back to first element
	hIp->rxQueue[IP_RX_BUF_CNT-1].pNext = hIp->rxQueue;

	// initialize queues for loopback interface
	#if IP_LOOPBACK==1
		// initialize read queue2 next-pointers
		hIp->pRxRead2 = hIp->pRxWrite2 = hIp->rxQueue2;	// set start pointers

		// set pointers of rx queue structures (point to next rx queue element)
		for(i=0;i<IP_TX_BUF_CNT;i++) hIp->rxQueue2[i].pNext = hIp->rxQueue2+i+1;

		// ptr in last queue element points back to first element
		hIp->rxQueue2[IP_TX_BUF_CNT-1].pNext = hIp->rxQueue2;
	#endif

	#if IP_TCP_SOCKETS > 0
		sock_set_ip(hIpList);	// add ip stack to socket driver
	#endif

	return hIp;
}

// destroy ip stack
void ipDestroy(IP_STACK_H hIp)
{
	//free memory
	free(hIp->pTxBuffer);
	free(hIp->pReassBuffer);

	//free ip handler
	free(hIp);

}

void ipRestartArpProbe(IP_STACK_H hIp)
{
	arp_table_entry	*pArpEntry;

	hIp->arp_refresh = 0;
	hIp->arp_probe   = 0;

	// remove old entries from arp table
	for(pArpEntry = hIp->arp_table ; pArpEntry < hIp->arp_table + IP_ARP_TABSIZE ; pArpEntry++)
	{
		pArpEntry->ip.S_un.S_addr = 0;
	}

	#if IP_DHCP == 1
		// start with DHCP discovery
		hIp->state = IP_STATE_DHCP_INIT;
	#else
		hIp->state = IP_STATE_INIT_ARP;
	#endif
}

// change IP address
void ipChangeAddress(IP_STACK_H hIp, struct in_addr* pIpAddr)
{
	// return here if IP was not changed
	if(pIpAddr->s_addr == hIp->local_ip_addr.s_addr) return;

	// overtake new IP and clear arp table
	hIp->local_ip_addr.s_addr = pIpAddr->s_addr;

	ipRestartArpProbe(hIp);

	#if IP_DHCP != 1
		// ip address invalid
		if(pIpAddr->S_un.S_addr == 0 || pIpAddr->S_un.S_addr==0xFFFFFFFF)
		{
			hIp->state = IP_STATE_ERROR_INVALID_IP;
		}
	#endif
}

#ifdef IP_SECONDARY_ADDRESS
void ipSetSecondaryIp(IP_STACK_H hIp, struct in_addr* pIpAddr, struct in_addr* pNetMask)
{
	if(hIp == 0) hIp = hIpList;	// use recently created IP stack if not defined by user

	if(pIpAddr)  hIp->local_ip_addr2.s_addr	= pIpAddr->s_addr;
	if(pNetMask) hIp->netmask2.s_addr       = pNetMask->s_addr;

	if(hIp->netmask2.s_addr == 0)	// invalid value, set subnet to invalid also
	{
		hIp->subnet2.s_addr = 0xFFFFFFFF;
	}
	else
	{
		hIp->subnet2.s_addr = hIp->local_ip_addr2.s_addr & hIp->netmask2.s_addr;
	}
}
#endif

#if IP_TCP_SOCKETS > 0
// set new MTU size (MTU is initialized with the value IP_MTU given in ip_opt.h, this function
//                   allows to adjust the MTU after initialization)
void ipSetMtu(unsigned short mtu)
{
	sock_set_mtu(mtu);
}
#endif

#if IP_DHCP == 1
// activate dhcp at runtime
void ipDhcpActivate(IP_STACK_H hIp)
{
	if(hIp==0) return;

	memset(&hIp->local_ip_addr, 0,4);

	hIp->state = IP_STATE_DHCP_INIT;
}

// deactivate dhcp at runtime
void ipDhcpDeactivate(IP_STACK_H hIp)
{
	if(hIp==0) return;
	hIp->dhcp_t1 = 0xFFFFFFFF;	// set renewal time to endless (136 Years)
	hIp->state = IP_STATE_OK;	// set state immediately to OK to avoid Init-ARPs (stack is absolutely 'silent')
}
#endif


/*********************************************************************************

  Function    : ipDisableInitArp
  Description : can be called after ipInit() to disable the initial arp requests

  Parameter:
	hIp			: handle of used interface

  Return:
	0  = Ok
	-1 = Error (hIp=0 or current state is not INIT_ARP)

*********************************************************************************/
int ipDisableInitArp(IP_STACK_H hIp)
{
	if(hIp==0) return -1;
	
	// tell stack that init-arp is finished and send gratuitous arp+change to OK state
	hIp->arp_probe	= IP_INIT_ARP_DISABLE;
	hIp->time_s		= 0;	// initialize time to activate own ip immediately

	return 0;
}

/*********************************************************************************

  Function    : ipSetGateway
  Description : set gateway ip address for this interface

  Parameter:
	hIp			: handle of used interface
	pGateway	: ptr to gateway ip address

*********************************************************************************/
void ipSetGateway(IP_STACK_H hIp, struct in_addr* pGateway)
{
	if(hIp==0) return;

	if(pGateway)	copy_ip_address(&hIp->gateway	, pGateway);
}

/*********************************************************************************

  Function    : ipSetNetmask
  Description : set network mask address for this interface

  Parameter:
    hIp         : handle of used interface
    pSubnetMask : ptr to subnet mask

*********************************************************************************/
void ipSetNetmask(IP_STACK_H hIp, struct in_addr* pSubnetMask)
{
    if(hIp==0) return;

    if(pSubnetMask) copy_ip_address(&hIp->netmask   , pSubnetMask);

    // generate subnet
    hIp->subnet.S_un.S_addr = hIp->local_ip_addr.S_un.S_addr & hIp->netmask.S_un.S_addr;
}

/*********************************************************************************

  Function    : ipUdpListen
  Description : listen on UDP port

  Parameter:
	hIp		: handle of used interface
	lport	: local port
	pFct	: ptr to udp callback function
	arg		: argument for callback function

  Return Value:
	0  = Ok
	-1 = Error (hIp=0 or pFct=0 or port already opened or no listen port free)
*********************************************************************************/
int ipUdpListen(IP_STACK_H hIp, unsigned long lport, IP_HOOKFCT *pFct, void *arg)
{
	listen_type		*pList, *pFree;
	unsigned long	i;

	if(hIp==0 || pFct==0) return -1;

	lport = htons((unsigned short)lport);	// port is stored in network byte order

	pList = hIp->listen_udp;
	
	pFree = 0;
	for(i = IP_LISTEN_PORTS_UDP ; i ; pList++, i--)
	{
		if(pList->lport == 0)	// take free connection if found
		{
			if(pFree==0) pFree = pList;
		}
		// check if port is already used on the system !!
		else if(pList->lport == lport)
		{
			return -1;
		}
	}

	if(pFree==0) return -1;	// no free connection available

	// overtake parameters to found connection
	pFree->lport	= lport;
	pFree->arg		= arg;
	pFree->pFct		= pFct;

	return 0;
}
						
/*********************************************************************************

  Function    : ipUdpClose
  Description : Close listening port

  Parameter:
	hIp		: handle of used interface
	lport	: local port

  Return Value:
	0  = Ok
	-1 = Error (hIp=0)
*********************************************************************************/
int ipUdpClose(IP_STACK_H hIp, unsigned long lport)
{
	listen_type		*pList;
	unsigned long	i;

	if(hIp==0) return -1;

	lport = htons((unsigned short)lport);	// port is stored in network byte order

	pList = hIp->listen_udp;
	
	for(i = IP_LISTEN_PORTS_UDP ; i ; pList++, i--)
	{
		// clear all listening entries to this port
		if(pList->lport == lport) pList->lport = 0;
	}

	return 0;
}


#if IP_STATISTICS == 1
	//-------------------- get address of statistic structure --------------------
	ip_stat* ipStats(IP_STACK_H hIp)
	{
		if(hIp==0) hIp = hIpList;	// take first instance if handle is 0

		if(hIp==0) return 0;		// not even 1 instance installed !

		return &hIp->stat;
	}
#endif

/*********************************************************************************

  Function    : ip_packet_free
  Description : release a buffer which is not used anymore

  Parameter:
	pBuffer	: ptr of buffer with data

*********************************************************************************/
static void ip_packet_free(ip_packet_typ *pPacket)
{
	ip_buf_hdr		*pHdr = &((ip_buf_type*)(((char*)pPacket)-sizeof(ip_buf_hdr)))->header;

	if(pPacket==0) return;

	EnableGlobalInterrupt(FALSE);

	switch(pHdr->state)
	{
		case IP_BUF_STATE_TX_ACK_Q:	// sender wants an ack but no change to idle state
			pHdr->state = IP_BUF_STATE_TX_DONE;	// the sender has to release the buffer
			break;

		case IP_BUF_STATE_TX_Q:
		case IP_BUF_STATE_RX:		// reassembled buffers (due to fragmentation) are marked as RX
			pHdr->push	= 0;
			pHdr->state	= IP_BUF_STATE_IDLE;
			break;

		default:	// should never happen !
			pHdr->push	= 0;
			pHdr->state	= IP_BUF_STATE_IDLE;
			break;
	}

	EnableGlobalInterrupt(TRUE);
}

// get IP address and MAC address
void ipGetAddress(IP_STACK_H hIp, void *pMacAddr, void *pIpAddr)
{
	if(hIp==0) hIp = hIpList;	// take first instance if handle is 0

	if(pMacAddr) memcpy(pMacAddr, &hIp->local_eth_addr , 6);
	if(pIpAddr)  memcpy(pIpAddr,  &hIp->local_ip_addr  , 4);
}


// get SubnetMask and Gateway
void ipGetGateway(IP_STACK_H hIp, void *pSubnetMask, void *pGateway)
{
    if(hIp==0) hIp = hIpList;	// take first instance if handle is 0

    if(pSubnetMask) memcpy(pSubnetMask, &hIp->netmask  , 4);
    if(pGateway)    memcpy(pGateway,    &hIp->gateway  , 4);
}

static int processPacket(IP_STACK_H hIp, ip_packet_typ *pPacket)
{
	unsigned short	len;
	eth_frame		*pFrame = (eth_frame*)&pPacket->data;

	#if IP_REASS_BUF_CNT > 0
		reass_buf_type	*pReass=0;
	#endif

	// processing of new frame
	switch(pFrame->eth.type)
	{
		case HTONS(IP_ETHTYPE_ARP):
			ip_arp_in(hIp, pFrame);		// ARP frame received (request or response)
			break;

		case HTONS(IP_ETHTYPE_IP):
			IP_STAT( hIp->stat.prot_ip++ );

			// Check validity of the IP header (header with options is not accepted)
			if((pFrame->prot.ip.vhl >> 4) != IP_VERSION_V4)
			{
				IP_STAT( hIp->stat.ip_err_version++ );
				IP_LOG("ip: invalid version or header length.");
				break;
			}

			// Compute and check the IP header checksum
			if(ip_chksum(&pFrame->prot.ip, 0))
			{
				IP_STAT( hIp->stat.ip_err_chksum++);
				IP_LOG("ip: bad checksum.");
				break;
			}

			// Check the fragment flag
			if(pFrame->prot.ip.ipoffset & HTONS(0x3FFF))
			{
				#if IP_REASS_BUF_CNT > 0
					// call reassembly function only if reassembly is enabled
					pReass = ip_reass(hIp,pFrame);

					// leave loop if no reassembled buffer was completed
					if(pReass == 0) break;

					pFrame = &pReass->buf.data.frame;	// new frame pointer if reassembled buffer is ready

					// store callback function inside the frame (first 4 bytes of dst-mac)
					((ip_int_hdr*)pFrame)->pFct = ip_packet_free;
				#else
					IP_STAT( hIp->stat.ip_err_frag++ );
					IP_LOG("ip: fragment dropped.");
					break;
				#endif
			}

			ip_arp_in(hIp, pFrame);		// IP frame received -> update ARP table

			// ip header length
			len = (pFrame->prot.ip.vhl & 0xF)*4;

			switch(pFrame->prot.ip.proto)
			{
				#if IP_TCP_SOCKETS > 0
					case IPPROTO_TCP:
						if( sock_in(hIp, pFrame, len) == IP_FRAME_RETRY )
						{
							if(pReass) pReass->timer = IP_REASS_MAXAGE;
							return -1;
						}
						break;
				#endif

				case IPPROTO_UDP:
					// add source-ip to secondary-ip-remotes if the remote has addressed my secondary io
					#ifdef IP_SECONDARY_ADDRESS
						// packet was sent to my secondary IP
						if( memcmp(&hIp->local_ip_addr2, pFrame->prot.ip.dst_ip,4) == 0 ) 
						{
							int ixHost;

							// search in lisst
							for(ixHost=0;ixHost<IP_SECONARY_HOST_QUEUE;ixHost++)
							{
								if( memcmp(hIp->ip2_remotehosts+ixHost, pFrame->prot.ip.src_ip,4) == 0 ) break;	// source-ip found
							}

							// make entry, if not yet there
							if(ixHost == IP_SECONARY_HOST_QUEUE)
							{
								memcpy( hIp->ip2_remotehosts + hIp->ip2_remotehost_index, pFrame->prot.ip.src_ip, 4);
								hIp->ip2_remotehost_index++;
								if(hIp->ip2_remotehost_index >= IP_SECONARY_HOST_QUEUE) hIp->ip2_remotehost_index = 0;
							}
						}
					#endif

					#if IP_TCP_SOCKETS > 0
						if( sock_in(hIp, pFrame, len) == IP_FRAME_RETRY )
						{
							if(pReass) pReass->timer = IP_REASS_MAXAGE;
							return -1;
						}
					#endif

					ip_udp_in(hIp, pFrame, len);
					break;

				case IPPROTO_ICMP:
					ip_icmp_in(hIp, pFrame, len);
					break;

				default:
					IP_STAT(hIp->stat.ip_err_proto++);
					IP_LOG("ip: unknown protocol");
					break;
			}

			break;
	}

	#if IP_REASS_BUF_CNT > 0
		{
			if(pReass)	// reassembled buffer was created, release if not used anymore
			{
				if(((ip_int_hdr*)pFrame)->pFct) ((ip_int_hdr*)pFrame)->pFct((ip_packet_typ*)&pReass->buf.length);
			}
		}
	#endif

	// free buffer and clear rx-queue entry ...  pass buffer to back ethernet driver
	pFrame = (eth_frame*)pPacket->data;
	if(((ip_int_hdr*)pFrame)->pFct) ((ip_int_hdr*)pFrame)->pFct(pPacket);

	return 0;
}


// periodic call of ip stack
ipState_enum ipPeriodic(IP_STACK_H hIp, unsigned long timeMs)
{
	ip_rx_queue_typ	*pQueue;
	ip_buf_type		*pBuf;
	unsigned short	i,len;

	if (hIp==0) return 0xFFFF;

	// send all packets which were not sent already at input processing
	#if IP_TCP_SOCKETS > 0
		sock_out(hIp);
	#endif

	#ifdef DEBUG
		for(i=0, pBuf = hIp->pTxBuffer ; pBuf < hIp->pTxBuffer + IP_TX_BUF_CNT ; i++, pBuf++)
		{
			hIp->dbgBufTxState[i] = '0' + pBuf->header.state;
			hIp->pDbgTxBuf[i] = pBuf;
		}
	#endif

	// process rx and tx queue
	for(i=0;i < IP_RX_BUF_CNT;i++)	// maximum number of processed frames in one cycle : len of rx queue
	{
		// try to output pending tx frames
		while(1)
		{
			pBuf = hIp->txQueue[hIp->txQRead];	// get address of next tx buffer

			if(pBuf==0) break;					// no tx buffer available
			
			// try to send buffer to ethernet driver
			len = hIp->pEthSend(hIp->hEth, (ip_packet_typ*)&pBuf->length, ip_packet_free);

			if(len)	// send successful, clear entry and switch to next buffer in queue
			{
				hIp->stat.txPackets++;				// count sent packets

				hIp->txQueue[hIp->txQRead++] = 0;	// txbuffer was read by the ethernet driver
		
				if(hIp->txQRead >= IP_TX_BUF_CNT) hIp->txQRead = 0;	// increment read index
			}
			else
			{
				// ethernet driver queue full, try next time
				hIp->stat.ethSendOverflow++;
				break;
			}
		}

		pQueue = hIp->pRxRead;		// get ptr to current queue element

		if(pQueue->pPacket == 0)
		{
			// also check 2nd queue (internal loopback) if standard rx-queue does not contain data
			#if IP_LOOPBACK==1
				pQueue = hIp->pRxRead2;
				if(pQueue->pPacket)
				{
					// process rx packet
					// stop processing frames if the current frame is denied (because socket is occupied)
					if( processPacket(hIp, pQueue->pPacket ) ) break;

					pQueue->pPacket	= 0;				// mark as empty
					hIp->pRxRead2	= pQueue->pNext;	// switch read index to next buffer
					continue;				
				}
			#endif

			break;		// stop loop, no buffer available
		}

		// process rx packet
		// stop processing frames if the current frame is denied (because socket is occupied)
		if( processPacket(hIp, pQueue->pPacket ) ) break;

		pQueue->pPacket = 0;			// mark as empty
		hIp->pRxRead = pQueue->pNext;	// switch read index to next buffer
	}

	if(hIp->time_s == 0)	// initialization
	{
		hIp->time_s = 1;
		hIp->time_ms_old = timeMs-1000;	// initialize time_ms_old
	}

	// every second ... ARP refresh and periodic handling of IP
	if( ((signed long)(timeMs - hIp->time_ms_old)) < 1000 ) return hIp->state;

	hIp->time_ms_old += 1000;
	hIp->time_s++;			// free running second counter

	#if IP_DHCP == 1
		ip_dhcp_handler(hIp);
	#endif

	//-------------------------------- Periodic IP ---------------------------------------

	// decrease reassembly timers
	#if IP_REASS_BUF_CNT > 0
	{
		reass_buf_type *pReass;

		// loop through reassembly buffers and decrement timer (buffers with timer=0) will be deleted if the 
		// memory is required
		for(pReass = hIp->pReassBuffer ; pReass < hIp->pReassBuffer + IP_REASS_BUF_CNT; pReass++)
		{
			if(pReass->timer) pReass->timer--;
		}
	}
	#endif
	
	#if IP_TCP_SOCKETS > 0
		sock_periodic(hIp);	// cyclic socket handling
	#endif

	i = 0;

	if(hIp->state==IP_STATE_INIT_ARP)
	{
		if(hIp->arp_probe > IP_ARP_PROBE_COUNT)
		{
			hIp->state = IP_STATE_OK;	// init-arp disabled or finished
		}
		else if(hIp->arp_probe < IP_ARP_PROBE_COUNT)	// send probes
		{
			i = TX_IP_HEADER | TX_ARP_PROBE;
		}
		else	// last one is a announcement
		{
			i = TX_IP_HEADER | TX_ARP_ANNOUNCEMENT;
			hIp->state = IP_STATE_OK;	// init-arp disabled or finished
		}
	}
	else if ( (hIp->state==IP_STATE_OK) && (hIp->arp_probe == (IP_ARP_PROBE_COUNT+1)) )
	{
		i = TX_IP_HEADER | TX_ARP_ANNOUNCEMENT;	// one more announcement when stat already ok (RFC5227)
	}

	if(i)
	{
		pBuf = ip_alloc_tx_buffer(hIp);	// get tx buffer
		if(pBuf)	// send arp request to myself
		{
			copy_ip_address(pBuf->data.frame.prot.ip.dst_ip, &hIp->local_ip_addr);
			ip_buf_send(hIp, pBuf, i);
			hIp->arp_probe++;
		}
	}

	if(hIp->state == IP_STATE_INIT_ARP) return IP_STATE_INIT_ARP;	// init-arp not yet finished

	//-------------------------------- ARP Refresh ---------------------------------------
	hIp->arp_refresh++;	// increase refresh counter


	// refresh time reached, go through list and remove entries which have reached IP_ARP_MAXAGE
	if(hIp->arp_refresh >= IP_ARP_REFRESH_S )
	{
		arp_table_entry	*pArpEntry, *pArpGateway = 0;

		hIp->arp_refresh = 0;

		// remove old entries from arp table
		for(pArpEntry = hIp->arp_table ; pArpEntry < hIp->arp_table + IP_ARP_TABSIZE ; pArpEntry++)
		{
			if(pArpEntry->ip.S_un.S_addr)
			{
				if (((unsigned short)hIp->time_s - pArpEntry->time) >= IP_ARP_MAXAGE)	// remove entry if too old
				{
					pArpEntry->ip.S_un.S_addr = 0;
				}
				else if( pArpEntry->ip.S_un.S_addr == hIp->gateway.S_un.S_addr)		// store ptr to gateway entry
				{
					pArpGateway = pArpEntry;
				}

			}
		}

		// request mac address of gateway (if configured, and last reception was more than 10 minutes ago)
		if(hIp->gateway.S_un.S_addr)
		{
			if(pArpGateway!=0 && ((unsigned short)hIp->time_s - pArpGateway->time) >= 600 )
			{
				sendArpRequest(hIp, &hIp->gateway);
			}
		}
	}

	return hIp->state;
}

static void prepareArpReq(IP_STACK_H hIp, ip_buf_type *pBuf, struct in_addr *pIpAddr, unsigned long option)
{
	arp_hdr *pArp = &((eth_frame*)&pBuf->data.frame)->prot.arp;

	IP_STAT( hIp->stat.arp_req_tx++ );

	// ethernet header
#ifdef IP_VLAN_TAG
	pBuf->data.frame.eth.vlanTag[0] = HTONS(0x8100);
	pBuf->data.frame.eth.vlanTag[1] = IP_VLAN_TAG_VALUE;
#endif

	pBuf->data.frame.eth.type = HTONS(IP_ETHTYPE_ARP);
	memset(pBuf->data.frame.eth.dst_hw,	-1,	6);			// ethernet broadcast
	copy_eth_address(pBuf->data.frame.eth.src_hw,	hIp->local_eth_addr.addr);

	// ARP header
	memset(pArp->dst_hw, 0, 6);
	copy_eth_address(pArp->src_hw,	hIp->local_eth_addr.addr);
	copy_ip_address(pArp->dst_ip,	pIpAddr);
	copy_ip_address(pArp->src_ip,	&hIp->local_ip_addr);

	// replace own IP address with 0 if searching for duplicate address (ARP Probe as in RFC5227)
	// (to avoid that other stations overtake the MAC address in their ARP tables)
	if(option & TX_ARP_PROBE) memset(&pArp->src_ip, 0,4);

	pArp->opcode	= HTONS(ARP_REQUEST);		// ARP request
	pArp->hwtype	= HTONS(ARP_HWTYPE_ETH);
	pArp->protocol	= HTONS(IP_ETHTYPE_IP);
	pArp->hwlen		= 6;
	pArp->protolen	= 4;

	// send packet with request to to network
	pBuf->length = sizeof(eth_hdr)+sizeof(arp_hdr);
}

void ip_buf_send(IP_STACK_H hIp, ip_buf_type *pBuf, unsigned long option)
{
	void			*ptr;		// multi purpose pointer
	eth_frame		*pFrame;
	struct in_addr	ipAddr;

	#if IP_TCP_SOCKETS > 0
		tcp_hdr			*pTCP;
	#endif


	pFrame	= &pBuf->data.frame;
	ptr		= &pFrame->prot.ip;

	// always copy local hw address to eth header
	copy_eth_address(pFrame->eth.src_hw,	hIp->local_eth_addr.addr);

	// mark buffer as TX if not already set to TX_ACK by another instance
	if(pBuf->header.state != IP_BUF_STATE_TX_ACK) pBuf->header.state = IP_BUF_STATE_TX;

	// do not modify the ip header for a arp-reply (option=0)
	if (option)
	{
#ifdef IP_VLAN_TAG
		pBuf->data.frame.eth.vlanTag[0] = HTONS(0x8100);
		pBuf->data.frame.eth.vlanTag[1] = IP_VLAN_TAG_VALUE;
#endif
		pFrame->eth.type = HTONS(IP_ETHTYPE_IP);

		// TX_IP_REPLY : reply to sender ip address
		if(option & TX_IP_REPLY)
		{
			copy_ip_address(&ipAddr, IP(ptr)->dst_ip);
			copy_ip_address(IP(ptr)->dst_ip, IP(ptr)->src_ip);
			copy_ip_address(IP(ptr)->src_ip, &ipAddr);
		}
		else
		{
#ifdef IP_SECONDARY_ADDRESS
			if(option & TX_SECONDARY_IP)
			{
				copy_ip_address(IP(ptr)->src_ip, &hIp->local_ip_addr2);	// always use local ip as source ip
			}
			else
			{
				copy_ip_address(IP(ptr)->src_ip, &hIp->local_ip_addr);	// always use local ip as source ip
			}
#else
			copy_ip_address(IP(ptr)->src_ip, &hIp->local_ip_addr);	// always use local ip as source ip
#endif
		}

		// create ip header (except the length, this field must be already valid)
		if(option & TX_IP_HEADER)
		{
			// set output length for ethernet driver
			pBuf->length = IP(ptr)->len + sizeof(eth_hdr);

			// set the vhl only if this is not a direct reply to a ip packet
			// (in case of a direct reply we leave the field as it was, maybe with options)
			if((option & TX_IP_REPLY) == 0)	IP(ptr)->vhl = (IP_VERSION_V4 << 4) + (sizeof(ip_hdr)/4);
			
			IP(ptr)->len		= htons(IP(ptr)->len);			// convert length to network byte order
			IP(ptr)->tos		= 0;
			IP(ptr)->ipoffset	= 0;
			IP(ptr)->ttl		= IP_TTL;
			IP(ptr)->ipid		= htons(hIp->ipid++);
			IP(ptr)->chksum		= 0;
			IP(ptr)->chksum		= ip_chksum(ptr, 0);	// Calculate IP header checksum
		}

		#if IP_TCP_SOCKETS > 0
			if(option & TX_IP_TCP_CHKSUM)
			{
				pTCP = (tcp_hdr*)((char*)ptr + sizeof(ip_hdr));
				pTCP->chksum = 0;
				pTCP->chksum = ip_chksum(ptr, IPPROTO_TCP);

				IP_STAT(hIp->stat.tcp_tx++);
			}
		#endif

		// Find the destination IP address in the ARP table and construct the Ethernet header.
		// If the destination IP address isn't on the local network, we use the default router's IP address instead
		// If not ARP table entry is found, we overwrite the original IP packet with an ARP request for the IP address

		// get ip address from frame by word (not long aligned in ethernet frame)
		copy_ip_address(&ipAddr, IP(ptr)->dst_ip);

		// check for IP multicast
		if(ipAddr.S_un.S_un_b.s_b1 >= 224 && ipAddr.S_un.S_un_b.s_b1 <= 239)
		{
            // generate correct MAC address to this multicast address
			pFrame->eth.dst_hw[0] = HTONS(0x0100);
			pFrame->eth.dst_hw[1] = htons(0x5E00 | (ipAddr.S_un.S_un_w.s_w1 & 0x7F));
			pFrame->eth.dst_hw[2] = ipAddr.S_un.S_un_w.s_w2;
		}
		#if IP_LOOPBACK==1
		// check for loopback address (but not when state is still init-arp, at this state the stack transmits packets to its own
		else if
			(
				((option & (TX_ARP_ANNOUNCEMENT|TX_ARP_PROBE))==0)
				&&
				(ipAddr.S_un.S_un_b.s_b1==127 || ipAddr.S_un.S_addr==hIp->local_ip_addr.S_un.S_addr)
			)
		{
			// add to receive queue and increment rx buf write index
			ip_rx_queue_typ	*pQueue;

			IP_LOCK_LEVEL_ON

			pQueue = hIp->pRxWrite2;

			if(pBuf->header.state == IP_BUF_STATE_TX)		pBuf->header.state = IP_BUF_STATE_TX_Q;
			if(pBuf->header.state == IP_BUF_STATE_TX_ACK)	pBuf->header.state = IP_BUF_STATE_TX_ACK_Q;

			if(pQueue->pPacket==0)	// queue entry free
			{
				IP_STAT(hIp->stat.packets_used++);	// count incoming packets

				// store callback function inside the frame (first 4 bytes of dst-mac)
				((ip_int_hdr*)pFrame)->pFct = ip_packet_free;


				pQueue->pPacket	= (ip_packet_typ*)&pBuf->length;		// write info to queue

				hIp->pRxWrite2 = pQueue->pNext;	// set to next queue element
			}
			else
			{
				IP_STAT(hIp->stat.loopBackDropped++);
				ip_packet_free((ip_packet_typ*)&pBuf->length);
			}

			IP_LOCK_LEVEL_OFF

			return;
		}
		#endif
		else if((ipAddr.S_un.S_addr | hIp->netmask.S_un.S_addr) != 0xFFFFFFFF)	// do not access arp table for ip broadcasts
		{
			// check if the destination address is on the local network
#ifdef IP_SECONDARY_ADDRESS
			if(
				((ipAddr.S_un.S_addr & hIp->netmask.S_un.S_addr) != hIp->subnet.S_un.S_addr)
				&&
				((ipAddr.S_un.S_addr & hIp->netmask2.S_un.S_addr) != hIp->subnet2.S_un.S_addr)
			  )
#else
			if((ipAddr.S_un.S_addr & hIp->netmask.S_un.S_addr) != hIp->subnet.S_un.S_addr)
#endif
			{
				// Destination address was not on the local network, so we need to
				// use the default router's IP address instead of the destination
				// address when determining the MAC address
				ipAddr.S_un.S_addr = hIp->gateway.S_un.S_addr;
			}

			for(ptr = hIp->arp_table ; ARP_TAB(ptr) < hIp->arp_table + IP_ARP_TABSIZE ; ptr = (char*)ptr + sizeof(arp_table_entry))
			{
				if(ipAddr.S_un.S_addr == ARP_TAB(ptr)->ip.S_un.S_addr)
				{
					// build an ethernet header
					copy_eth_address(pFrame->eth.dst_hw, ARP_TAB(ptr)->eth.addr);
					break;
				}
			}

			// change buffer to arp request if entry in arp table not found (or frame is addressed to local ip)
			if(ARP_TAB(ptr)== hIp->arp_table + IP_ARP_TABSIZE || ipAddr.S_un.S_addr==hIp->local_ip_addr.S_un.S_addr)
			{
				// todo
				prepareArpReq(hIp, pBuf, &ipAddr, option);
			}
		}
		else
		{
			memset(pFrame->eth.dst_hw, -1, 6);	// generate ethernet broadcast
		}
	}


	// add to tx queue (no overflow check necessary because queue length and available buffers is same
	EnableGlobalInterrupt(FALSE);

	// only buffers with state  IP_BUF_STATE_TX or IP_BUF_STATE_TX_ACK will be sent to tx-queue
	switch(pBuf->header.state)
	{
		case IP_BUF_STATE_TX:
			pBuf->header.state = IP_BUF_STATE_TX_Q;
			break;

		case IP_BUF_STATE_TX_ACK:
			pBuf->header.state = IP_BUF_STATE_TX_ACK_Q;
			break;

		default:	// other buffer states will not be placed to TX-queue
		    EnableGlobalInterrupt(TRUE);
			return;
	}

	hIp->txQueue[hIp->txQWrite++] = pBuf;					// add to send queue
	if(hIp->txQWrite >= IP_TX_BUF_CNT) hIp->txQWrite = 0;	// increment write index

	EnableGlobalInterrupt(TRUE);
}


/*********************************************************************************

  Function    : ip_alloc_tx_buffer
  Description : allocates a buffer of the tx buffers

  Parameter:
	hIp		: handle of used interface

  Return:
	Address of a free tx buffer

*********************************************************************************/
ip_buf_type* ip_alloc_tx_buffer(IP_STACK_H hIp)
{
	ip_buf_type		*pBuf;
	
	IP_LOCK_LEVEL_VAR

	for(pBuf = hIp->pTxBuffer ; pBuf < hIp->pTxBuffer + IP_TX_BUF_CNT ; pBuf++)
	{
		IP_LOCK_LEVEL_ON

		if(pBuf->header.state != IP_BUF_STATE_IDLE)
		{
			IP_LOCK_LEVEL_OFF
			continue;
		}

		pBuf->header.state		= IP_BUF_STATE_TX;
		pBuf->header.dataSize	= 0;
		pBuf->header.push		= 0;

		IP_LOCK_LEVEL_OFF

		return pBuf;
	}

	hIp->stat.txBufferFull++;

	return 0;	// no free buffer found
}


/*********************************************************************************

  Function    : ip_chksum
  Description : calculate checksum of IP header, UDP or TCP frame

  Parameter:
	pIP		: ptr to IP header
	prot	: IPPROTO_IP (0) , IPPROTO_TCP or IPPROTO_UDP

  Return Value when testing incoming packets:
	The value must be 0, otherwise the frame is damaged
	UDP:
	Checksum is optional, this checksum function does not consider this,
	if the chksum field of the UDP header is 0 this function should not be called

  Return Value when generating outbound traffic:
	The value must be saved to the respective 'chksum' field of the header
	UDP:
	If this function returns 0 the application has to write 0xFFFF to the UDP header
	

*********************************************************************************/

unsigned short ip_chksum(ip_hdr *pIP, unsigned long prot)
{
	unsigned long len;
	unsigned long offset;
	unsigned long temp;

	len = (pIP->vhl & 0xF)*2;	// ip header size in words

	if(prot)	// calculate checksum of payload data if another checksum than IP is reqested
	{
		offset	= len+len;		// the header length is the offset to the payload data
		len		= pIP->len;	// get length from ip-header

		// swap if host is little endian
		#if LITTLE_ENDIAN==1
			((char*)&temp)[0] = (char)(len>>8);	// high byte -> low address (big endian)
			((char*)&temp)[1] = (char)len;		// low byte -> high address (big endian)
			len = (unsigned short)temp;
		#endif

		len		= len - offset;	// take ip header length off
		prot	= prot + len;	// first part of pseudo header: protocol number + payload length
	
		#if LITTLE_ENDIAN==1		// swap first part of pseudo header on LE systems
			((char*)&temp)[0] = (char)(prot>>8);
			((char*)&temp)[1] = (char)prot;
			prot = (unsigned short)temp;
		#endif

		// rest of pseudo header (src and dst ip)
		prot = prot + pIP->src_ip[0] + pIP->src_ip[1] + pIP->dst_ip[0] + pIP->dst_ip[1];

		pIP = (ip_hdr*)(((char*)pIP) + offset);	// set ptr to start of payload data

		// odd length, set 1 byte after the frame to 0 because it will be included in the chksum calculation
		if(len & 1) ((char*)pIP)[len++] = 0;
		
		len=len/2;	// generate word counter
	}
	
	while(len)	// checksum calculation
	{
		prot = prot + *(unsigned short*)pIP;	// get word from frame and add to checksum
		pIP  = (ip_hdr*)(((char*)pIP)+2);		// switch to next word
		len--;
	}
	
	// add carries from high word to low word
	do
	{
		temp = prot;

		len  = ((temp & 0xFFFF0000) >> 16);
		temp = temp & 0xFFFF;

		prot = temp + len;
	}
	while(len);

	return ~(unsigned short)prot;
}

//-----------------------------------------------------------------------------
static void copy_eth_address(void *pDst,void *pSrc)
{
	((short*)pDst)[0] = ((short*)pSrc)[0];
	((short*)pDst)[1] = ((short*)pSrc)[1];
	((short*)pDst)[2] = ((short*)pSrc)[2];
}

//-----------------------------------------------------------------------------

void*				ipArpAnnouncement		// update arp table (only if entry is already there)
(
 IP_STACK_H		hIp,			// handle to IP stack
 void			*pMacAddr,		// ptr to remote mac address
 void			*pIpAddr		// ip address of remote host
)
{
	arp_table_entry *pTab,*pFirstFree,*pMaxAge = NULL;		// also used for ARP header in frame
	struct in_addr	ipAddr;
	unsigned int	maxAge,tmpAge;

	copy_ip_address(&ipAddr , pIpAddr);

	// Walk through the ARP mapping table and try to find an entry to update.
	// If none is found, return the first free entry, or otherwise the oldest entry

	pFirstFree	= 0;
	maxAge		= 0;
	for(pTab = hIp->arp_table ; pTab < hIp->arp_table + IP_ARP_TABSIZE ; pTab++)
	{
		// Check if the source IP address of the incoming packet matches
		// the IP address in this ARP table entry
		if(ipAddr.S_un.S_addr == pTab->ip.S_un.S_addr)
		{
			// entry found, update
			copy_eth_address(pTab->eth.addr, pMacAddr);
			pTab->time = (unsigned short)hIp->time_s;
			return 0;	// return 0 = entry was done
		}

		// find the oldest entry
		tmpAge = hIp->time_s - pTab->time;
		if(tmpAge >= maxAge)
		{
			maxAge	= tmpAge;
			pMaxAge	= pTab;
		}

		// try to find the first unused entry in the ARP table
		if(pTab->ip.S_un.S_addr == 0 && pFirstFree==0) pFirstFree = pTab;
	}

	if(pFirstFree) return pFirstFree;

	return pMaxAge;
}

void				ipArpUpdate		// update arp table
(
	IP_STACK_H		hIp,			// handle to IP stack
	void			*pMacAddr,		// ptr to remote mac address
	void			*pIpAddr		// ip address of remote host
)
{
	arp_table_entry *pTab;
	struct in_addr	ipAddr;
	int	local;

	copy_ip_address(&ipAddr , pIpAddr);

	// do not take stations which are not in local subnet (to avoid that arp table is flooded with external addresses all having the same mac)
	local = 0;

	if((ipAddr.S_un.S_addr & hIp->netmask.S_un.S_addr) == hIp->subnet.S_un.S_addr) local=1;	// in local subnet ... enter in ARP-Table

#ifdef IP_SECONDARY_ADDRESS
	if((ipAddr.S_un.S_addr & hIp->netmask2.S_un.S_addr) == hIp->subnet2.S_un.S_addr) local=1;	// in local subnet ... enter in ARP-Table
#endif

	if(local == 0) return;		// frame not in local subnet

	// add mac-address to table ... and get free entry if ip was not yet there
	pTab = ipArpAnnouncement(hIp,pMacAddr,pIpAddr);

	if(pTab==0) return;	// entry already updated

	// Now, pNew the ARP table entry which we will fill with the new information
	copy_eth_address(pTab->eth.addr, pMacAddr);

	pTab->ip.S_un.S_addr	= ipAddr.S_un.S_addr;
	pTab->time				= (unsigned short)hIp->time_s;
}

// get MAC address address of specified IP address
// return 0 : OK
// return SOCKET_ERROR
int ipArpQuery(unsigned long ip, void *pMac)
{
	IP_STACK_H		hIp;
	arp_table_entry *pTab;

	// search through all IP stacks
	for( hIp = hIpList ; hIp != 0 ; hIp = hIp->pNext)
	{
		// check if this is the local IP of this instance
		if(hIp->local_ip_addr.S_un.S_addr == ip)
		{
			if(pMac) copy_eth_address(pMac, &hIp->local_eth_addr);
			return 0;
		}

        // check ARP list
		for(pTab = hIp->arp_table ; pTab < hIp->arp_table + IP_ARP_TABSIZE ; pTab++)
		{
			// Check if the source IP address of the incoming packet matches
			// the IP address in this ARP table entry
			
			if(pTab->ip.S_un.S_addr != ip) continue;		// try next

			if(pMac) copy_eth_address(pMac, &pTab->eth);	// copy address to user var

			// test again (to make sure to get consistent data because IRQ may modify table entries)
			if(pTab->ip.S_un.S_addr != ip) continue;

			return 0;
		}
	}

	return SOCKET_ERROR;
}

static void sendArpRequest(IP_STACK_H hIp, struct in_addr *pIp)
{
	ip_buf_type *pBuf = ip_alloc_tx_buffer(hIp);

	IP_LOCK_LEVEL_VAR

	if(pBuf==0) return;

	prepareArpReq(hIp, pBuf, pIp, 0);

	// add to tx queue (no overflow check necessary because queue length and available buffers is same
	IP_LOCK_LEVEL_ON
	hIp->txQueue[hIp->txQWrite++] = pBuf;					// add to send queue
	if(hIp->txQWrite >= IP_TX_BUF_CNT) hIp->txQWrite = 0;	// increment write index
	IP_LOCK_LEVEL_OFF
}

// get MAC address address of specified IP address
// (send ARP request if address is not available)
// return 0 : OK
// return SOCKET_ERROR
int ipArpRequest(unsigned long ip, void *pMac)
{
	int				retVal = ipArpQuery(ip,pMac);
	IP_STACK_H		hIp;

	if(retVal == 0) return 0;

	// send arp request if IP is in local subnet
	for( hIp = hIpList ; hIp != 0 ; hIp = hIp->pNext)
	{
		if ((hIp->local_ip_addr.S_un.S_addr & hIp->subnet.S_un.S_addr) == (ip & hIp->subnet.S_un.S_addr) )
		{
			// the requested IP address fits to this network ... generate ARP request
			sendArpRequest(hIp, (struct in_addr*)&ip );
			break;
		}
	}
	
	return retVal;
}

static void ip_arp_in(IP_STACK_H hIp, eth_frame *pFrame)
{
	arp_hdr			*pArpOut,*pArpIn;
	ip_buf_type		*pBuf;
	struct in_addr	tempIp;

	// check if arp frame received (must be addressed to local IP address)
	if(pFrame->eth.type == HTONS(IP_ETHTYPE_IP))
	{
		// Only insert/update an entry if the
		// incoming IP packet is sent to the local IP
#ifdef IP_SECONDARY_ADDRESS
		if(memcmp(&pFrame->prot.ip.dst_ip, &hIp->local_ip_addr,4) && memcmp(&pFrame->prot.ip.dst_ip, &hIp->local_ip_addr2,4)) return;
#else
		if(memcmp(&pFrame->prot.ip.dst_ip, &hIp->local_ip_addr,4)) return;
#endif

		ipArpUpdate(hIp, pFrame->eth.src_hw, pFrame->prot.ip.src_ip);
	}
	else if(pFrame->eth.type == HTONS(IP_ETHTYPE_ARP))
	{
		pArpIn = (void*)&pFrame->prot.arp;

		// signal IN_USE if state is init_arp and received ARP comes from desired IP
		if( (hIp->state == IP_STATE_INIT_ARP) & (memcmp(&hIp->local_ip_addr, &pArpIn->src_ip, 4) == 0) )
		{
			hIp->state = IP_STATE_ERROR_IP_IN_USE;
		}


		if(pArpIn->opcode ==  HTONS(ARP_REPLY))
		{
			ipArpUpdate(hIp, pArpIn->src_hw, pArpIn->src_ip);
		}
		else if(
			pArpIn->opcode == HTONS(ARP_REQUEST)
			&&
			((hIp->state==IP_STATE_OK)||(hIp->state==IP_STATE_DHCP_RENEWING)||(hIp->state==IP_STATE_DHCP_REBIND) )
			)
		{
			pBuf = ip_alloc_tx_buffer(hIp);	// get tx buffer

			if(pBuf==0) return;	// no tx buffer available
			
			pFrame = &pBuf->data.frame;	// access to data of allocated buffer

			// build arp response
			pFrame->eth.type = HTONS(IP_ETHTYPE_ARP);

			//--------- take destination hw from arp header
			copy_eth_address(pFrame->eth.dst_hw,	pArpIn->src_hw);

			//---------- arp header
			// access to arp response
			pArpOut = &pFrame->prot.arp;
			
			// copy arp request to response (hwtype, protocol, hwlen, ...)
			memcpy(pArpOut,pArpIn,sizeof(arp_hdr));

			pArpOut->opcode = HTONS(ARP_REPLY);

			copy_eth_address(pArpOut->dst_hw,	pArpIn->src_hw);
			copy_eth_address(pArpOut->src_hw,	hIp->local_eth_addr.addr);

			// swap target and sender ip address
			copy_ip_address(&tempIp, pArpOut->dst_ip);
			copy_ip_address(pArpOut->dst_ip, pArpIn->src_ip);
			copy_ip_address(pArpOut->src_ip, &tempIp);

			pBuf->length = sizeof(eth_hdr)+sizeof(arp_hdr);

			ip_buf_send(hIp,pBuf,0);	// send packet with reply to to network
		}
		else if(pArpIn->opcode == HTONS(ARP_ANNOUNCEMENT))
		{
			if(pArpIn->src_hw[0] != 0xFFFF)	// normal ARP announcement (RFC5227)
			{
				ipArpAnnouncement(hIp, pArpIn->src_hw, pArpIn->src_ip);
			}
			else	// source-hw is set to broadcast ... use the senders source-MAC in MAC-header
			{
				ipArpAnnouncement(hIp, pFrame->eth.src_hw, pArpIn->src_ip);
			}
		}
	}
}

//-----------------------------------------------------------------------------
static void ip_icmp_in(IP_STACK_H hIp, eth_frame *pFrame, unsigned short ipHdrLen)
{
	unsigned short	chksum;
	icmp_hdr		*pICMP = (icmp_hdr*)((char*)&pFrame->prot.ip + ipHdrLen);
	ip_buf_type		*pBuf;

	IP_STAT(hIp->stat.icmp_rx++);

	// ICMP echo (i.e., ping) processing. This is simple, we only change the ICMP type
	// from ECHO to ECHO_REPLY and adjust the ICMP checksum before we return the packet
	if(pICMP->type != ICMP_ECHO)
	{
		IP_STAT( hIp->stat.ip_err_icmp++ );
		IP_LOG("icmp: not icmp echo.");
		return;
	}
	
	pBuf = ip_alloc_tx_buffer(hIp);	// get tx buffer

	if(pBuf==0) return;	// no tx buffer available

	// overtake complete icmp message
	pBuf->length = sizeof(eth_hdr) + htons(pFrame->prot.ip.len);
	memcpy(&pBuf->data, pFrame, pBuf->length);

	pICMP = (icmp_hdr*)((char*)&pBuf->data.frame.prot.ip + ipHdrLen);

	pICMP->type = ICMP_ECHO_REPLY;

	// update checksum
	chksum = pICMP->chksum;
	if(chksum >= HTONS(0xFFFF - (ICMP_ECHO << 8))) chksum++;
	pICMP->chksum = chksum + HTONS(ICMP_ECHO << 8);

	IP_STAT( hIp->stat.icmp_tx++);

	ip_buf_send(hIp, pBuf, TX_IP_REPLY);
}

//-----------------------------------------------------------------------------
static void ip_udp_in(IP_STACK_H hIp, eth_frame *pFrame, unsigned short ipHdrLen)
{
	ip_hdr			*pIP  = &pFrame->prot.ip;
	udp_hdr			*pUDP = (udp_hdr*)(((char*)pIP) + ipHdrLen);
	listen_type		*pList;
	ip_udp_info		info;	// info structure for callback

	#if IP_STATISTICS == 1
		hIp->stat.udp_rx++;
	#endif

	// search for udp connection with this port
	for(pList = hIp->listen_udp ; pList < hIp->listen_udp + IP_LISTEN_PORTS_UDP ; pList++)
	{
		if(pList->lport != 0  &&  pList->lport == pUDP->dst_port)
		{
			// UDP connection with this local port found ... callback to application

			// create info structure
			info.pData			= ((char*)pUDP)+sizeof(udp_hdr);
			info.len			= htons(pUDP->len)-sizeof(udp_hdr);
			info.localPort		= htons(pUDP->dst_port);
			info.remotePort		= htons(pUDP->src_port);
			info.pRemoteMac		= (eth_addr*)&pFrame->eth.src_hw;

			copy_ip_address(&info.remoteHost, pIP->src_ip);
			copy_ip_address(&info.localHost,  pIP->dst_ip);

			pList->pFct(pList->arg, &info);

			return;
		}
	}

	// dispatch default ports
	switch(pUDP->dst_port)
	{
		#if IP_DHCP == 1
		case HTONS(IP_DHCP_PORT_CLIENT):
			ip_dhcp_receive(hIp, pFrame, ipHdrLen);
			break;
		#endif

		default:
			IP_STAT( hIp->stat.udp_unused++ );
			break;
	}
}

/*********************************************************************************

  Function    : ipUdpSend
  Description : send UDP frame to ethernet driver

  Parameter:
	hIp		: handle of used interface
	pInfo	: ptr to structure with connection info

  Return Value:
	>0 ... number of sent bytes (must be same as given value in pInfo->len
	0  ... no send buffer available or driver not yet ready, try again
	-1 ... Error (hIp=0 or length too big)
*********************************************************************************/
int ipUdpSend(IP_STACK_H hIp, ip_udp_info *pInfo)
{
	ip_buf_type	*pBuf;
	void		*ptr;

	// check pointers
	if(hIp==0 || pInfo==0) return -1;

	// check maximum send length
	if(pInfo->len > IP_MTU - sizeof(ip_hdr)-sizeof(udp_hdr)) return -1;
	
	// not ready and no broadcast
	if((hIp->state != IP_STATE_OK) && (pInfo->remoteHost.S_un.S_addr !=0xFFFFFFFF)) return 0;

	pBuf = ip_alloc_tx_buffer(hIp);	// get tx buffer

	if(pBuf==0) return 0;	// no tx buffer available

	// prepare IP header
	ptr = &pBuf->data.frame.prot.ip;
	
	IP(ptr)->len	= sizeof(ip_hdr) + sizeof(udp_hdr) + pInfo->len;
	IP(ptr)->proto	= IPPROTO_UDP;

	copy_ip_address(IP(ptr)->dst_ip, &pInfo->remoteHost);

	// prepare udp frame and send
	ptr = (udp_hdr*)(((char*)ptr) + sizeof(ip_hdr));

	memcpy( UDP(ptr)+1 , pInfo->pData, pInfo->len);			// copy udp data to send buffer

	UDP(ptr)->dst_port	= htons(pInfo->remotePort);
	UDP(ptr)->src_port	= htons(pInfo->localPort);
	UDP(ptr)->len		= htons((unsigned short)(pInfo->len + sizeof(udp_hdr)));
	UDP(ptr)->chksum	= 0;

	IP_STAT(hIp->stat.udp_tx++);

	ip_buf_send(hIp, pBuf, TX_IP_HEADER);	// send frame

	return pInfo->len;
}


/*********************************************************************************

  Function    : ipPacketReceive
  Description : marks a buffer which was allocated with ip_packet_produce()
				as ready for processing

  Parameter:
	hIp		: handle of used interface
	pBuffer	: ptr of buffer with data
	len		: number of valid bytes in buffer
	pFct	: ptr to release-function when buffer is not used anymore

  Returns:
	0  ... buffer was overtaken and will be released with the function pFct
	-1 ... buffer was not taken, it can be reused

*********************************************************************************/
int	ipPacketReceive(IP_STACK_H hIp, ip_packet_typ *pPacket, IP_BUF_FREE_FCT *pFct)
{
	struct in_addr	ipAddr;
	ip_rx_queue_typ	*pQueue;
	eth_frame		*pFrame;

	if(pPacket==0 || hIp==0) return -1;

	// pre-filter to see if buffer should be processed
	hIp->stat.rxPackets++;

	pFrame = (eth_frame*)pPacket->data;

	// filter to discard frames which are not used
	if (pFrame->eth.type == HTONS(IP_ETHTYPE_IP))
	{
		copy_ip_address(&ipAddr,pFrame->prot.ip.dst_ip);

		// if ip broadcast received (also subnet-broadcast) simulate local address
		if((ipAddr.S_un.S_addr | hIp->netmask.S_un.S_addr) == 0xFFFFFFFF)
		{
			ipAddr.S_un.S_addr = hIp->local_ip_addr.S_un.S_addr;
		}
	}
	else if(pFrame->eth.type == HTONS(IP_ETHTYPE_ARP))
	{
		// check if dst and src ARP address is same ... in this case we received an announcement ... forward to stack
		if( memcmp(pFrame->prot.arp.dst_ip, pFrame->prot.arp.src_ip, 4) == 0 )
		{
			ipAddr.S_un.S_addr = hIp->local_ip_addr.S_un.S_addr;
			pFrame->prot.arp.opcode = HTONS(ARP_ANNOUNCEMENT);	// mark frame as announcement frame
		}
		else
		{
			copy_ip_address(&ipAddr,pFrame->prot.arp.dst_ip);

			// ARP Response to ARP Probe will have 0 in dst_ip, use source IP in this case
			if(ipAddr.S_un.S_addr == 0) copy_ip_address(&ipAddr,pFrame->prot.arp.src_ip);
		}
	}
	else
	{
		return -1;	// not ARP and not IP, re-use buffer for the next time
	}

	// ip frame not for local ip, discard frame, re-use buffer for the next frame
#ifdef IP_SECONDARY_ADDRESS
	if( (ipAddr.S_un.S_addr == hIp->local_ip_addr.S_un.S_addr) || (ipAddr.S_un.S_addr == hIp->local_ip_addr2.S_un.S_addr))
#else
	if(ipAddr.S_un.S_addr == hIp->local_ip_addr.S_un.S_addr )
#endif
	{
		// add to receive queue and increment rx buf write index
		pQueue = hIp->pRxWrite;
		
		if(pQueue->pPacket==0)	// queue entry free
		{
			IP_STAT(hIp->stat.packets_used++);	// count incoming packets

			// store callback function inside the frame (first 4 bytes of dst-mac)
			((ip_int_hdr*)pFrame)->pFct = pFct;
			
			pQueue->pPacket	= pPacket;		// write info to queue

			hIp->pRxWrite = pQueue->pNext;	// set to next queue element

			return 0;
		}
		else
		{
			// queue full, free buffer and return
			IP_STAT(hIp->stat.packets_queue_full++);	// count incoming packets
		}
	}

	return -1;
}

// get ip stack state
unsigned short	ipGetState(IP_STACK_H hIp)
{
	return hIp->state;
}

#if IP_REASS_BUF_CNT > 0

// local variable which only hold the bitmasks for the reassembly algorithm
static const unsigned char bitmask[] = {0xFF,0x7F,0x3F,0x1F,0x0F,0x07,0x03,0x01,0x00};



static reass_buf_type *ip_reass(IP_STACK_H hIp, eth_frame *pFrame)
{
	unsigned short		len, hdrLen;
	unsigned short		offset;
	reass_buf_type		*pBuf;
	unsigned char		*pByte,*pBitmap;

// 	logReass( pFrame->prot.ip.ipid, pFrame->prot.ip.ipoffset);
// 	if(pFrame->prot.ip.ipoffset == 105) assert(0);

	// loop through reassembly buffers and decrement timer (buffers with timer=0) will be deleted if the 
	// memory is required
	pBuf	= hIp->pReassBuffer;	// set to first reassembly buffer (after mac rx buffers)
	offset	= 0;

	for(len=1;len<=IP_REASS_BUF_CNT;len++ , pBuf++)
	{
		// check if this ip datagram is already somewhere in the reassembly buffers
		if(pBuf->buf.data.frame.prot.ip.ipid == pFrame->prot.ip.ipid
			&&
			memcmp(pBuf->buf.data.frame.prot.ip.src_ip, pFrame->prot.ip.src_ip,8) == 0 )
		{
			// skip packet completely if a buffer is existing with the same ipid/srcip/dstip
			// (the received fragment is either too late and the reass buffer was dropped becuase
			// of timeout or the fragment was duplicated somewhere on the way and first copy
			// arrived already
			if(pBuf->timer == 0)
			{
				IP_STAT( hIp->stat.ip_reass_late_rx++);
				return 0;
			}
			
			offset = len;	// use this reassembly buffer (remember index and finish for-loop)
			break;			// stop loop and proceed with reassembly
		}

		// store buffer index if timer is elapsed (buffer free)
		if(offset==0 && pBuf->buf.header.state == IP_BUF_STATE_IDLE) offset = len;
	}

	if(offset==0) return 0;		// no existing reassembly buffer for this frame found or no free buffer available
	
	hdrLen	= (pFrame->prot.ip.vhl & 0xF)*4;			// length of ip header
	pBuf	= hIp->pReassBuffer + offset - 1;
	pBitmap	= pBuf->bitmap;

	// new fragment buffer, initialize with headers
	if(pBuf->timer==0)
	{
		pBuf->timer = IP_REASS_MAXAGE;
		pBuf->buf.header.dataSize = 0;	// will be set to ip payload length when last packet is received

		// copy the ethernet + ip header to the buffer, ip-options are ignored
		// (too complicated to handle options of fragmented datagrams)
		memcpy(&pBuf->buf.data.frame , pFrame , sizeof(eth_hdr) + hdrLen);
		memset(pBitmap,0xFF,sizeof(pBuf->bitmap));						// clear the bitmap
	}

	// get length and offset from header
	len		= htons(pFrame->prot.ip.len) - hdrLen;
	offset	= htons(pFrame->prot.ip.ipoffset);

	if((offset & IP_FRAG_FLAG_MORE) == 0)	// last fragment
	{
		pBuf->buf.header.dataSize = (unsigned short)(offset*8 + len);	// payload data

		// write complete ip length to header
		pBuf->buf.data.frame.prot.ip.len = htons((unsigned short)(pBuf->buf.header.dataSize+sizeof(ip_hdr)));
	}

	// remove flags in offset and convert to bytes (original value is in 8-byte-multiples)
	offset = offset * 8;

	// If the offset or the offset + fragment length overflows the reassembly
	// buffer, we discard the entire packet
	if(offset > IP_MTU-sizeof(ip_hdr)  ||  offset+len > IP_MTU-sizeof(ip_hdr))
	{
		pBuf->timer = 0;
		return 0;
	}

    // copy the fragment into the reassembly buffer, at the right offset
    memcpy(((char*)&pBuf->buf.data.frame.prot.ip) + sizeof(ip_hdr) + offset, ((char*)&pFrame->prot.ip) + hdrLen, len);

	// calculate index of the first and the last bit to be set in the bitmap
	offset	= offset / 8;	// this is the index of the first bit
	len		= (len-1)/8+1;	// number of bits which should be set

	pByte = pBitmap + offset/8;	// set byte pointer to the byte which contains the first bit
	
	offset = offset & 7;	// get bit offset in the first byte

	if(offset)	// bit-set does not start on a byte-beginning, therefore we have to set the first few bits with &
	{
		if(offset + len > 7) 
		{
			*(pByte++) &= ~bitmask[offset];	// clear all bits until the end of this byte
			len = len + offset - 8;
		}
		else
		{
			*(pByte++) &= ~bitmask[offset] | bitmask[offset+len];
			len = 0;	// all bits done
		}
	}
	while(len>7)		// set all 64-byte blocks
	{
		*(pByte++) = 0;
		len-=8;
	}
	if(len) *pByte &= bitmask[len];	// set remaining bits

	len = (unsigned short)pBuf->buf.header.dataSize;	// get total buffer length from buffer header

	if(len==0) return 0;		// last fragment not yet received, test makes no sense at this moment, wait

	//---- test if all fragments were received ----
	len		= (len-1) / 8;	// index of the last bit which has to be 1 in the bitmap (1 bit for every 8 bytes)
	offset	= len & 7;		// get index (0-7) of the last bit in the last byte
	len		= len / 8;		// get index of last bitmap byte

	// set byte ptr to first bitmap byte
	pByte = pBitmap;

	// compare all bytes in bitmap to see if all fragments are arrived
	while(len)
	{
		len--;
		if(*(pByte++)) return 0;	// not yet complete
	}

	// compare last byte, it is maybe not fully used because the overall length must not be multiple of 8
	if(*pByte != bitmask[offset+1]) return 0;		// not yet complete

	// frame ready, pass it to the stack
	pBuf->buf.header.state = IP_BUF_STATE_RX;
	pBuf->timer = 0;

	// Pretend to be a "normal" (i.e., not fragmented) IP packet from now on
	pByte = (void*)&pBuf->buf.data.frame.prot.ip;
	IP(pByte)->vhl		= (IP_VERSION_V4<<4) + sizeof(ip_hdr)/4;
	IP(pByte)->ipoffset	= 0;
	IP(pByte)->chksum	= 0;
	IP(pByte)->chksum	= ip_chksum((ip_hdr*)pByte,0);

	return pBuf;
}
#endif

