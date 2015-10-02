/* ip_internal.h - Ethernet Library for FPGA MAC Controller*/ 
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
 File:      ip_internal.h
 Author:    Thomas Enzinger(enzingert)
 Created:   26.03.2004
------------------------------------------------------------------------------
 History:
 26.03.2004	enzingert	created
 05.04.2004 et			minor changes in internal structures

 16.06.2004	enzinger	- Interface of some functions changed
							(data interface to ethernet driver now with packets
							instead of ptr and length)
						 --> The interface to the application has not changed 
 28.02.2005 enzinger	- structure element time_ms removed .. not required anymore
 14.03.2005	enzinger	- new flag in tx-buffer (for tcp-ip communication)
 13.09.2007	enzinger	- New function sock_set_mtu()
 24.10.2007	enzinger	- Some elements of the IP-structure are allocated and not static anymore (no functional change)
 12.02.2008	enzinger	additions for IP_LOOPBACK and IP_ACTIVE_OPEN
 06.03.2008	enzinger	flag-field in socket structure (for setsockopt() )
 26.05.2008	enzinger	socket-type changed
 11.07.2008	enzinger	debug-infos to ip-type added (dbgSockState)
 11.08.2008	enzinger	bug in tx-buffer handling solved (for TCP-Communication)
 16.03.2010 prenninger	move defines from ip.c and ip_internal.h to ip.h
 22.06.2011	enzinger	state-konstanten in enum umgebaut, bessere debug-anzeige
 12.01.2012	enzinger	Erweiterungen für 2te IP-Adresse
 02.03.2012	enzinger	Korrektur für Module mit IP_LOOPBACK==1 (z.Bsp. X20BC0088)

------------------------------------------------------------------------------
*/

/*****************************************************************************
DESCRIPTION
Common defines and declarations for ip stack and socket interface

INCLUDE FILES:
 ip_opt.h

STATE:
 stabil

*/

#ifndef __IP_INTERNAL_H__
#define __IP_INTERNAL_H__

#include "ip.h"
#include "ip_opt.h"

// options for the function ip_buf_send()
#define TX_IP_REPLY				0x0001	// reply to sender ip address
#define TX_IP_HEADER			0x0002	// create ip header
#define TX_IP_TCP_CHKSUM		0x0004	// build tcp checksum
#define TX_ARP_PROBE			0x0008	// send arp request to search for duplicate IP (RFC5227)
#define TX_SECONDARY_IP			0x0010	// use secondary-ip to send this packet
#define TX_ARP_ANNOUNCEMENT		0x0020	// ARP announcement (RFC5227)

// state of tx buffers / reassembly buffers
#define IP_BUF_STATE_IDLE		0	// buffer available
#define IP_BUF_STATE_RX			1	// rx packet
#define IP_BUF_STATE_TX			2	// tx packet (mark as IDLE after sending)
#define IP_BUF_STATE_TX_ACK		3	// mark as DONE after sending (for possible retransmit)
#define IP_BUF_STATE_TX_DONE	4	// buffer was sent to the network

#define IP_BUF_STATE_TX_Q		5	// like TX but already in TX-queue
#define IP_BUF_STATE_TX_ACK_Q	6	// like TX-Ack but already in TX-queue

#define IP_FRAME_QUEUED			0
#define IP_FRAME_RETRY			1
#define IP_FRAME_UNUSED			2

#define IP_SECONARY_HOST_QUEUE	10	// number of last hosts which accessed the secondary IP

#if IP_STATISTICS == 1
	#define IP_STAT(s) s
#else
	#define IP_STAT(s)
#endif


#if IP_LOGGING == 1
	#include <stdio.h>
	void ip_log(char *msg);
	#define IP_LOG(m) ip_log(m)
#else
	#define IP_LOG(m)
#endif

//*************************************************************************************
//
// Get the base address of a structure where only the address of a sub-element is known
//
//	typ		: type name of the regarding struct (must be done with typedef)
//	element	: name of the structure element from which we know the address
//	ptr		: address of the structure element
//
//	result	: structure pointer to the base of the type
//
//*************************************************************************************
#define GET_TYPE_BASE(typ, element, ptr)	\
	((typ*)( ((size_t)ptr) - (size_t)&((typ*)0)->element ))


/************************************************************************************
*************************************************************************************

	Types

*************************************************************************************
************************************************************************************/
//--------------------- header which replaces the destination mac ---------------
typedef struct
{
	IP_BUF_FREE_FCT		*pFct;
	unsigned short       tag;	// buffer-tagging for debug purpose, normally not used
}ip_int_hdr;


//---------------------------------- packet buffer ----------------------------------

//----------------- header of packet (must be multiple of 4 !!!! ---------------
typedef struct
{
	unsigned short	dataSize;	// payload data
	unsigned char	state;		// 0..IDLE
	unsigned char	push;		// 1 if send() has completed the buffer of the user
}ip_buf_hdr;

//------------------ data of packet
typedef union
{
	eth_frame		frame;
	unsigned long	dataLong[((IP_MTU+18)+3)/4];	// data array are longs to make sure the the field starts on an 4-byte address
}ip_buf_data;

//------------------- complete packet
typedef struct
{
	ip_buf_hdr		header;
	unsigned long	length;		// data length
	ip_buf_data		data;
}ip_buf_type;

//--------------------------------- arp table entry ---------------------------------
typedef struct
{
	struct in_addr	ip;
	eth_addr		eth;
	unsigned short	time;		// Timestamp of arp entry (arp_time_s)
}arp_table_entry;

//-------------------- udp listen type
typedef struct
{
	unsigned long	lport;		// listen port
	IP_HOOKFCT		*pFct;		// hook function
	void			*arg;		// user info for each connection
}listen_type;

//-------------------- structure for 1 reassembly-buffer
typedef struct
{
	ip_buf_type		buf;
	unsigned char	timer;
	// reassemble-bitmap to store which fragments we have received already
	unsigned char	bitmap[ (IP_MTU-sizeof(ip_hdr)-1)/64 + 1 ];
}reass_buf_type;

//--------------------------------- UDP header ---------------------------------
typedef struct
{
	unsigned short	src_port;
	unsigned short	dst_port;
	unsigned short	len;
	unsigned short	chksum;
}udp_hdr;

//--------------------------------- TCP header ---------------------------------
typedef struct
{
	unsigned short	src_port;
	unsigned short	dst_port;
	unsigned short	seqno[2];
	unsigned short	ackno[2];
	unsigned char	tcpoffset;
	unsigned char	flags;
	unsigned short	wnd;
	unsigned short	chksum;
	unsigned short	urgp;
}tcp_hdr;


#define	IP(p)		((ip_hdr*)p)
#define	UDP(p)		((udp_hdr*)p)

//--------------------------------- tcp/ip connection ---------------------------------
//
// Representation of a TCP connection.
//
// The ip_conn structure is used for identifying a connection (32 Byte / connection)
//
//-------------------------------------------------------------------------------------
typedef struct IP_CONN_HEADER
{
	unsigned char	state;		// TCP/UDP state and flags
	unsigned char	cntOpen;
	unsigned char	cntFree;
	unsigned char	reserve;
}IP_CONN_HEADER;

typedef struct IP_CONN
{
	IP_CONN_HEADER	header;
	// header must always be the first element !!

	IP_STACK_H		hIp;		// handle to if structure (0 socket structure not used)

	unsigned char	ack;		// data was acknowledged
	unsigned char	cmdClose;	// user commands to the stack

	unsigned short	type;		// DGRAM/...

	unsigned short	lport;		// local TCP port
	unsigned short	rport;		// local remote TCP port

	struct in_addr	ripaddr;	// IP address of the remote host

	unsigned long	rcv_nxt;	// The sequence number that we expect to receive next
	unsigned long	snd_nxt;	// The sequence number that was last sent by us
	unsigned short	len;		// Length of the data that was previously sent
	unsigned short	mss;		// Current maximum segment size for the connection
	unsigned short	initialmss;	// Initial maximum segment size for the connection
	unsigned char	sa;			// Retransmission time-out calculation state variable
	unsigned char	sv;			// Retransmission time-out calculation state variable
	unsigned char	rto;		// Retransmission time-out
	unsigned char	timer;		// The retransmission timer
	unsigned char	nrtx;		// The number of retransmissions for the last segment sent

	ip_buf_type		*pTx;		// address to tx buffer
	
	IP_BUF_FREE_FCT *pFctFree;	// function ptr to free rx buffer
	eth_frame		*pRx;		// address to rx buffer

	char			*pRecvData;	// address to next raw data byte in tcpip buffer
	unsigned short	recvLen;	// number of bytes in revive buffer
	unsigned short	flags;		// socket flags (SOCK_FLAG ...)
}IP_CONN;

typedef IP_CONN	*SOCK_PTR;

//-----------------	Complete IP/UDP/TCP data of 1 ethernet interface -----------------
struct IP_IF
{
	eth_addr		local_eth_addr;	// local ethernet address
	unsigned short	ipid;			// incrementing datagram identification
	unsigned long	time_ms_old;	// old ms time stamp to build difference
	unsigned long	time_s;			// free running second counter
	struct in_addr	local_ip_addr;	// local ip address
	struct in_addr	netmask;		// netmask
	struct in_addr	subnet;			// subnet (local_ip_addr & netmask)
	struct in_addr	gateway;		// gateway IP address

	ipState_enum	state;			// IP_STATE_xxxx

#ifdef IP_SECONDARY_ADDRESS
	struct in_addr	local_ip_addr2;	// local ip address (secondary)
	struct in_addr	netmask2;		// netmask (secondary)
	struct in_addr	subnet2;		// subnet (local_ip_addr & netmask)

	struct in_addr	ip2_remotehosts[IP_SECONARY_HOST_QUEUE];
	int ip2_remotehost_index;
#endif


	//------------------ ARP ------------------------
	unsigned char	arp_refresh;
	unsigned char	arp_probe;

	arp_table_entry	arp_table[IP_ARP_TABSIZE];			// arp table (12 byte RAM / entry)

	//------------------ DHCP ------------------------
	#if IP_DHCP == 1
		unsigned long	dhcp_interval;
		unsigned long	dhcp_lease;
		unsigned long	dhcp_timer;
		unsigned long	dhcp_t1;
		unsigned long	dhcp_t2;
		struct in_addr	dhcp_server;
		struct in_addr	dhcp_given;
		unsigned char	hostName[32];
	#endif

	//----------------- ptr and arg of ethernet driver send function -------------------
	IP_ETHSEND		*pEthSend;
	void			*hEth;

	//------------------  rx/tx queues  --------------------------

	unsigned char	txQRead,txQWrite;

	ip_rx_queue_typ	*pRxWrite,*pRxRead;		// read and write ptr for receive queue

	// queue for buffer information of received frames
	ip_rx_queue_typ	rxQueue[IP_RX_BUF_CNT];		// rx queue with buffer pointers
	ip_buf_type		*txQueue[IP_TX_BUF_CNT];	// tx queue with buffer pointers

	#if IP_LOOPBACK==1
		// rx queue for loopback buffers (since packets are taken from tx buffers we only need to reserve that much
		ip_rx_queue_typ	rxQueue2[IP_TX_BUF_CNT];
		ip_rx_queue_typ	*pRxWrite2,*pRxRead2;		// read and write ptr for receive queue 2
	#endif

	//------------------  listen ports and sockets  --------------------------

	listen_type	listen_udp[IP_LISTEN_PORTS_UDP];	// UDP listen ports (12 byte RAM / entry)

	#if IP_TCP_SOCKETS > 0
		IP_CONN			sock[IP_TCP_SOCKETS];		// sockets
	#endif

	unsigned short	nextFreePort;

	//------------------  rx/tx/reass buffers  --------------------------
	ip_buf_type		*pTxBuffer;		// buffers for active sending

	#if IP_REASS_BUF_CNT > 0
		reass_buf_type	*pReassBuffer;	// buffers for reassembly (can also become a tx buffer)
	#endif

	//------------------  statistics  --------------------------
	ip_stat		stat;

	#if IP_TCP_SOCKETS > 0
		char dbgSockState[IP_TCP_SOCKETS];
	#endif

	#ifdef DEBUG
		char dbgBufTxState[IP_TX_BUF_CNT];
		ip_buf_type *pDbgTxBuf[IP_TX_BUF_CNT];
	#endif
	
	struct IP_IF *pNext;		// ptr to next ip stack if existing
};

//----------------------------------- functions of ip.c ---------------------------------------

// allocate a tx buffer
ip_buf_type*	ip_alloc_tx_buffer(IP_STACK_H hIp);

// add buffer to send queue
void			ip_buf_send(IP_STACK_H hIp, ip_buf_type *pBuf, unsigned long option);


//------------------------- function declarations -----------------------------
// ip_sock.c
void sock_set_ip(IP_STACK_H hIp);
int  sock_in(IP_STACK_H hIp,eth_frame *pFrame, unsigned short ipHdrLen);
void sock_out(IP_STACK_H hIp);
void sock_periodic(IP_STACK_H hIp);
void sock_set_mtu(unsigned short mtu);

#endif
