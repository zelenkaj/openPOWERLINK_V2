/* ip_sock.c - TCPIP */ 
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
 File:      ip_sock.c
 Author:    Thomas Enzinger(enzinger)
 Created:   12.03.2004
------------------------------------------------------------------------------
 History:
 29.03.2004	enzinger	first version
 30.03.2004 enzinger	ipGetHandle() new						
 05.04.2004 enzinger	minor changes in internal structures

 16.06.2004	enzinger	- Interface of some functions changed
							(data interface to ethernet driver now with packets
							instead of ptr and length)
						 --> The interface to the application has not changed 
 14.03.2005	enzinger	- push-flag is now only set if the last packet of a send()-call 
						was transmitted
 16.03.2005	enzinger	- if a send() is called before ipClose(), the data will be sent first
						and then the connection will be closed
						- pData for receive and send to void-pointers changed
 07.12.2005	enzinger	- any rx-data which is left will be discarded if the user closes the connection
   .11.2006	enzinger	- Functions and types renamed to windows standard function names
						- Support for IP Multicast added, support for DGRAM-Sockets added
 22.02.2007	enzinger	- TCP/UDP Packets will not be dropped anymore, if the socket is occupied with old data
						the queue processing will be stopped for the current cycle and the frame will be
						processed at the next cycle.
 19.03.2007	enzinger	- when closing a UPD socket the socket was 'lost' (not available for new socket anymore)
                        (caused problems when opening/closing UDP sockets several times)
 06.04.2007	enzinger	- TCP KeepAlive packets were not acknowledged if TCP-Data size was 0
 13.09.2007	enzinger	- New function sock_set_mtu() -> MTU can be modified in order to limit MSS
						(required for EPL Safety)
 12.02.2008	enzinger	functions connect() and select() added
 06.03.2008	enzinger	setsockopt(s, IPPROTO_TCP, TCP_ACK_REQUEST,(void*)1,0)  can be used to cause an ack after data frames
                         (to overcome the fact that the client has not set TCP_NODELAY)
 01.04.2008	enzinger	when TCP retransmission was received the BC did only responde with ACK (without data)
						-> corrected : retransmission response now also contains correct data
						(this only happened when packets got lost on ethernet)
 26.05.2008	enzinger	Socket functions are now reentrant
 10.07.2008	enzinger	Bug in closesocket() corrected (could lead to invalid receive frames)
 11.07.2008	enzinger	Improved handling of reused-sockets (for Ajax communication)
 30.07.2008	enzinger	Improved handling of socket handling (for very high tcp-ip load from web-clients)

 10.03.2009	enzinger	Rx-Data after closing socket will be ignored
 24.03.2009	enzinger	New socket-flag TCP_REJECT_SEGMENT (SOCK_FLAG_REJECT_SEGMENT) to mark special 
						sockets (firmware download-socket)
 02.04.2009	enzinger	TCP state machine optimized
 04.01.2011	enzinger	Limitation of MSS
 12.01.2012	enzinger	Erweiterungen für 2te IP Adresse
 30.07.2012 enzinger    TCP-Socket will accumulate more send() packets into the same buffer if socket flag
                        TCP_PACK_SEND_DATA is set, and old buffer is not yet transferred to send queue
						(implemented for X20BC0087)

------------------------------------------------------------------------------
*/

/*****************************************************************************
DESCRIPTION
TCP/IP stack + socket interface

DEPENDENCIES:
 ip.c

INCLUDE FILES:
 ip_opt.h
 ip.h
 ip_internal.h
 string.h 

STATE:
 creating (everything is subject to change)
*/

#include "ip.h"
#include "ip_internal.h"

#include <assert.h>
#include <string.h>

#if IP_TCP_SOCKETS > 0

#ifdef IP_LOCK_LEVEL
	#include <px32intlib.h>

	#define IP_LOCK_LEVEL_VAR	unsigned short	level;
	#define IP_LOCK_LEVEL_ON	level = px32intLevelLock(PX32INT_LEVEL(IP_LOCK_LEVEL));
	#define IP_LOCK_LEVEL_OFF	px32intLevelRestore(level);
#else
	#define IP_LOCK_LEVEL_VAR	;
	#define IP_LOCK_LEVEL_ON	;
	#define IP_LOCK_LEVEL_OFF	;
#endif

#define RET_SOCK_ERROR(e)	{ipSockInt.lastError = e; return SOCKET_ERROR;}
#define RET_SOCK_INVALID(e)	{ipSockInt.lastError = e; return INVALID_SOCKET;}

static struct 
{
	struct IP_IF	*hIpList;	// interface list (currently only 1 supported)
	unsigned long	lastError;
	unsigned short	mss;
}ipSockInt;

// options for the function ip_tcp_send()
//#define TX_TCP_ACK		0x0001	// send ack
#define TX_TCP_SYN		0x0002	// send syn
//#define TX_TCP_NODATA	0x0004	// send no data

// The states used in the socket->state
#define IP_FREE			0
#define IP_CLOSED		1
#define IP_BOUND		2
#define IP_SYN_RCVD		3
#define IP_CONNECTED	4
#define IP_FIN_WAIT_1	5
#define IP_FIN_WAIT_2	6
#define IP_CLOSING		7
#define IP_TIME_WAIT	8		// wait after closing, maybe remote asks for resending the last ack
#define IP_LAST_ACK		9		// wait till remote send ack to our FIN
#define IP_LISTEN		10
#define IP_SYN_TX		11
#define IP_SYN_SENT		12
#define IP_SYN_ACK_TX	13

static const char dbgSockStateInfo[] = {
	'-',	// Free
	'Z',	// Closed (sockets should never 'freeze' on this state, then something is wrong)
	'B',	// Bound
	'/',	// Syn (connection establishment)
	'+',	// Verbunden (connection established)
	'1',	// WAIT_1 (comes after active close)
	'2',	// WAIT_2 (comes after active close)
	'\\',	// CLOSING (comes after active close)
	'?',	// TIME_WAIT (comes after active close, socket is used when no 'F' Socket is available)
	'A',	// last ack
	'L',	// Listen

	't',	// syn transmitted
	's',	// syn sent
	'a',	// syn ack

	'X'};

// Structures and definitions
#define TCP_FIN		0x01
#define TCP_SYN		0x02
#define TCP_RST		0x04
#define TCP_PSH		0x08
#define TCP_ACK		0x10
#define TCP_URG		0x20
#define TCP_CTL		0x3f

// socket flags
#define SOCK_FLAG_ACKREQ			0x0001
#define SOCK_FLAG_REJECT_SEGMENT	0x0002
#define SOCK_FLAG_SECONDARY_IP		0x0004
#define SOCK_FLAG_PACK_SEND_DATA	0x0008


/*
*  The maximum number of times a segment should be retransmitted
*  before the connection should be aborted.
*
*  This should not be changed.
*/
#define IP_MAXRTX			3

/*
*  The maximum number of times a SYN segment should be retransmitted
*  before a connection request should be deemed to have been
*  unsuccessful.
*
*  This should not need to be changed.
*/
#define IP_MAXSYNRTX		3

/*
*  How long a connection should stay in the TIME_WAIT state.
*
*  This configuration option has no real implication, and it should be
*  left untouched.
*/ 
#define IP_TIME_WAIT_TIMEOUT 120

/*
*  The initial retransmission timeout counted in timer pulses.
*
*  This should not be changed.
*/
#define IP_RTO				3

#define IP_DYNAMIC_PORT_RANGE	49152



void ip_tcp_appsend(SOCK_PTR socket);
void ip_tcp_send(SOCK_PTR socket, ip_buf_type *pBuf, unsigned short flags);
void ip_socket_free(SOCK_PTR socket, int state);



/*********************************************************************************

  Function    : ip_sock_add_ip
  Description : add ip stack to socket driver

  Parameter:
	hIp		: handle of ip stack

  Return Value: -

*********************************************************************************/
void sock_set_ip(IP_STACK_H hIp)
{
	ipSockInt.hIpList = hIp;	// set ptr to first ip stack
}

void freeWaitSocket(SOCK_PTR s)
{
	IP_LOCK_LEVEL_VAR

	IP_LOCK_LEVEL_ON

	// mark connection as closed if no frame was received for a certain time
	// (dont if socket was not taken by the function socket() at the meantime)
	if(s->header.state==IP_FIN_WAIT_1 || s->header.state==IP_FIN_WAIT_2 || s->header.state==IP_TIME_WAIT) ip_socket_free(s, IP_FREE);

	IP_LOCK_LEVEL_OFF
}

static SOCK_PTR findSock(IP_STACK_H hIp, unsigned char state)
{
	SOCK_PTR sock;
	IP_LOCK_LEVEL_VAR

	for(sock = hIp->sock; sock < hIp->sock + IP_TCP_SOCKETS ; sock++)
	{
		IP_LOCK_LEVEL_ON

		if(sock->header.state == state)	// socket with required state found
		{
			freeWaitSocket(sock);
			sock->header.state = IP_CLOSED;	// mark as used (but still closed)
			IP_LOCK_LEVEL_OFF
			return sock;
		}
		IP_LOCK_LEVEL_OFF
	}

	return 0;
}

/*********************************************************************************

Function    : socket
Description : get free tcpip/UDP socket

Parameter:
hIp		: handle of used interface

Return Value:
new socket or INVALID_SOCKET
*********************************************************************************/
SOCKET	socket(int af, int type, int protocol)
{
	IP_STACK_H		hIp;
	SOCK_PTR		sock;

	// only AF_INET supported
	if(af != AF_INET) 
	{
		assert(0);
		ipSockInt.lastError = WSAEAFNOSUPPORT;
		return INVALID_SOCKET;
	}

	// access to the only available interface (to be changed if more than 1 interface should be supported
	hIp = ipSockInt.hIpList;

	
	sock = findSock(hIp, IP_FREE); // search for free socket

	if(sock==0) sock = findSock(hIp, IP_TIME_WAIT);		// otherwise try if socket in TIME_WAIT-State is available
	if(sock==0) sock = findSock(hIp, IP_FIN_WAIT_2);	// otherwise try other wait-states
	if(sock==0) sock = findSock(hIp, IP_FIN_WAIT_1);

	if(sock==0)
	{
		ipSockInt.lastError = WSAEMFILE;
		//assert(0);
		IP_STAT(hIp->stat.tcp_sock_full++);
		return INVALID_SOCKET;	// no free socket found
	}

	// clear socket structure
	memset(&sock->header + 1, 0 , sizeof(*sock) - sizeof(sock->header) );

	sock->type	= type;
	sock->hIp	= hIp;

	sock->header.cntOpen++;

	return (SOCKET)sock;	// return address to user
}

/*********************************************************************************

  Function    : bind
  Description : bind socket to a address (preparation for listen()) function

  Parameter:
	socket	: socket will be switched to listen mode (only if not already connected)
	addr	: ptr to socket address structure

  Return Value:
	0  ... no error
	SOCKET_ERROR ... error (parameters wrong)
*********************************************************************************/

int bind(SOCKET s, struct sockaddr *addr)
{
	SOCK_PTR			sock = (SOCK_PTR)s, sockSearch;
    IP_STACK_H			hIp;
    struct sockaddr_in	*pAddr = (struct sockaddr_in*)addr;
    int                 i;
    unsigned short      newPort;
    
	if(sock == 0 || sock->header.state == IP_FREE || addr->sa_family != AF_INET)
	{
		RET_SOCK_ERROR(WSAENOTSOCK);
	}
  
	hIp = sock->hIp;

	if(sock->lport) RET_SOCK_ERROR(WSAEINVAL);	// already bound

	// reject if address is not ANY or my own
	if(pAddr->sin_addr.S_un.S_addr!=ADDR_ANY && pAddr->sin_addr.S_un.S_addr!=hIp->local_ip_addr.S_un.S_addr)
	{
		RET_SOCK_ERROR(WSAEADDRNOTAVAIL);
	}

	if(sock->type == SOCK_DGRAM)
	{
        // UDP or IP
		sock = sock;
	}

    sock->lport = pAddr->sin_port;

    if(sock->lport==0)  // user wants stack to search for free port number
    {
        newPort = hIp->nextFreePort;

        do
        {
            if(newPort < IP_DYNAMIC_PORT_RANGE) newPort = IP_DYNAMIC_PORT_RANGE;

            // search if port is already used by another socket
            i = 0;
            for(sockSearch = hIp->sock; sockSearch < hIp->sock + IP_TCP_SOCKETS; sockSearch++)
            {
                if(sockSearch->lport == newPort)
                {
                    i=1;		// port already used
                    newPort++;	// try next port
                }
            }
        }while(i);	// repeat this loop, till i=0 (which means port not found in current socket list)

        sock->lport         = newPort;
        hIp->nextFreePort   = newPort+1;
    }

	sock->header.state = IP_BOUND;

	return 0;
}


/*********************************************************************************

  Function    : listen
  Description : start listening on socket

  Parameter:
	socket	: socket will be switched to listen mode (only if not already connected)

  Return Value:
	0  ... no error
	SOCKET_ERROR ... error
*********************************************************************************/

int listen(SOCKET s)
{
	SOCK_PTR sock = (SOCK_PTR)s, searchSock;

	if(sock == 0)						RET_SOCK_ERROR(WSAENOTSOCK);	// socket invalid
	if(sock->header.state != IP_BOUND)	RET_SOCK_ERROR(WSAEINVAL);		// socket not bound
	if(sock->cmdClose)					RET_SOCK_ERROR(WSAESHUTDOWN);	// socket closing soon
	if(sock->type != SOCK_STREAM)		RET_SOCK_ERROR(WSAEOPNOTSUPP);	// listen only for stream supported

	// search for free socket
	for(searchSock = sock->hIp->sock ; searchSock < sock->hIp->sock + IP_TCP_SOCKETS ; searchSock++)
	{
		if(searchSock->header.state == IP_LISTEN && sock->lport == searchSock->lport && searchSock != sock)
		{
			// TODO ... verhalten kann mit SO_REUSEADDR gesteuert werden
			RET_SOCK_ERROR(WSAEADDRINUSE);
		}
	}

	sock->rport = 0;
	sock->header.state = IP_LISTEN;	// set socket to listen mode
	return 0;
}


/*********************************************************************************

  Function    : accept
  Description : accept a connection on a listening socket

  Parameter:
	socket	: socket will be switched to listen mode (only if not already connected)

  Return Value:
	new socket with connection
	0
	INVALID_SOCKET ... error (another socket is already listening on this port)
*********************************************************************************/

SOCKET accept(SOCKET s, struct sockaddr *addr, int *addrlen)
{
	ip_buf_type			*pBuf;
	SOCK_PTR			sock = (SOCK_PTR)s,newSock;
	struct sockaddr_in	*pAddr = (struct sockaddr_in*)addr;

	if(sock == 0)						RET_SOCK_INVALID(WSAENOTSOCK);	// socket invalid
	if(sock->type != SOCK_STREAM)		RET_SOCK_INVALID(WSAEOPNOTSUPP);	// listen only for stream supported

	// if the last accept call created a new socket this call is used to enable the next connection establishment
	// (result: if the user of the socket does not call accept() anymore because the application has reached its
	//  maximum number of connections all incoming connection requests will be rejected)
	if(sock->len)
	{
		sock->len	= 0;
		sock->rport	= 0;
	}

	// socket not in use or no connection pending or socket not listening
	if(sock->header.state != IP_LISTEN)	RET_SOCK_INVALID(WSAEINVAL);
	if(sock->rport==0)					RET_SOCK_INVALID(WSAEWOULDBLOCK);

	// check if a buffer is available for the synack
	pBuf = ip_alloc_tx_buffer(sock->hIp);

	if(pBuf == 0) RET_SOCK_INVALID(WSAENOBUFS);

	// search for free socket
	newSock = (SOCK_PTR)socket(AF_INET,SOCK_STREAM,0);

	if(newSock==INVALID_SOCKET)
	{
		pBuf->header.state = IP_BUF_STATE_IDLE;
		RET_SOCK_INVALID(ipSockInt.lastError);
	}

	// Fill in the necessary fields for the new connection
	sock->rto		= IP_RTO,
	sock->timer		= IP_RTO,
	sock->sv		= 4;
	sock->sa		= 0;
	sock->nrtx		= 0;
	sock->len		= 1;
	sock->recvLen	= 0;
	sock->snd_nxt	= sock->hIp->time_s;	// use second counter as start sequence
	
	// copy content of listening socket to new socket (without modifying the counters)
	memcpy(&newSock->header + 1, &sock->header+1 ,sizeof(*sock) - sizeof(sock->header));

	if(pAddr)
	{
		pAddr->sin_family		= AF_INET;
		pAddr->sin_port			= sock->rport;
		pAddr->sin_addr.s_addr	= sock->ripaddr.s_addr;
	}

	newSock->header.state = IP_SYN_RCVD;				// start socket state machine

	sock->rport = 0;							// free listen socket for next connection establishment
	
	ip_tcp_send(newSock, pBuf, TCP_SYN | TCP_ACK);	// send synack

	return (SOCKET)newSock;
}

/*********************************************************************************

Function    : getsockname
Description : Get socket information (local port and IP)

Parameter:

Return Value:
0
SOCKET_ERROR ... error
*********************************************************************************/
int getsockname(SOCKET s, struct sockaddr *addr, int *addrlen)
{
	SOCK_PTR			sock = (SOCK_PTR)s;
	struct sockaddr_in	*pAddr = (struct sockaddr_in*)addr;

	if(sock==0)			RET_SOCK_ERROR(WSAENOTSOCK);	// socket invalid
	if(pAddr==0)		RET_SOCK_ERROR(WSAEFAULT);
	if(sock->lport==0)	RET_SOCK_ERROR(WSAEINVAL);

	pAddr->sin_family		= AF_INET;
	pAddr->sin_port			= sock->lport;
	pAddr->sin_addr.s_addr	= sock->hIp->local_ip_addr.s_addr;

	return NULL;
}

/*********************************************************************************

Function    : getpeername
Description : Get socket information (remote port and IP)

Parameter:

Return Value:
0
SOCKET_ERROR ... error
*********************************************************************************/
int getpeername(SOCKET s, struct sockaddr *addr, int *addrlen)
{
	SOCK_PTR			sock = (SOCK_PTR)s;
	struct sockaddr_in	*pAddr = (struct sockaddr_in*)addr;

	if(sock==0)			RET_SOCK_ERROR(WSAENOTSOCK);	// socket invalid
	if(pAddr==0)		RET_SOCK_ERROR(WSAEFAULT);
	if(sock->rport==0)	RET_SOCK_ERROR(WSAENOTCONN);

	pAddr->sin_family		= AF_INET;
	pAddr->sin_port			= sock->rport;
	pAddr->sin_addr.s_addr	= sock->ripaddr.s_addr;

	return NULL;
}

/*********************************************************************************

  Function    : send
  Description : send data to a tcp/ip connection

  Parameter:
	socket	: socket created with socket(...)
	pData	: ptr to data to be sent
	size	: total number of bytes which should be sent

  Return Value:
	>= 0 ... number of bytes which were sent
	SOCKET_ERROR ... error
*********************************************************************************/

int send(SOCKET s, const char *buf, long len)
{
	SOCK_PTR	sock = (SOCK_PTR)s;
	ip_buf_type *pBuf;
	int         maxLen;

	if(sock == 0)						RET_SOCK_ERROR(WSAENOTSOCK);	// socket invalid
	if(sock->header.state <= IP_CLOSED)	RET_SOCK_ERROR(WSAENOTCONN);	// socket closed
	if(sock->cmdClose)					RET_SOCK_ERROR(WSAESHUTDOWN);	// socket closing soon
	
	if(sock->type == SOCK_DGRAM)	// dgram socket not connected
	{
		if(sock->header.state != IP_CONNECTED) RET_SOCK_ERROR(WSAENOTCONN);

		// send on dgram not yet implemented
		assert(0);
		RET_SOCK_ERROR(WSAENOTCONN);
	}

	if( (sock->header.state != IP_CONNECTED)	// not yet established
		|| (sock->mss==0)				// mss not yet negotiated
		|| (len == 0)					// no data for sending
		)
	{
		RET_SOCK_ERROR(WSAEWOULDBLOCK);
	}

	if(sock->pTx)  // last packet was not sent yet
	{
		if(sock->pTx->header.state != IP_BUF_STATE_TX)		RET_SOCK_ERROR(WSAEWOULDBLOCK);  // last buffer already in tx queue

		if(0 == (sock->flags & SOCK_FLAG_PACK_SEND_DATA))	RET_SOCK_ERROR(WSAEWOULDBLOCK);  // packing not allowed for this socket

		// add data to existing buffer if possible
		pBuf   = sock->pTx;
		maxLen = sock->mss - pBuf->header.dataSize;
	}
	else
	{
		if(sock->len) RET_SOCK_ERROR(WSAEWOULDBLOCK);	// last sequence not yet acknowledged

		pBuf = ip_alloc_tx_buffer(sock->hIp);			// try to allocate a new send buffer

		if(pBuf==0) RET_SOCK_ERROR(WSAEWOULDBLOCK);		// check if buffer was available

		maxLen                = sock->mss;
		pBuf->header.dataSize = 0;
	}

	if(len > maxLen)
	{
		len = maxLen;	// limit size to maximum data size of this telegram
	}
	else
	{
		pBuf->header.push = 1;	// set flag to tell transmitter to add the push-flag
	}

	// copy data to tx buffer
	memcpy(((char*)&pBuf->data.frame.prot.ip) + sizeof(ip_hdr)+sizeof(tcp_hdr) + pBuf->header.dataSize , buf, len);

	pBuf->header.dataSize = pBuf->header.dataSize + (unsigned short)len;	// enter tcp size to buffer header

	sock->pTx = pBuf;	// set address to socket structure

	return len;		// return the number of copied bytes
}

/*********************************************************************************

  Function    : sendto
  Description : send data to a UPD or IP connection

  Parameter:
  socket	: socket created with socket(...)
  pData		: ptr to data to be sent
  size		: total number of bytes which should be sent

  Return Value:
    >= 0 ... number of bytes which were sent
    SOCKET_ERROR ... error
*********************************************************************************/

int sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr* to, int tolen)
{
	SOCK_PTR	sock = (SOCK_PTR)s;
	ip_buf_type *pBuf;
	void		*ptr;

	if(sock == 0)						RET_SOCK_ERROR(WSAENOTSOCK);		// socket invalid
	if(sock->header.state <= IP_CLOSED)	RET_SOCK_ERROR(WSAENOTCONN);		// socket closed
	if(sock->type != SOCK_DGRAM)		RET_SOCK_ERROR(WSAEAFNOSUPPORT);	// function for stream sockets not supported

	// send UDP packet to specified address
	// check maximum send length
	if(len > IP_MTU - sizeof(ip_hdr)-sizeof(udp_hdr)) RET_SOCK_ERROR(WSAENOBUFS);

	pBuf = ip_alloc_tx_buffer(sock->hIp);		// try to allocate a new send buffer

	if(pBuf==0) RET_SOCK_ERROR(WSAEWOULDBLOCK);	// check if buffer was available

	// prepare IP header
	ptr = &pBuf->data.frame.prot.ip;

	IP(ptr)->len	= sizeof(ip_hdr) + sizeof(udp_hdr) + len;
	IP(ptr)->proto	= IPPROTO_UDP;

	copy_ip_address(IP(ptr)->dst_ip, &((struct sockaddr_in*)to)->sin_addr.s_addr);

	// prepare udp frame and send
	ptr = (udp_hdr*)(((char*)ptr) + sizeof(ip_hdr));

	memcpy( UDP(ptr)+1 , buf, len);			// copy udp data to send buffer

	UDP(ptr)->dst_port	= htons(((struct sockaddr_in*)to)->sin_port);
	UDP(ptr)->src_port	= htons(sock->lport);
	UDP(ptr)->len		= htons((unsigned short)(len + sizeof(udp_hdr)));
	UDP(ptr)->chksum	= 0;

	IP_STAT(sock->hIp->stat.udp_tx++);

	// send frame
	flags = TX_IP_HEADER;

#ifdef IP_SECONDARY_ADDRESS
	{
		// search if the destination IP is one of the IPs which connected to the 2nd IP Address
		int ix	= IP_SECONARY_HOST_QUEUE;
		ptr		= sock->hIp->ip2_remotehosts;

		while(ix--)
		{
			if(memcmp(ptr, &((struct sockaddr_in*)to)->sin_addr,4)==0 )	// ip in list found
			{
				flags = flags + TX_SECONDARY_IP;
				break;
			}

			ptr = ((struct in_addr*)ptr)+1;
		}
	}
#endif

	ip_buf_send(sock->hIp, pBuf, flags);	// send frame

	return len;
}


/*********************************************************************************

  Function    : recv
  Description : receive data from tcp-ip connection

  Parameter:
	socket	: socket created with socket(...)
	pData	: ptr to data buffer
	size	: maximum number of bytes in the data buffer

  Return Value:
	>= 0 ... number of received bytes
	SOCKET_ERROR  ... error
*********************************************************************************/

int recv(SOCKET s, void *buf, int len, int flags)
{
	return recvfrom(s,buf,len,flags,0,0);
}

/*********************************************************************************

  Function    : recvfrom
  Description : receive data from tcp-ip connection

  Parameter:
    socket	: socket created with socket(...)
    pData	: ptr to data buffer
    size	: maximum number of bytes in the data buffer

  Return Value:
    >= 0 ... number of received bytes
    SOCKET_ERROR  ... error
*********************************************************************************/
int recvfrom(SOCKET s, void *buf, int len, int flags, struct sockaddr *from, int *fromlen)
{
	SOCK_PTR	sock = (SOCK_PTR)s;
	long		copyLen;

	if(sock == 0)						RET_SOCK_ERROR(WSAENOTSOCK);	// socket invalid
	if(sock->header.state <= IP_CLOSED)	RET_SOCK_ERROR(WSAENOTCONN);	// socket closed
	if(sock->cmdClose)					RET_SOCK_ERROR(WSAESHUTDOWN);	// socket closing soon

	copyLen = len;

	if(copyLen > sock->recvLen) copyLen = sock->recvLen;	// available data is less than the user buffer

	if(copyLen==0) RET_SOCK_ERROR(WSAEWOULDBLOCK);			// no data available or user size = 0

	memcpy(buf, sock->pRecvData, copyLen);				// copy available bytes to user buffer

	if(from)
	{
		((struct sockaddr_in*)from)->sin_family			= AF_INET;
		((struct sockaddr_in*)from)->sin_port			= sock->rport;
		((struct sockaddr_in*)from)->sin_addr.s_addr	= sock->ripaddr.s_addr;
	}

	if(flags == MSG_PEEK) return sock->recvLen;				// return available rx-data if peek-option is set

	sock->recvLen = (unsigned short)(sock->recvLen - copyLen);	// set new receive len

	sock->pRecvData = sock->pRecvData + copyLen;				// set new receive pointer

	// release buffer immediately if this is a datagram socket
	if(sock->recvLen==0 && sock->type == SOCK_DGRAM && sock->pFctFree)
	{
		// free receive buffer
		sock->pFctFree(GET_TYPE_BASE(ip_packet_typ, data, sock->pRx));
		sock->pRx = 0;
		sock->recvLen = 0;
	}

	return copyLen;											// return number of copied bytes
}

#if IP_ACTIVE_OPEN==1
/*********************************************************************************

Function    : connect
Description : connect to another host

Parameter:
socket	: socket created with socket(...)

Return Value:
0  ... no error

*********************************************************************************/
int connect(SOCKET s, const struct sockaddr* name, int namelen)
{
	SOCK_PTR			sock = (SOCK_PTR)s;
	IP_STACK_H			hIp = sock->hIp;
	struct sockaddr_in	addr;

	if(ipSockInt.mss==0)					RET_SOCK_ERROR(WSANOTINITIALISED);	// socket invalid
	if(sock == 0)							RET_SOCK_ERROR(WSAENOTSOCK);		// socket invalid
	if(namelen < sizeof(*name))				RET_SOCK_INVALID(WSAEFAULT);
	if(sock->header.state == IP_LISTEN)		RET_SOCK_INVALID(WSAEINVAL);		// listening socket
	if(sock->header.state == IP_CONNECTED)	RET_SOCK_INVALID(WSAEISCONN);		// already connected

    if(sock->header.state == IP_BOUND)
    {
        // already bound .. no action required
    }
	else if(sock->header.state == IP_CLOSED)
	{
		// bind with unique address

		addr.sin_family = AF_INET;
		addr.sin_addr.S_un.S_addr = ADDR_ANY;
		addr.sin_port = 0;
		bind(s, (struct sockaddr*)&addr);
	}
	else	// wrong state (maybe in connect already)
	{
		RET_SOCK_INVALID(WSAEALREADY);
	}

	// overtake remote address and port
	memcpy(&sock->ripaddr, &((struct sockaddr_in*)name)->sin_addr, 4);

	#if IP_LOOPBACK==1
		// convert loopback to local IP address
		if(sock->ripaddr.S_un.S_un_b.s_b1 == 127) sock->ripaddr.S_un.S_addr = hIp->local_ip_addr.S_un.S_addr;
	#endif


	sock->rport = ((struct sockaddr_in*)name)->sin_port;

	sock->snd_nxt		= hIp->time_s;
	sock->initialmss	= sock->mss = ipSockInt.mss;
	sock->header.state	= IP_SYN_TX;

	RET_SOCK_INVALID(WSAEWOULDBLOCK);
}

/*********************************************************************************

Function    : select
Description : get information about a list of sockets

Parameter:
see microsoft help

Return Value:

*********************************************************************************/
int select(int nfds, fd_set* readfds, fd_set* writefds, fd_set* exceptfds, const struct timeval* timeout)
{
	int selected = 0,i;
	SOCK_PTR	s;

	if (ipSockInt.mss == 0)							RET_SOCK_ERROR(WSANOTINITIALISED);
	if (readfds==0 && writefds==0 && exceptfds==0)	RET_SOCK_ERROR(WSAEINVAL);

	// check sockets if they are read for reading
	if(readfds)
	{
		for(i=0;i<readfds->fd_count;i++)
		{
			s = (SOCK_PTR)readfds->fd_array[i];
			if(s==INVALID_SOCKET || s==0) RET_SOCK_ERROR(WSAENOTSOCK);

			if(
				(s->header.state==IP_LISTEN  &&  s->type==SOCK_STREAM  &&  s->rport>0)	// user can call accept on this socket
				||
				(s->header.state==IP_CONNECTED  && s->recvLen>0)		// user can call recv on this socket
				)
			{
				selected++;
			}
			else
			{
				readfds->fd_array[i] = 0;
			}
		}
	}

	// check sockets if they are read for writing
	if(writefds)
	{
		for(i=0;i<writefds->fd_count;i++)
		{
			s = (SOCK_PTR)writefds->fd_array[i];
			if(s==INVALID_SOCKET || s==0) RET_SOCK_ERROR(WSAENOTSOCK);

			if(
				(s->header.state==IP_CONNECTED  &&  s->type==SOCK_STREAM)		// user can call recv on this socket
				)
			{
				selected++;
			}
			else
			{
				writefds->fd_array[i] = 0;
			}
		}
	}

	// check sockets if they are in error state (or not connected which is an error state in this case)
	if(exceptfds)
	{
		for(i=0;i<exceptfds->fd_count;i++)
		{
			s = (SOCK_PTR)exceptfds->fd_array[i];
			if(s==INVALID_SOCKET || s==0) RET_SOCK_ERROR(WSAENOTSOCK);

			if(s->header.state <= IP_CLOSED)
			{
				selected++;
			}
			else
			{
				exceptfds->fd_array[i] = 0;
			}
		}
	}

	return selected;
}

// returns 0 if socket s is not in fd_set
int	__WSAFDIsSet(SOCKET s, fd_set* set)
{
	int i;
	for(i=0 ; i<set->fd_count ; i++)
	{
		if(s == set->fd_array[i]) return 1;	// s is a member of set
	}

	return 0;
}

#endif

/*********************************************************************************

  Function    : closesocket
  Description : close a socket

  Parameter:
	socket	: socket created with socket(...)

  Return Value:
	0  ... no error
	SOCKET_ERROR ... error
*********************************************************************************/

int closesocket(SOCKET s)
{
	SOCK_PTR sock = (SOCK_PTR)s;
	IP_LOCK_LEVEL_VAR

	if(sock==0)							RET_SOCK_ERROR(WSAENOTSOCK);	// socket invalid
	if(sock->header.state == IP_FREE)	return 0;						// socket already closed

	if(sock->type == SOCK_DGRAM)
	{
		ip_socket_free(sock, IP_FREE);
		return 0;
	}

	IP_LOCK_LEVEL_ON

	if(sock->cmdClose)
	{
		IP_LOCK_LEVEL_OFF
		return 0;		// already about to close
	}


	// release rx buffer
	if(sock->pRx)
	{
		if(sock->pFctFree)
		{
			sock->pFctFree( GET_TYPE_BASE(ip_packet_typ, data, sock->pRx) );
		}

		sock->pRx		= 0;
		sock->recvLen	= 0;
	}

	// free socket if in listening mode
	if(sock->header.state == IP_LISTEN || sock->header.state == IP_CLOSED)
	{
		ip_socket_free(sock, IP_FREE);
	}
	else
	{
		// other active sockets just get a command to start close sequence
		sock->cmdClose = 1;
	}
	IP_LOCK_LEVEL_OFF

	return 0;
}

/*********************************************************************************

  Function    : ioctlsocket
  Description : control I/O mode of socket

  Parameter:
    socket	: socket created with socket(...)
    cmd		: only FIONBIO allowed
    *argp	: ptr to argument

  Return Value:
    0  ... no error
    SOCKET_ERROR ... error

*********************************************************************************/
int ioctlsocket(SOCKET s, long cmd, unsigned long *argp)
{
	if(s==0)	RET_SOCK_ERROR(WSAENOTSOCK);
	if(argp==0)	RET_SOCK_ERROR(WSAEFAULT);

	if(cmd != FIONBIO)	// vorerst wird nur dieses Command unterstützt
	{
		assert(0);
		RET_SOCK_ERROR(WSAENOTSOCK);
	}

	if(*argp == 0)	// FIONBIO kann nicht abgeschaltet werden ...
	{
		assert(0);
		RET_SOCK_ERROR(WSAENOTSOCK);
	}

	return 0;
}


int setsockopt(SOCKET s, int level, int optname, const char  * optval, int optlen)
{
	IP_CONN	*sock = (IP_CONN*)s;
	long	val = 0;

	if( (size_t)optval < 100 )
	{
		val = (size_t)optval;	// take address as value if < 100 (such address is not possible)
	}
	else if ( optlen>=1  &&  optlen<=4 )
	{
		memcpy( ((char*)&val) + 3 - optlen , optval, optlen );	// get value
	}


	switch(level)
	{
		case SOL_SOCKET:
			switch(optname)
			{
				case SO_REUSEADDR:
				case SO_KEEPALIVE:
				case SO_SNDBUF:
				case SO_RCVBUF:
				case SO_LINGER:
					// TODO : Fürs erste nicht so wichtig
					break;

				default:
					assert(0);
					RET_SOCK_ERROR(WSAEOPNOTSUPP);
					break;
			}
			break;

		case IPPROTO_IP:
			switch(optname)
			{
				case IP_MULTICAST_TTL:
					// TODO : ttl für Multicasts muss übernommen werden !
					break;

				case IP_MULTICAST_IF:	// defines the interface where the multicast should be sent
					// we just ignore this option because we only send it to 1 interface anyways
					break;
			}

			break;

		case IPPROTO_TCP:

			// this must be a stream socket !
			if(sock->type != SOCK_STREAM) RET_SOCK_ERROR(WSAEOPNOTSUPP);

			switch(optname)
			{
				case TCP_ACK_REQUEST:
					if(val)	sock->flags = sock->flags | SOCK_FLAG_ACKREQ;
					else	sock->flags = sock->flags & (unsigned short)~SOCK_FLAG_ACKREQ;
					break;

				case TCP_REJECT_SEGMENT:
					if(val)	sock->flags = sock->flags | SOCK_FLAG_REJECT_SEGMENT;
					else	sock->flags = sock->flags & (unsigned short)~SOCK_FLAG_REJECT_SEGMENT;
					break;

				case TCP_PACK_SEND_DATA:
					if(val)	sock->flags = sock->flags | SOCK_FLAG_PACK_SEND_DATA;
					else	sock->flags = sock->flags & (unsigned short)~SOCK_FLAG_PACK_SEND_DATA;
					break;
			}
			break;

		default:
			//assert(0);
			RET_SOCK_ERROR(WSAEPROTONOSUPPORT);
	}

	return 0;
}

/*********************************************************************************

  Function    : ip_socket_free
  Description : free resources of a socket

  Parameter:
	socket	: socket created with socket(...)

*********************************************************************************/

void ip_socket_free(SOCK_PTR s, int state)
{
	// release any currently used buffers
	if(s->pTx && state == IP_FREE)
	{
	    EnableGlobalInterrupt(FALSE);

		switch(s->pTx->header.state)
		{
			case IP_BUF_STATE_TX_Q:	// nothing to do, buffer will be cleared automatically after transmission
				break;

			case IP_BUF_STATE_TX_ACK_Q:	// change to TX_Q ... free buffer automatically after transmission (only if socket is set to FREE)
				s->pTx->header.state = IP_BUF_STATE_TX_Q;
				break;

			default:	// in all other cases free buffer immediately
				s->pTx->header.push		= 0;
				s->pTx->header.state	= IP_BUF_STATE_IDLE;
				break;
		}

		EnableGlobalInterrupt(TRUE);
	}

	IP_LOCK_LEVEL_ON

	// release rx buffer
	if(s->pRx)
	{
		if(s->pFctFree)
		{
			s->pFctFree( GET_TYPE_BASE(ip_packet_typ, data, s->pRx) );
		}

		s->pRx = 0;
		s->recvLen = 0;
	}

	if(state == IP_FREE) s->header.cntFree++;

	s->header.state = state;

	IP_LOCK_LEVEL_OFF
}

/*********************************************************************************

  Function    : parse_mss
  Description : find mss option and return if found

  Return Value:
	mss (or own MSS if found MSS is higher than the own or no mss option found)

*********************************************************************************/

int parse_mss(unsigned char *pOpt, int len)
{
	int i,c,mss;

	for(i=0;i<len;i++)	// process all option bytes
	{
		c = pOpt[i];			// get byte
		
		if(c==0) return ipSockInt.mss;	// end of options
		
		if(c==1) continue;		// NOP option
		
		if(c==2 && pOpt[i+1]==4)	// An MSS option with the right option length
		{
			pOpt = pOpt+i+2;		// access to mss field
			
			mss = (((unsigned short)pOpt[0]) << 8) + pOpt[1];
			
			if(mss > ipSockInt.mss) mss = ipSockInt.mss;	// take the own mss if the found one is too high

			return mss;
		}
		else
		{
			// All other options have a length field, so that we easily can skip past them
			c = pOpt[i+1];			// get length byte

			if(c==0) return ipSockInt.mss;

			i=i+c-1;				// jump over this option
		}
	}
	return ipSockInt.mss;
}



/*********************************************************************************

  Function    : sock_out
  Description : function is called in the stacks periodic function to send
				packets to the ethernet layer

*********************************************************************************/

void sock_out(IP_STACK_H hIp)
{
	SOCK_PTR		sock;
	int				ackRequest;

	// loop through all stream sockets to see if there is one which has tx data
	for(sock = hIp->sock; sock < hIp->sock + IP_TCP_SOCKETS; sock++)
	{
		if(sock->type != SOCK_STREAM)		continue;
		if(sock->header.state <= IP_CLOSED)	continue;

		ackRequest = 0;

		if(sock->pRx && sock->recvLen==0)
		{
			// free receive buffer
			if(sock->pFctFree) sock->pFctFree(GET_TYPE_BASE(ip_packet_typ, data, sock->pRx));
			sock->pRx = 0;				// reset receive buffer
			sock->recvLen = 0;
		}
		else if( sock->pTx && sock->pTx->header.state == IP_BUF_STATE_TX )
		{
			// send response with ack and data (followed by ack-request if configured)
			if(sock->flags & SOCK_FLAG_ACKREQ) ackRequest = 1;
		}
		else if(sock->cmdClose == 1)
		{
			// send
		}
		else
		{
			continue;	// don't send
		}

		ip_tcp_appsend(sock);

		if( ackRequest && (sock->header.state != IP_LAST_ACK) ) ip_tcp_send(sock, 0, TCP_ACK);
	}

}


/*********************************************************************************

  Function    : sock_in
  Description : function is called by ip stack if a tcp packet is received

  returns:
	IP_FRAME_QUEUED			frame queued
	IP_FRAME_RETRY			socket full, leave frame in rx queue
	IP_FRAME_UNUSED			frame not used because no socket for this frame or it was a reset frame
							or a frame with invalid checksum

*********************************************************************************/
int sock_in(IP_STACK_H hIp, eth_frame *pFrame, unsigned short ipHdrLen)
{
	tcp_hdr			*pTCP;
	udp_hdr			*pUDP;
	SOCK_PTR		sock,listenSock;
	struct in_addr	ipAddr;
	unsigned int	dataLen,tcpHdrLen,response;
	unsigned int	ret = IP_FRAME_UNUSED;
	unsigned long	seq,ack;
	ip_buf_type		*pBuf;
	IP_LOCK_LEVEL_VAR

	copy_ip_address(&ipAddr , pFrame->prot.ip.src_ip);

	#if IP_LOOPBACK==1
		// convert ip address of loopback range to own ip address
		if(ipAddr.S_un.S_un_b.s_b1 == 127) ipAddr.S_un.S_addr = hIp->local_ip_addr.S_un.S_addr;
	#endif

	if(pFrame->prot.ip.proto == IPPROTO_UDP)
	{
		pUDP = (udp_hdr*)((char*)&pFrame->prot.ip + ipHdrLen);
		
		IP_STAT(hIp->stat.udp_rx++);

		// search for socket which is bound to this UDP port
		for(sock = hIp->sock; sock < hIp->sock + IP_TCP_SOCKETS; sock++)
		{
			if(sock->type != SOCK_DGRAM)		continue;	// not a DGRAM socket (UDP)
			if(sock->header.state <= IP_CLOSED)	continue;	// closed ... can not receive data, bind first
            if(pUDP->dst_port != sock->lport)	continue;	// not for the bound port

			// do not process sockets with pending rx data (not yet read by the user)
			if(sock->recvLen) return IP_FRAME_RETRY;

			sock->rport				= pUDP->src_port;
			sock->ripaddr.s_addr	= ipAddr.s_addr;

			// overtake callback address from the first 4 bytes of the frame
			sock->pFctFree = ((ip_int_hdr*)pFrame)->pFct;

			// clear callback function in frame to mark as queued
			((ip_int_hdr*)pFrame)->pFct = 0;

			sock->pRx		= pFrame;
			sock->pRecvData	= (char*)pUDP + sizeof(udp_hdr);
			sock->recvLen	= htons(pFrame->prot.ip.len) - ipHdrLen - sizeof(udp_hdr);

			return IP_FRAME_QUEUED;
		}
        

		return IP_FRAME_UNUSED;	// frame not used by this instance
	}

	pTCP = (tcp_hdr*)((char*)&pFrame->prot.ip + ipHdrLen);

	IP_STAT( hIp->stat.tcp_rx++);

	// Start of TCP input header processing code
	if(ip_chksum(&pFrame->prot.ip, IPPROTO_TCP))
	{
		// Compute and check the TCP checksum
		IP_STAT( hIp->stat.err_tcp_chksum++ );
		IP_LOG("tcp: bad checksum.");
		return IP_FRAME_UNUSED;
	}

	// calculate header length
	tcpHdrLen	= (pTCP->tcpoffset & 0xF0) >> 2;
	dataLen		= htons(pFrame->prot.ip.len) - ipHdrLen - tcpHdrLen;

	// get sequence number into temp variable
	htonlc(&seq, pTCP->seqno);
	htonlc(&ack, pTCP->ackno);

	// check if there is a socket for this connection
	listenSock = 0;
	for(sock = hIp->sock; sock < hIp->sock + IP_TCP_SOCKETS; sock++)
	{
		// only consider stream sockets
		if(sock->type != SOCK_STREAM) continue;

		// not interesting at all if port is wrong
		if(pTCP->dst_port != sock->lport) continue;

		// listen port found
		if(sock->header.state == IP_LISTEN)
		{
			// remember this socket if there is no connection pending at the moment
			if(sock->rport==0) listenSock = sock;
			continue;
		}

		// do not handle closed sockets
		if(sock->header.state <= IP_CLOSED) continue;

		// connection found if port and ip are equal to the ones stored in the socket
		if(pTCP->src_port == sock->rport && ipAddr.S_un.S_addr == sock->ripaddr.S_un.S_addr )
		{
			sock->ack = 0;

			// about to close and fin not set
			if( (sock->cmdClose!=0) && ((pTCP->flags&TCP_FIN)==0)  ) return IP_FRAME_UNUSED;

			// SYN or RST or FIN flag on socket which is already closed ?
			if(pTCP->flags & (TCP_SYN|TCP_RST|TCP_FIN) )
			{
				if(sock->header.state == IP_TIME_WAIT)
				{
					freeWaitSocket(sock);
					if(pTCP->flags & TCP_SYN) IP_STAT( hIp->stat.port_reused++ );
					continue;
				}
				
				if(sock->header.state == IP_LAST_ACK)
				{
					ip_socket_free(sock,IP_CLOSED);
					IP_STAT( hIp->stat.port_reused++ );
					continue;
				}

				// We do a very naive form of TCP reset processing; we just accept any RST and kill
				// our connection. We should in fact check if the sequence number of
				// this reset is within our advertised window before we accept the reset
				if(pTCP->flags & TCP_RST)
				{
					IP_LOCK_LEVEL_ON

					if(sock->cmdClose)
					{
						ip_socket_free(sock, IP_FREE);
						IP_LOCK_LEVEL_OFF
					}
					else
					{
						ip_socket_free(sock, IP_CLOSED);
						IP_LOCK_LEVEL_OFF

						IP_LOG("tcp: got reset, aborting connection.");
					}


					return IP_FRAME_UNUSED;	// drop packet
				}

				// open connections should never receive TCP_SYN without TCP_ACK
				// (this would mean a client wants to open a connection to an existing socket)
				// TCP_SYN+TCP_ACK is allowed (when establishing client sockets)
				if( (pTCP->flags & (TCP_SYN|TCP_ACK)) == TCP_SYN )
				{
					IP_STAT( hIp->stat.port_reused_invalid++ );
					return IP_FRAME_UNUSED;	// frame not used by this instance
				}
			}

			// Next, check if the incoming segment acknowledges any outstanding data.
			// If so, we update the sequence number, reset the length of the outstanding data,
			// calculate RTT estimations, and reset the retransmission timer
			if((pTCP->flags & TCP_ACK) && sock->len)
			{
				if(ack == sock->snd_nxt + sock->len)
				{
					// Update sequence number
					sock->snd_nxt = ack;

					// Do RTT estimation, unless we have done retransmissions
					if(sock->nrtx == 0)
					{
						signed char m;

						m = sock->rto - sock->timer;
						
						// This is taken directly from VJs original code in his paper
						m = m - (sock->sa >> 3);
						sock->sa += m;
						if(m < 0) m = -m;
						m = m - (sock->sv >> 2);
						sock->sv += m;
						sock->rto = (sock->sa >> 3) + sock->sv;

						// make sure rto does not become too small, avoid unjustified retransmissions
						if(sock->rto < 2) sock->rto = 2;
					}

					sock->len	= 0;
					sock->ack	= 1;			// Set the acknowledged flag
					sock->timer	= sock->rto;	// Reset the retransmission timer

					// free tx buffer if sent successfully
					IP_LOCK_LEVEL_ON

					if(sock->pTx && sock->pTx->header.state == IP_BUF_STATE_TX_DONE)
					{
						pBuf = sock->pTx;
						sock->pTx = 0;

						pBuf->header.push	= 0;
						pBuf->header.state	= IP_BUF_STATE_IDLE;
					}

					IP_LOCK_LEVEL_OFF
				}
				else if (sock->header.state != IP_LAST_ACK)
				{
					// re-send last packet when retransmission request was received
					if(sock->pTx && sock->pTx->header.state == IP_BUF_STATE_TX_DONE)
					{
						sock->pTx->header.state = IP_BUF_STATE_TX;
						ip_tcp_appsend(sock);
					}
				}
			}

			#if (IP_ACTIVE_OPEN > 0)

				// SYNACK expected and received
				if( (sock->header.state == IP_SYN_SENT)
					&&
					( (pTCP->flags & (TCP_SYN|TCP_ACK)) == (TCP_SYN|TCP_ACK) )
				  )
				{
					sock->rcv_nxt	= seq+1;
					sock->snd_nxt	= ack;
					sock->len		= 0;

					pBuf = ip_alloc_tx_buffer(hIp);	// allocate packet for syn

					if(pBuf==0)
					{
						// no tx buffer available, try in periodic call
						sock->header.state	= IP_SYN_ACK_TX;
					}
					else
					{
						ip_tcp_send(sock, pBuf, TCP_ACK);
						sock->header.state = IP_CONNECTED;
					}

					return IP_FRAME_UNUSED;
				}

			#endif


			// check if the sequence number of the incoming packet is what we're
			// expecting next. If not, we send out an ACK with the correct numbers in.
			if(seq != sock->rcv_nxt)
			{
				if(sock->header.state != IP_LAST_ACK) ip_tcp_send(sock, 0, TCP_ACK);
				return IP_FRAME_UNUSED;
			}
			else if((sock->cmdClose==0) && (dataLen>0))	// if socket is going to be closed, don't queue packet
			{
				// do not process sockets with pending rx data (not yet read by the user)
				if(sock->recvLen)
				{
					// discard packet if reject flag is set, or if packet does not carry payload data
					if( (sock->flags & SOCK_FLAG_REJECT_SEGMENT) ) return IP_FRAME_UNUSED;

					// ACK was already processed, make sure it is not processed anymore when the retry is done
					pTCP->flags = pTCP->flags & ~TCP_ACK;
					pTCP->chksum = 0;
					pTCP->chksum = ip_chksum(&pFrame->prot.ip, IPPROTO_TCP);

					return IP_FRAME_RETRY;
				}

				// overtake callback address from the first 4 bytes of the frame
				sock->pFctFree = ((ip_int_hdr*)pFrame)->pFct;
				
				// clear callback function in frame to mark as queued
				((ip_int_hdr*)pFrame)->pFct = 0;
				
				sock->pRx		= pFrame;
				sock->pRecvData	= (char*)pTCP + tcpHdrLen;
				sock->rcv_nxt	+= dataLen;
				sock->recvLen	= dataLen;

				ret = IP_FRAME_QUEUED;
			}

			response = 0;	// response flag

			// Do different things depending on in what state the connection is
			switch(sock->header.state)
			{
				// CLOSED and LISTEN are not handled here. CLOSE_WAIT is not implemented, since we force the
				// application to close when the peer sends a FIN (hence the application goes directly from
				// ESTABLISHED to LAST_ACK)
				case IP_SYN_RCVD:
					if(pTCP->flags & TCP_FIN)
					{
						sock->rcv_nxt++;
						sock->len			= 1;
						sock->header.state	= IP_LAST_ACK;
						sock->nrtx			= 0;

						ip_tcp_send(sock, 0, TCP_FIN | TCP_ACK);
						return ret;
					}

					// switch to established if ack to our syn-ack was received
					if(sock->ack) sock->header.state = IP_CONNECTED;

					break;

				case IP_CONNECTED:
					/* If the incoming packet is a FIN, we should close the connection on
					this side as well, and we send out a FIN and enter the LAST_ACK
					state. We require that there is no outstanding data; otherwise the
					sequence numbers will be screwed up. */

					if(pTCP->flags & TCP_FIN)
					{
						if(sock->len) break;

						sock->rcv_nxt++;
						sock->len			= 1;
						sock->header.state	= IP_LAST_ACK;
						sock->nrtx			= 0;

						ip_tcp_send(sock, 0, TCP_FIN | TCP_ACK);
						return ret;
					}

#if 0
-> urgent_mode_not_supported
					// Check the URG flag. If this is set, the segment carries urgent
					// data that we must pass to the application
					if(pTCP->flags & TCP_URG)
					{
						ip_urglen = (BUF->urgp[0] << 8) | BUF->urgp[1];
					
						if(ip_urglen > ip_len)
						{
							// There is more urgent data in the next segment to come
							ip_urglen = ip_len;
						}
						ip_add_rcv_nxt(ip_urglen);
						ip_len -= ip_urglen;
						ip_urgdata = ip_appdata;
						ip_appdata += ip_urglen;
					}
					else
					{
						ip_urglen = 0;
						ip_appdata += (BUF->urgp[0] << 8) | BUF->urgp[1];
						ip_len -= (BUF->urgp[0] << 8) | BUF->urgp[1];
					}
#endif


					/* Check if the available buffer space advertised by the other end
					is smaller than the initial MSS for this connection. If so, we
					set the current MSS to the window size to ensure that the
					application does not send more data than the other end can
					handle.

					If the remote host advertises a zero window, we set the MSS to
					the initial MSS so that the application will send an entire MSS
					of data. This data will not be acknowledged by the receiver,
					and the application will retransmit it. This is called the
					"persistent timer" and uses the retransmission mechanism.
					*/
					sock->mss = htons(pTCP->wnd);

					if(sock->mss > sock->initialmss || sock->mss == 0) sock->mss = sock->initialmss;

					break;

				case IP_FIN_WAIT_1:
					// The application has closed the connection, but the remote host hasn't closed its end yet.
					// Thus we do nothing but wait for a FIN from the other side
					if(pTCP->flags & TCP_FIN)
					{
						if(sock->ack)
						{
							sock->header.state	= IP_TIME_WAIT;
							sock->timer			= 0;
							sock->len			= 0;
						}
						else
						{
							sock->header.state	= IP_CLOSING;
						}
					
						sock->rcv_nxt++;
					}
					else if(sock->ack)
					{
						sock->header.state	= IP_FIN_WAIT_2;
						sock->len			= 0;
						break;
					}
					else if(dataLen==0)
					{
						break;
					}

					ip_tcp_send(sock, 0, TCP_ACK);	// send ack

					break;

				case IP_FIN_WAIT_2:
					if(pTCP->flags & TCP_FIN)
					{
						sock->header.state	= IP_TIME_WAIT;
						sock->timer			= 0;
						sock->rcv_nxt++;
					}
					else if(dataLen==0)
					{
						break;
					}
					
					ip_tcp_send(sock, 0, TCP_ACK);	// send ack
					break;

				case IP_TIME_WAIT:
					break;

				case IP_CLOSING:
					if(sock->ack)
					{
						sock->header.state	= IP_TIME_WAIT;
						sock->timer			= 0;
					}
					break;

				case IP_LAST_ACK:
					// We can close this connection if the peer has acknowledged our FIN.
					if(sock->ack)
					{
						ip_socket_free(sock, IP_CLOSED);
					}

					break;

#if 0
->active_open_not_supported
				case IP_SYN_SENT:
					// In SYN_SENT, we wait for a SYNACK that is sent in response to our SYN.
					// The rcv_nxt is set to sequence number in the SYNACK plus one, and we
					// send an ACK. We move into the ESTABLISHED state
					if((socket->flags & IP_ACKDATA) && pTCP->flags == (TCP_SYN | TCP_ACK))
					{
						// Parse the TCP MSS option, if present
						listen_socket->initialmss = listen_socket->mss = 
							parse_mss(((unsigned char*)pTCP) + sizeof(tcp_hdr), tcpHdrLen - sizeof(tcp_hdr) );

						socket->state	= IP_CONNECTED;
						socket->rcv_nxt = seq+1;
						socket->len		= 0;

						IP_APPCALL();
						goto appsend;
					}

					goto reset;


					apprexmit:
					ip_appdata = ip_sappdata;

					/* If the application has data to be sent, or if the incoming
					packet had new data in it, we must send out a packet. */
					if(ip_slen > 0 && ip_connr->len > 0) {
					/* Add the length of the IP and TCP headers. */
					ip_len = ip_connr->len + IP_TCPIP_HLEN;
					/* We always set the ACK flag in response packets. */
					BUF->flags = TCP_ACK | TCP_PSH;
					/* Send the packet. */
					goto tcp_send_noopts;
					}
					/* If there is no data to send, just send out a pure ACK if
					there is newdata. */
					if(ip_flags & ) {
					ip_len = IP_TCPIP_HLEN;
					BUF->flags = TCP_ACK;
					goto tcp_send_noopts;
					}
					}
					goto drop;

#endif
			}

			// send response only if dataLen is 0, otherwise no response is required or the response will be
			// sent when the user has read all data
			if(response==1 && dataLen==0) ip_tcp_appsend(sock);

			return ret;
		}
	}

	// If we didn't find and active connection that expected the packet, either this packet
	// is an old duplicate, or this is a SYN packet destined for a connection in LISTEN.
	// If the SYN flag isn't set, it is an old packet and we send a RST.
	if((pTCP->flags & TCP_CTL) == TCP_SYN)
	{
		if(listenSock)	// overtake data to listen socket if free
		{
			// store connection info
			copy_ip_address(&listenSock->ripaddr, pFrame->prot.ip.src_ip);

			// rcv_nxt should be the seqno from the incoming packet + 1
			htonlc(&listenSock->rcv_nxt, pTCP->seqno);

			listenSock->rcv_nxt++;

			listenSock->pTx	= 0;
			listenSock->pRx	= 0;

			// Parse the TCP MSS option
			listenSock->initialmss = listenSock->mss = 
				parse_mss(((unsigned char*)pTCP) + sizeof(tcp_hdr), tcpHdrLen - sizeof(tcp_hdr) );

			listenSock->flags = 0;

#ifdef IP_SECONDARY_ADDRESS	// check if this request went to 2nd IP address
			if(memcmp(pFrame->prot.ip.dst_ip, &hIp->local_ip_addr2,4) == 0) listenSock->flags = SOCK_FLAG_SECONDARY_IP;
#endif

			listenSock->rport = pTCP->src_port;	// now listen socket is occupied and will wait for accept call
			return ret;
		}
		
		// No matching connection found, so we send a RST packet
		IP_STAT(hIp->stat.tcp_synrst++);
	}

	// We do not send resets in response to resets	
	if(pTCP->flags & TCP_RST) return ret;

	pBuf = ip_alloc_tx_buffer(hIp);		// get send buffer
	if(pBuf==0) return ret;

	// copy received data to transmit buffer
	memcpy(&pBuf->data, pFrame, sizeof(eth_hdr)+sizeof(tcp_hdr)+ipHdrLen);

	pTCP = (tcp_hdr*)((char*)&pBuf->data.frame.prot.ip + ipHdrLen);

	IP_STAT(hIp->stat.tcp_rst);

	pTCP->flags = TCP_RST | TCP_ACK;
	pTCP->tcpoffset = sizeof(tcp_hdr)*4;	// *4 to get the number of longs in the high nibble

	// Flip the seqno and ackno fields in the TCP header and increment the ackno
	htonlc(&ipAddr , pTCP->seqno);		// get sequence number to local var
	
	pTCP->seqno[0] = pTCP->ackno[0];	// copy ack number to sequence
	pTCP->seqno[1] = pTCP->ackno[1];

	// We also have to increase the sequence number we are acknowledging. If the least
	// significant byte overflowed, we need to propagate the carry to the other bytes as well
	ipAddr.S_un.S_addr++;
	htonlc(pTCP->ackno , &ipAddr);	// write new sequence number

	// Swap port numbers
	ipAddr.S_un.S_un_w.s_w1 = pTCP->src_port;
	pTCP->src_port = pTCP->dst_port;
	pTCP->dst_port = ipAddr.S_un.S_un_w.s_w1;

	// calculate ip length
	pBuf->data.frame.prot.ip.len = ipHdrLen + sizeof(tcp_hdr);

	// original : goto tcp_send_noconn
	ip_buf_send(hIp, pBuf, TX_IP_HEADER + TX_IP_REPLY + TX_IP_TCP_CHKSUM);	// send frame (build ip header and tcp checksum)

	return ret;
}


/*********************************************************************************

  Function    : sock_periodic
  Description : periodic socket handling

*********************************************************************************/

void sock_periodic(IP_STACK_H hIp)
{
	SOCK_PTR		sock;
	ip_buf_type		*pBuf;
	unsigned short	nrtx;
	char *pSockState = hIp->dbgSockState;

	IP_LOCK_LEVEL_VAR


	// loop through all connections and process if not closed
	for(sock = hIp->sock ; sock < hIp->sock + IP_TCP_SOCKETS ; sock++)
	{
		*pSockState++ = dbgSockStateInfo[sock->header.state];

		switch(sock->header.state)
		{
			// connection closed or in listen mode, next loop
			case IP_FREE:
			case IP_CLOSED:
			case IP_LISTEN:
				continue;

			case IP_TIME_WAIT:
			case IP_FIN_WAIT_2:
				sock->timer++;

				if(sock->timer < IP_TIME_WAIT_TIMEOUT) continue;

				freeWaitSocket(sock);

				continue;	// connection closed, next loop

			#if (IP_ACTIVE_OPEN > 0)

			case IP_SYN_TX:
				pBuf = ip_alloc_tx_buffer(hIp);	// allocate packet for syn
				if(pBuf==0) continue;			// no tx buffer available, try next time

				ip_tcp_send(sock, pBuf, TCP_SYN);

				sock->header.state = IP_SYN_SENT;
				continue;

			case IP_SYN_ACK_TX:
				pBuf = ip_alloc_tx_buffer(hIp);	// allocate packet for syn
				if(pBuf==0) continue;			// no tx buffer available, try next time

				ip_tcp_send(sock, pBuf, TCP_ACK);

				sock->header.state = IP_CONNECTED;
				continue;

			#endif


		}


		// all other states than CLOSED,TIME_WAIT and FIN_WAIT_2 ...

		// If the connection has outstanding data, we increase the connection's timer
		// and see if it has reached the RTO value in which case we retransmit
		if(sock->len==0) continue;

		pBuf = 0;

		if(sock->timer > 0)
		{
			sock->timer--;
			continue;	// nothing to do if timer not yet elapsed
		}
		else if(sock->header.state != IP_CONNECTED)
		{
			pBuf = ip_alloc_tx_buffer(hIp);	// allocate packet for retransmission
			if(pBuf==0) continue;			// no tx buffer available, try next time
		}

		nrtx = sock->nrtx;

		if(nrtx >= IP_MAXRTX || 
			((sock->header.state == IP_SYN_SENT || sock->header.state == IP_SYN_RCVD) && nrtx >= IP_MAXSYNRTX))
		{
			if(sock->header.state == IP_CONNECTED)	// use pending transmit buffer when connection is established
			{
				pBuf = sock->pTx;
				if(pBuf==0) continue;	// no buffer available

				sock->pTx = 0;
			}

			IP_LOCK_LEVEL_ON

			if(sock->cmdClose)
			{
				ip_socket_free(sock, IP_FREE);
				IP_LOCK_LEVEL_OFF

				// buffer which was allocated is not used in this path
				if(pBuf) pBuf->header.state = IP_BUF_STATE_IDLE;
			}
			else
			{
				ip_socket_free(sock, IP_CLOSED);
				IP_LOCK_LEVEL_OFF

				// send frame with reset (no data) to signal shutdown of connection
				ip_tcp_send(sock, pBuf, TCP_RST | TCP_ACK);
			}

			continue;
		}

		// Exponential backoff
		sock->timer = IP_RTO << (nrtx > 4 ? 4 : nrtx);

		nrtx++;	// increase retry count

		/* Ok, so we need to retransmit. We do this differently depending on which
		state we are in. In SYN_RCVD, we resend the	SYNACK that
		we sent earlier and in LAST_ACK we have to retransmit our FINACK. */
		IP_STAT(hIp->stat.tcp_retransmit++);

		switch(sock->header.state)
		{
			case IP_SYN_RCVD:
				// In the SYN_RCVD state, we should retransmit our SYNACK
				ip_tcp_send(sock, pBuf, TCP_SYN | TCP_ACK);	// send synack
				break;

			case IP_SYN_SENT:
				// In the SYN_SENT state, we retransmit out SYN
				ip_tcp_send(sock, pBuf, TCP_SYN);	// send syn
				break;

			case IP_CONNECTED:
				// retransmit last packet with data if available
				if(sock->pTx && sock->pTx->header.state == IP_BUF_STATE_TX_DONE)
				{
					sock->pTx->header.state = IP_BUF_STATE_TX;
					ip_tcp_appsend(sock);
				}

				break;

			case IP_FIN_WAIT_1:
			case IP_CLOSING:
			case IP_LAST_ACK:
				// In all these states we should retransmit a FINACK
				ip_tcp_send(sock, pBuf, TCP_FIN | TCP_ACK);
				break;;

			default:
				// free allocated buffer if not used (should not happen)

				if(pBuf) pBuf->header.state = IP_BUF_STATE_IDLE;
				break;
		}

		sock->nrtx = nrtx;	// write retry count back to socket (because ip_tcp_appsend() has reset it to 0)
	}
}

void sock_set_mtu(unsigned short mtu)
{
	// limit MTU for socket interface to max possible value
#ifdef IP_VLAN_TAG
	if(mtu > 1496) mtu = 1496;
#else
	if(mtu > 1500) mtu = 1500;
#endif

	ipSockInt.mss = mtu - sizeof(ip_hdr) - sizeof(tcp_hdr);
}

void ip_tcp_appsend(SOCK_PTR sock)
{
	ip_buf_type	*pBuf;
	tcp_hdr		*pTCP;

	unsigned char	data = 0;

	if(sock->header.state == IP_LAST_ACK) return;	// do not send anymore if in last-ack state

	if(sock->pTx && sock->pTx->header.state == IP_BUF_STATE_TX)
	{
		pBuf = sock->pTx;	// use the data buffer for sending the packet
		pBuf->header.state = IP_BUF_STATE_TX_ACK;
		data = 1;
	}
	else
	{
		pBuf = ip_alloc_tx_buffer(sock->hIp);	// allocate tx buffer
	}

	if(pBuf==0) return;		// no tx buffer available

	pTCP = (tcp_hdr*)(&pBuf->data.frame.prot.ip + 1);

	// If the application has data to be sent
	if(data)
	{
		// Remember how much data we send out now so that we know when everything has been acknowledged
		sock->len = pBuf->header.dataSize;
	}
	else
	{
		sock->nrtx = 0;
	}

	ip_tcp_send(sock, pBuf, TCP_ACK);				// Send the packet
}


void ip_tcp_send(SOCK_PTR sock, ip_buf_type *pBuf, unsigned short flags)
{
	ip_hdr			*pIP;
	tcp_hdr			*pTCP;
	unsigned char	*pByte;

	// buffer not yet allocated, try to get one
	if(pBuf==0)
	{
		pBuf = ip_alloc_tx_buffer(sock->hIp);
	}
	else if(pBuf->header.push)
	{
		flags = flags | TCP_PSH;				// send the packet with push if data is sent
	}
	
	// no buffer available, just skip the frame
	if(pBuf==0) return;

	pIP  = &pBuf->data.frame.prot.ip;
	pTCP = (tcp_hdr*)(pIP + 1);

	if(sock->cmdClose==1 && pBuf->header.push==0)	// do not close connection if there is still data to be sent
	{
		// increment send-sequence-number if there is outstanding data (should not be)
		sock->snd_nxt += sock->len;

		sock->cmdClose		= 2;
		sock->len			= 1;
		sock->header.state	= IP_FIN_WAIT_1;

		flags = TCP_ACK | TCP_FIN;
	}

	pTCP->flags = flags;

	// We're done with the input processing. We are now ready to send a reply. Our job is to fill in
	// all the fields of the TCP and IP headers before calculating the checksum and finally send the packet
	
	// ip header
	copy_ip_address(pIP->dst_ip, &sock->ripaddr);

	pIP->proto = IPPROTO_TCP;

	// tcp header
	pTCP->src_port = sock->lport;
	pTCP->dst_port = sock->rport;

	// this stack is not a high performance one and can not get more than 1 packet before ack must be sent to the client
	pTCP->wnd = HTONS(ipSockInt.mss);

	htonlc(pTCP->ackno, &sock->rcv_nxt);
	htonlc(pTCP->seqno, &sock->snd_nxt);

	pIP->len		= sizeof(ip_hdr)+sizeof(tcp_hdr);	// tcp frame without data

	pTCP->tcpoffset	= (sizeof(tcp_hdr)) * 4;			// *4 to get the number of longs in the high nibble

	if(flags & TX_TCP_SYN)
	{
  		// We send out the TCP Maximum Segment Size option with our SYNACK
		pByte = ((unsigned char*)pTCP) + sizeof(tcp_hdr);

		pByte[0] = 2;	// option MSS
		pByte[1] = 4;	// size of option

		*(unsigned short*)(pByte+2) = HTONS(ipSockInt.mss);	// put MSS value to option field

		pBuf->header.dataSize = 4;						// options

		pTCP->tcpoffset	= (sizeof(tcp_hdr)+4) * 4;		// *4 to get the number of longs in the high nibble
	}

	pIP->len = pIP->len + pBuf->header.dataSize;	// add data size to ip length


	// send ip frame (build ip header and tcp checksum
	flags = TX_IP_HEADER + TX_IP_TCP_CHKSUM;

#ifdef IP_SECONDARY_ADDRESS
	if(sock->flags & SOCK_FLAG_SECONDARY_IP) flags = flags + TX_SECONDARY_IP;
#endif
	ip_buf_send(sock->hIp, pBuf, flags);	// send frame
}


/*********************************************************************************

  Function    : ipGetHandle
  Description : get IP handle of a socket

  Parameter:
	socket	: socket created with socket(...)

  Return Value:
	Handle of IP stack
	0 = error
*********************************************************************************/
IP_STACK_H	ipGetHandle(SOCKET s)
{
	if(s==0 || s==INVALID_SOCKET) return 0;	// error, invalid socked passed

	return ((SOCK_PTR)s)->hIp;
}

/*********************************************************************************

  Function    : ipLastError
  Description : get last error code

*********************************************************************************/
int	WSAGetLastError(void)
{
	return ipSockInt.lastError;
}

#endif
