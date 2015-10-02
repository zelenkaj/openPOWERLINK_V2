/* ip.h - IP Stack for FPGA MAC Controller */
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
 File:      ip.h
 Author:    Thomas Enzinger(enzingert)
 Created:   26.03.2004
------------------------------------------------------------------------------
 History:
 26.03.2004	enzinger	created
 30.03.2004 enzinger		- ipGetAddress() changed, now also the eth-address
						  can be retrieved
						- ipGetHandle() new

 16.06.2004	enzinger	- Interface of some functions changed
							(data interface to ethernet driver now with packets
							instead of ptr and length)
						 --> The interface to the application has not changed 
 17.09.2004 enzinger	- new function : ipDisableInitArp()
 13.10.2004	enzinger	- udp info type also contains the local host
 29.10.2004	enzinger	- error in ipGetAddress() corrected (returned mac address was wrong)
 02.11.2004	enzinger	- IP_ETHERTYPE... defines moved from IP.c to IP.h
 28.12.2004	enzinger	- new function : ipUdpClose()
 16.03.2005	enzinger	- pData for recv and send to void-pointers changed
 10.10.2005	enzinger	- new function : ipPowerOn();
 12.10.2005	enzinger	- new fuunction: ipDhcpActivate(  );
 28.11.2006	enzinger	- new: htonl, ipArpQuery()
 17.01.2007	enzinger	- new: ipGetState()
						- Functions and types renamed to windows standard function names
						- Support for IP Multicast added, support for DGRAM-Sockets added
						- New function: ipDhcpSetDnsName(), ipChangeAddress()
 24.01.2007 chkr        - New function: ipDhcpGetDnsName(..)
 04.05.2007 chkr        - New function: ipGetGateway(...)
 31.05.2007	enzinger	- New function: ipArpRequest(...)
 13.09.2007	enzinger	- New function: ipSetMtu(...)
 12.02.2008	enzinger	additions for IP_LOOPBACK and IP_ACTIVE_OPEN
 06.03.2008	enzinger	new parameter TCP_ACK_REQUEST for setsockopt()
 24.03.2008	enzinger	new TCP_REJECT_SEGMENT
 16.03.2010 prenninger	move defines from ip.c and ip_internal.h to ip.h
 04.01.2011	enzinger	support of VLAN-Tags (all Tx-Frames will be tagged with 0x8100 0000)
 22.06.2011	enzinger	state-konstanten in enum umgebaut, bessere debug-anzeige
 30.09.2011	enzinger	new function : ipDhcpDeactivate() ... no DHCP-Request even if IP=0.0.0.0
                        (for Profinet BC)
 12.01.2012	enzinger	Erweiterungen für 2te IP Adresse
 07.02.2012	enzinger	Neue Funktion ipArpAnnouncement (wird von ip.c verwendet)
 30.07.2012 enzinger    new parameter TCP_PACK_SEND_DATA for setsockopt()

------------------------------------------------------------------------------
*/

/*****************************************************************************
DESCRIPTION
Defines and declarations

INCLUDE FILES:
 ip_opt.h

STATE:
 stabil

*/

#ifndef __IP_H__
#define __IP_H__

#include "ip_opt.h"		// application dependent defines for the ip-stack

#include "hton.h"

typedef struct	IP_IF	*IP_STACK_H;	// handle of IP stack
typedef unsigned long	SOCKET;			// socket handle

#define IP_ETHTYPE_ARP 0x0806
#define IP_ETHTYPE_IP  0x0800
#define IP_ETHTYPE_IP6 0x86dd 

#define IP_VERSION_V4  4

#define IP_FRAG_FLAG_MORE	0x2000	// More Fragments (MF)
#define IP_FRAG_FLAG_DF		0x4000	// Don't Fragment (DF)

//---------------------  Representation of a 32-bit IP address ---------------------
#ifndef _WINSOCK2API_
	// return value of the socket() or accept() to signal invalid sockets
	#define	INVALID_SOCKET	0		// invalid value for a socket
	#define SOCKET_ERROR	(-1)	// error return value of socket interface Functions
	
	// parameters for the addr structure of the function ip_bind()
	#define AF_INET         2		// internetwork: UDP, TCP, etc.
	#define ADDR_ANY		0		// any address

	#define MSG_PEEK		0x2		// peek at incoming message

	#define IPPROTO_ICMP	1
	#define IPPROTO_TCP		6
	#define IPPROTO_UDP		17

	#define SOCK_STREAM     1               /* stream socket */
	#define SOCK_DGRAM      2               /* datagram socket */

	struct in_addr
	{
		union
		{
			struct { unsigned char s_b1,s_b2,s_b3,s_b4; }	S_un_b;
			struct { unsigned short s_w1,s_w2; }			S_un_w;
			unsigned long									S_addr;
		}S_un;
	};

	/*
	* Structure used by kernel to store most
	* addresses.
	*/
	struct sockaddr
	{
		unsigned short	sa_family;              /* address family */
		char			sa_data[14];            /* up to 14 bytes of direct address */
	};

	// Socket address, internet style.
	struct sockaddr_in
	{
		short			sin_family;
		unsigned short	sin_port;
		struct in_addr	sin_addr;
		char			sin_zero[8];

		#define s_addr  S_un.S_addr
	};
#endif

//--------------------------------- ethernet header ---------------------------------
typedef struct
{
	unsigned short	dst_hw[3];
	unsigned short	src_hw[3];

#ifdef IP_VLAN_TAG
	unsigned short vlanTag[2];
#endif

	unsigned short	type;		// Protocol : ETH_IP, ETH_ARP, ...
}eth_hdr;

//--------------------------------- arp header ---------------------------------
typedef struct
{
	unsigned short	hwtype;
	unsigned short	protocol;
	unsigned char	hwlen;
	unsigned char	protolen;
	unsigned short	opcode;

	unsigned short	src_hw[3];
	unsigned short	src_ip[2];
	unsigned short	dst_hw[3];
	unsigned short	dst_ip[2]; 
}arp_hdr;

//--------------------------------- IP header ---------------------------------
typedef struct
{
	unsigned char	vhl;
	unsigned char	tos;       
	unsigned short	len;
	unsigned short	ipid;
	unsigned short	ipoffset;
	unsigned char	ttl;
	unsigned char	proto;     
	unsigned short	chksum;
	unsigned short	src_ip[2];
	unsigned short	dst_ip[2];
}ip_hdr;

//--------------------------------- ethernet frame ---------------------------------
typedef struct
{
	eth_hdr eth;

	union
	{
		arp_hdr	arp;
		ip_hdr	ip;
	}prot;
}eth_frame;

//--------------------------------- ICMP header ---------------------------------
typedef struct
{
	unsigned char	type;
	unsigned char	icode;
	unsigned short	chksum;
	unsigned short	id;
	unsigned short	seqno;
}icmp_hdr;

//------------------- Representation of a 48-bit Ethernet address ------------------
typedef union
{
	unsigned char	addr[6];
	unsigned short	word[3];
}eth_addr;

//------------------- Representation of a 48-bit Ethernet address ------------------
typedef struct ip_packet_typ
{
	unsigned long length;	// data length
	unsigned char data[2];	// first 2 bytes of the packet data
}ip_packet_typ;

//************************** udp hook function ******************************
typedef struct				// structure passed to udp hook function
{
	void			*pData;			// pointer to received data
	unsigned short	len;			// length of received data
	unsigned short	localPort;
	struct in_addr	localHost;		// ip address of destination host (local ip address or broadcast)
	unsigned short	remotePort;
	struct in_addr	remoteHost;		// ip address of remote host
	eth_addr		*pRemoteMac;	// ptr to remote mac address
}ip_udp_info;

typedef void	IP_HOOKFCT		// hook function
(
 void			*arg,			// function argument from ipListen() call
 ip_udp_info	*pInfo			// address to info structure
);

//******************** function to free receive buffer ************************
typedef void	IP_BUF_FREE_FCT		// function to free a buffer
(
 ip_packet_typ		*pPacket
);

//******************** ethernet driver send function ************************
typedef unsigned long IP_ETHSEND	// ethernet driver send function
(
 void				*hEth,			// handle to ethernet driver
 ip_packet_typ		*pPacket,		// ptr to ethernet packet
 IP_BUF_FREE_FCT	*pFctFree		// ptr to function to free sent buffers
);

// required information for each buffer received from the ethernet driver
typedef struct ip_rx_queue_typ
{
 ip_packet_typ			*pPacket;	// ptr to received packet
 struct ip_rx_queue_typ	*pNext;		// ptr to next rx queue element
}ip_rx_queue_typ;

//-----------------------------------  Statistics ----------------------------------
typedef struct
{
	unsigned long	ethSendOverflow;	// number of tx buffer overflows when sending
	unsigned long	txBufferFull;		// number of unsuccessful tx buffer allocations

	unsigned long	rxPackets;			// total incoming packets
	unsigned long	txPackets;			// total outgoing packets


	#if IP_STATISTICS == 1
		unsigned long	packets_used;	// packets entered into the queue
		unsigned long	prot_ip;		// ip packets processed

		unsigned long	arp_req_tx;		// sent ARP requests

		unsigned long	icmp_rx;		// icmp frames rx
		unsigned long	icmp_tx;		// icmp frames tx

		unsigned long	tcp_rx;			// tcp frames rx
		unsigned long	tcp_tx;			// tcp frames tx

		unsigned long	udp_rx;			// udp frames rx
		unsigned long	udp_tx;			// udp frames tx
		unsigned long	udp_unused;		// unused UDP frames

		unsigned long	ip_err_version;	// unkonown IP version
		unsigned long	ip_err_frag;	// fragmentation error (fragmentation disabled)
		unsigned long	ip_err_chksum;	// ip checksum error
		unsigned long	ip_err_icmp;	// icmp frames dropped
		unsigned long	ip_err_proto;	// unknown protocol
		unsigned long	err_tcp_chksum;	// tcp checksum error
		unsigned long	err_udp_chksum;	// udp checksum error
		unsigned long	ip_reass_late_rx;	// fragments arrived too late
		unsigned long	packets_queue_full;	// ethernet driver has passed too many buffers (should never happen)

		unsigned long	tcp_synrst;
		unsigned long	tcp_rst;
		unsigned long	tcp_sock_full;	// no socket available for socket() call
		unsigned long	tcp_retransmit;	// number of retransmissions

		unsigned long	port_reused;
		unsigned long	port_reused_invalid;

		#if IP_LOOPBACK==1
			unsigned long	loopBackDropped;	// should not happen ... packets dropped because loopback queue full
		#endif
	#endif
}ip_stat;

// return values of ip_periodic(...)
typedef enum
{
	IP_STATE_DHCP_INIT			= 0,
	IP_STATE_DHCP_DISCOVER		= 1,		// find dhcp server
	IP_STATE_DHCP_REQUEST		= 2,		// request for dhcp lease
	IP_STATE_DHCP_RENEWING		= 3,		// renew dhcp lease
	IP_STATE_DHCP_REBIND		= 4,		// broadcast for new dhcp lease
	IP_STATE_INIT_ARP			= 5,		// initializing (sending 2 ARPs to myself)
	IP_STATE_OK					= 6,		// ip stack is up and running 
	IP_STATE_ERROR_IP_IN_USE	= 10,		// error, ip is already used in this network, reboot required
	IP_STATE_ERROR_INVALID_IP	= 11		// error, invalid ip address
}ipState_enum;

#define copy_ip_address(dst,src)				\
	do{											\
	((short*)(dst))[0] = ((short*)(src))[0],	\
	((short*)(dst))[1] = ((short*)(src))[1];	\
	}while(0)

//************************** general stack functions ******************************

// clear all internal variables after 'power up'
void			ipPowerOn(void);

// initialize ip stack
IP_STACK_H		ipInit(eth_addr *pEthAddr, struct in_addr* pIpAddr, IP_ETHSEND *pEthSend, void *hEth);

// destroy ip stack
void ipDestroy(IP_STACK_H hIp);

// disable initial arp requests, can be called after ipInit()
int				ipDisableInitArp(IP_STACK_H hIp);

// set gateway of the network
void			ipSetGateway(IP_STACK_H hIp, struct in_addr* pGateway);

// set subnet mask of the network
void			ipSetNetmask(IP_STACK_H hIp, struct in_addr* pSubnetMask);

// get ptr to statistic of an interface
ip_stat*		ipStats(IP_STACK_H hIp);

// periodic call of stack
// return values: see IP_STATE_...
ipState_enum ipPeriodic(IP_STACK_H hIp, unsigned long timeMs);

// get IP address and MAC address (pIpAddr / pEthAddr can also be 0 if information not required)
void			ipGetAddress(IP_STACK_H hIp, void *pEthAddr, void *pIpAddr);

// get SubnetMask and Gateway
void            ipGetGateway(IP_STACK_H hIp, void *pSubnetMask, void *pGateway);

// update arp table (takes only IPs from local subnet, this function also adds IPs if not yet in table)
void			ipArpUpdate(IP_STACK_H hIp, void *pEthAddr, void *pIpAddr);

// update arp table (updates ARP table only if IP is already in the list)
void*			ipArpAnnouncement(IP_STACK_H hIp, void *pEthAddr, void *pIpAddr);

// get MAC address address of specified IP address
// return 0 : OK
// return SOCKET_ERROR
int				ipArpQuery(unsigned long, void *pMac);

// get MAC address address of specified IP address
// (and trigger ARP request if IP not yet known)
// return 0 : OK
// return SOCKET_ERROR
int				ipArpRequest(unsigned long ip, void *pMac);

// set DNS name for DHCP (max 64 characters)
void			ipDhcpSetDnsName(char *pHostName);

// get DNS name (max 64 characters)
// return 0 : OK
// return SOCKET_ERROR
int             ipDhcpGetDnsName(char *pName, int nameLen);

// activate dhcp at runtime
void			ipDhcpActivate(IP_STACK_H hIp);

// deactivate dhcp at runtime
void			ipDhcpDeactivate(IP_STACK_H hIp);

// get ip stack state
unsigned short	ipGetState(IP_STACK_H hIp);

// change IP address
void			ipChangeAddress(IP_STACK_H hIp, struct in_addr* pIpAddr);

// restart ARP-Probe-Process (or DHCP)
void			ipRestartArpProbe(IP_STACK_H hIp);

#ifdef IP_SECONDARY_ADDRESS
	void		ipSetSecondaryIp(IP_STACK_H hIp, struct in_addr* pIpAddr, struct in_addr* pNetMask);
#endif

// set new MTU size (MTU is initialized with the value IP_MTU given in ip_opt.h, this function
//                   allows to adjust the MTU after initialization)
void			ipSetMtu(unsigned short mtu);

// calculate IP or TCP checksum
unsigned short	ip_chksum(ip_hdr *pIP, unsigned long prot);		// calculate IP/UDP or TCP checksum


//************************** interface to packet driver ******************************

// pass receive buffer to ip stack (must be linked to receive-callback from ethernet driver
int	ipPacketReceive(IP_STACK_H hIp, ip_packet_typ *pPacket, IP_BUF_FREE_FCT *pFct);



//################################################################################
//#########################  callback interface for UDP  #########################
//################################################################################

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
int	ipUdpListen(IP_STACK_H hIp, unsigned long lport, IP_HOOKFCT *pFct, void *arg);

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
int ipUdpSend(IP_STACK_H hIp, ip_udp_info *pInfo);

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
int ipUdpClose(IP_STACK_H hIp, unsigned long lport);


//################################################################################
//#########################  socket interface (only tcp)  ########################
//################################################################################


/*********************************************************************************

Function    : ipSocketExt
Description : get free TCP or UDP socket

Parameter: -

Return Value:
new socket or INVALID_SOCKET
*********************************************************************************/
SOCKET		socket(int af, int type, int protocol);

/*********************************************************************************

  Function    : bind
  Description : bind socket to a address (preparation for ip_listen()) function

  Parameter:
	socket	: socket will be switched to listen mode (only if not already connected)
	addr	: ptr to socket address structure

  Return Value:
	0  ... no error
	SOCKET_ERROR ... error (parameters wrong)
*********************************************************************************/
int			bind(SOCKET socket, struct sockaddr *addr);

/*********************************************************************************

  Function    : listen
  Description : start listening on socket

  Parameter:
	socket	: socket will be switched to listen mode (only if not already connected)

  Return Value:
	0  ... no error
	SOCKET_ERROR ... error
*********************************************************************************/
int			listen(SOCKET socket);

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
SOCKET	accept(SOCKET socket, struct sockaddr *addr, int *addrlen);

/*********************************************************************************

Function    : getsockname
Description : Get socket information (local port and IP)

Parameter:

Return Value:
0
SOCKET_ERROR ... error
*********************************************************************************/
int getsockname(SOCKET socket, struct sockaddr *addr, int *addrlen);

/*********************************************************************************

Function    : getpeername
Description : Get socket information (remote port and IP)

Parameter:

Return Value:
0
SOCKET_ERROR ... error
*********************************************************************************/
int getpeername(SOCKET socket, struct sockaddr *addr, int *addrlen);

/*********************************************************************************

  Function    : send
  Description : send data to a tcp/ip connection

  Parameter:
	socket	: socket created with socket(...)
	buf		: ptr to data to be sent
	len		: total number of bytes which should be sent

  Return Value:
	>= 0 ... number of bytes which were sent
	SOCKET_ERROR ... error
*********************************************************************************/
int		send(SOCKET s, const char *buf, long len);

/*********************************************************************************

  Function    : sendto
  Description : send data to a UPD or IP connection

  Parameter:
    socket	: socket created with socket(...)
    pData	: ptr to data to be sent
    size	: total number of bytes which should be sent

  Return Value:
    >= 0 ... number of bytes which were sent
    SOCKET_ERROR ... error
*********************************************************************************/
int		sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr* to, int tolen);

/*********************************************************************************

  Function    : recv / recvfrom
  Description : receive data from tcp-ip connection or UDP socket

  Parameter:
	socket	: socket created with socket(...)
	pData	: ptr to data buffer
	size	: maximum number of bytes in the data buffer
	from	: optional : structure 
	fromlen	: optional : ptr to length or 'from'

  Return Value:
	>= 0 ... number of received bytes
	SOCKET_ERROR  ... error
*********************************************************************************/
int		recv(SOCKET s, void *buf, int len, int flags);
int		recvfrom(SOCKET s, void *buf, int len, int flags, struct sockaddr *from, int *fromlen);

/*********************************************************************************

  Function    : closesocket
  Description : close a socket

  Parameter:
	socket	: socket created with socket(...)

  Return Value:
	0  ... no error
	SOCKET_ERROR ... error

*********************************************************************************/
int		closesocket(SOCKET socket);

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
int		ioctlsocket(SOCKET s, long cmd, unsigned long *argp);

// set socket options
int		setsockopt(SOCKET s, int level, int optname, const char  * optval, int optlen);

/*********************************************************************************

  Function    : ipGetHandle
  Description : get IP handle of a socket

  Parameter:
	socket	: socket created with socket(...)

  Return Value:
	Handle of IP stack
	0 = error
*********************************************************************************/
IP_STACK_H	ipGetHandle(SOCKET socket);

/*********************************************************************************

   Function    : ipLastError
   Description : get last error code

*********************************************************************************/
int	WSAGetLastError(void);

#if IP_ACTIVE_OPEN > 0

	/*
	* Select uses arrays of SOCKETs.  These macros manipulate such
	* arrays.  FD_SETSIZE may be defined by the user before including
	* this file, but the default here should be >= 64.
	*
	* CAVEAT IMPLEMENTOR and USER: THESE MACROS AND TYPES MUST BE
	* INCLUDED  WINSOCK.H EXACTLY AS SHOWN HERE.
	*/
	#ifndef FD_SETSIZE
	#define FD_SETSIZE      64
	#endif /* FD_SETSIZE */

	typedef struct fd_set {
		unsigned long   fd_count;               /* how many are SET? */
		SOCKET			fd_array[FD_SETSIZE];   /* an array of SOCKETs */
	} fd_set;

	/*
	* Structure used in select() call, taken from the BSD file sys/time.h.
	*/
	struct timeval {
		signed long    tv_sec;         /* seconds */
		signed long    tv_usec;        /* and microseconds */
	};

	#define FD_CLR(fd, set) do { \
		UINT32 __i; \
		for (__i = 0; __i < ((fd_set  *)(set))->fd_count ; __i++) { \
		if (((fd_set  *)(set))->fd_array[__i] == fd) { \
		while (__i < ((fd_set  *)(set))->fd_count-1) { \
		((fd_set  *)(set))->fd_array[__i] = \
		((fd_set  *)(set))->fd_array[__i+1]; \
		__i++; \
		} \
		((fd_set  *)(set))->fd_count--; \
		break; \
		} \
		} \
		} while(0)

	#define FD_SET(fd, set) do { \
		if (((fd_set  *)(set))->fd_count < FD_SETSIZE) \
		((fd_set  *)(set))->fd_array[((fd_set  *)(set))->fd_count++]=(fd);\
		} while(0)

	#define FD_ZERO(set) (((fd_set  *)(set))->fd_count=0)

	int	__WSAFDIsSet(SOCKET s, fd_set* set);

	#define FD_ISSET(fd, set) __WSAFDIsSet((SOCKET)(fd), (fd_set  *)(set))

	/*********************************************************************************

	Function    : connect
	Description : connect to another host

	Parameter:
	socket	: socket created with socket(...)

	Return Value:
	0  ... no error

	*********************************************************************************/
	int connect(SOCKET s, const struct sockaddr* name, int namelen);



	/*********************************************************************************

	Function    : select
	Description : get information about a list of sockets

	Parameter:
	see microsoft help

	Return Value:

	*********************************************************************************/
	int select(int nfds, fd_set* readfds, fd_set* writefds, fd_set* exceptfds, const struct timeval* timeout);

#endif


/*
* Commands for ioctlsocket(),  taken from the BSD file fcntl.h.
*
*
* Ioctl's have the command encoded in the lower word,
* and the size of any in or out parameters in the upper
* word.  The high 2 bits of the upper word are used
* to encode the in/out status of the parameter; for now
* we restrict parameters to at most 128 bytes.
*/
#define IOCPARM_MASK    0x7f            /* parameters must be < 128 bytes */
#define IOC_VOID        0x20000000      /* no parameters */
#define IOC_OUT         0x40000000      /* copy out parameters */
#define IOC_IN          0x80000000      /* copy in parameters */
#define IOC_INOUT       (IOC_IN|IOC_OUT)
/* 0x20000000 distinguishes new &
old ioctl's */
#define _IO(x,y)        ((long)(IOC_VOID|((x)<<8)|(y)))

#define _IOR(x,y,t)     ((long)(IOC_OUT|(((INT32)sizeof(t)&IOCPARM_MASK)<<16)|((x)<<8)|(y)))

#define _IOW(x,y,t)     ((long)(IOC_IN|(((INT32)sizeof(t)&IOCPARM_MASK)<<16)|((x)<<8)|(y)))

#define FIONREAD    _IOR('f', 127, UINT32) /* get # bytes to read */
#define FIONBIO     _IOW('f', 126, UINT32) /* set/clear non-blocking i/o */
#define FIOASYNC    _IOW('f', 125, UINT32) /* set/clear async i/o */

/* Socket I/O Controls */
#define SIOCSHIWAT  _IOW('s',  0, UINT32)  /* set high watermark */
#define SIOCGHIWAT  _IOR('s',  1, UINT32)  /* get high watermark */
#define SIOCSLOWAT  _IOW('s',  2, UINT32)  /* set low watermark */
#define SIOCGLOWAT  _IOR('s',  3, UINT32)  /* get low watermark */
#define SIOCATMARK  _IOR('s',  7, UINT32)  /* at oob mark? */

/*
* Level number for (get/set)sockopt() to apply to socket itself.
*/
#define SOL_SOCKET      -1              /* options for socket level */

/*
* Option flags per-socket.
*/
#define SO_DEBUG        0x0001          /* turn on debugging info recording */
#define SO_ACCEPTCONN   0x0002          /* socket has had listen() */
#define SO_REUSEADDR    0x0004          /* allow local address reuse */
#define SO_KEEPALIVE    0x0008          /* keep connections alive */
#define SO_DONTROUTE    0x0010          /* just use interface addresses */
#define SO_BROADCAST    0x0020          /* permit sending of broadcast msgs */
#define SO_USELOOPBACK  0x0040          /* bypass hardware when possible */
#define SO_LINGER       0x0080          /* linger on close if data present */
#define SO_OOBINLINE    0x0100          /* leave received OOB data in line */

// #define SO_DONTLINGER   (UINT32)(~SO_LINGER)
// chkr 05.03.2007: in setsockopt(..) nicht unterstützt
// und in der EIP Stack SW abgeragt: #ifdef SO_DONTLINGER .... usw. File eipsckt.c



/*
* Additional options.
*/
#define SO_SNDBUF       0x1001          /* send buffer size */
#define SO_RCVBUF       0x1002          /* receive buffer size */
#define SO_SNDLOWAT     0x1003          /* send low-water mark */
#define SO_RCVLOWAT     0x1004          /* receive low-water mark */
#define SO_SNDTIMEO     0x1005          /* send timeout */
#define SO_RCVTIMEO     0x1006          /* receive timeout */
#define SO_ERROR        0x1007          /* get error status and clear */
#define SO_TYPE         0x1008          /* get socket type */

/*
* Options for connect and disconnect data and options.  Used only by
* non-TCP/IP transports such as DECNet, OSI TP4, etc.
*/
#define SO_CONNDATA     0x7000
#define SO_CONNOPT      0x7001
#define SO_DISCDATA     0x7002
#define SO_DISCOPT      0x7003
#define SO_CONNDATALEN  0x7004
#define SO_CONNOPTLEN   0x7005
#define SO_DISCDATALEN  0x7006
#define SO_DISCOPTLEN   0x7007

/*
* Constants and structures defined by the internet system,
* Per RFC 790, September 1981, taken from the BSD file netinet/in.h.
*/

/*
* Protocols
*/
#define IPPROTO_IP              0               /* dummy for IP */
#define IPPROTO_ICMP            1               /* control message protocol */
#define IPPROTO_IGMP            2               /* group management protocol */
#define IPPROTO_GGP             3               /* gateway^2 (deprecated) */
#define IPPROTO_TCP             6               /* tcp */
#define IPPROTO_PUP             12              /* pup */
#define IPPROTO_UDP             17              /* user datagram protocol */
#define IPPROTO_IDP             22              /* xns idp */
#define IPPROTO_ND              77              /* UNOFFICIAL net disk proto */

#define IPPROTO_RAW             255             /* raw IP packet */
#define IPPROTO_MAX             256

/*
* Options for use with [gs]etsockopt at the IP level.
*/
#define IP_OPTIONS          1           /* set/get IP per-packet options    */
#define IP_MULTICAST_IF     2           /* set/get IP multicast interface   */
#define IP_MULTICAST_TTL    3           /* set/get IP multicast timetolive  */
#define IP_MULTICAST_LOOP   4           /* set/get IP multicast loopback    */
#define IP_ADD_MEMBERSHIP   5           /* add  an IP group membership      */
#define IP_DROP_MEMBERSHIP  6           /* drop an IP group membership      */
#define IP_TTL              7           /* set/get IP Time To Live          */
#define IP_TOS              8           /* set/get IP Type Of Service       */
#define IP_DONTFRAGMENT     9           /* set/get IP Don't Fragment flag   */

/*
* Options for use with [gs]etsockopt at the TCP level.
*/
#define TCP_ACK_REQUEST		1			/* always send ack-request after sending data */
#define TCP_REJECT_SEGMENT	2			/* reject segment if socket is occupied */
#define TCP_PACK_SEND_DATA  3           /* pack send data into existing frame, if not yet in transmit queue */

#define IP_DEFAULT_MULTICAST_TTL   1    /* normally limit m'casts to 1 hop  */
#define IP_DEFAULT_MULTICAST_LOOP  1    /* normally hear sends if a member  */
#define IP_MAX_MEMBERSHIPS         20   /* per socket; must fit in one mbuf */

/*
* WinSock error codes are also defined in winerror.h
* Hence the IFDEF.
*/
#ifndef WSABASEERR

	/*
	* All Windows Sockets error constants are biased by WSABASEERR from
	* the "normal"
	*/
	#define WSABASEERR              10000
	/*
	* Windows Sockets definitions of regular Microsoft C error constants
	*/
	#define WSAEINTR                (WSABASEERR+4)
	#define WSAEBADF                (WSABASEERR+9)
	#define WSAEACCES               (WSABASEERR+13)
	#define WSAEFAULT               (WSABASEERR+14)
	#define WSAEINVAL               (WSABASEERR+22)
	#define WSAEMFILE               (WSABASEERR+24)

	/*
	* Windows Sockets definitions of regular Berkeley error constants
	*/
	#define WSAEWOULDBLOCK          (WSABASEERR+35)
	#define WSAEINPROGRESS          (WSABASEERR+36)
	#define WSAEALREADY             (WSABASEERR+37)
	#define WSAENOTSOCK             (WSABASEERR+38)
	#define WSAEDESTADDRREQ         (WSABASEERR+39)
	#define WSAEMSGSIZE             (WSABASEERR+40)
	#define WSAEPROTOTYPE           (WSABASEERR+41)
	#define WSAENOPROTOOPT          (WSABASEERR+42)
	#define WSAEPROTONOSUPPORT      (WSABASEERR+43)
	#define WSAESOCKTNOSUPPORT      (WSABASEERR+44)
	#define WSAEOPNOTSUPP           (WSABASEERR+45)
	#define WSAEPFNOSUPPORT         (WSABASEERR+46)
	#define WSAEAFNOSUPPORT         (WSABASEERR+47)
	#define WSAEADDRINUSE           (WSABASEERR+48)
	#define WSAEADDRNOTAVAIL        (WSABASEERR+49)
	#define WSAENETDOWN             (WSABASEERR+50)
	#define WSAENETUNREACH          (WSABASEERR+51)
	#define WSAENETRESET            (WSABASEERR+52)
	#define WSAECONNABORTED         (WSABASEERR+53)
	#define WSAECONNRESET           (WSABASEERR+54)
	#define WSAENOBUFS              (WSABASEERR+55)
	#define WSAEISCONN              (WSABASEERR+56)
	#define WSAENOTCONN             (WSABASEERR+57)
	#define WSAESHUTDOWN            (WSABASEERR+58)
	#define WSAETOOMANYREFS         (WSABASEERR+59)
	#define WSAETIMEDOUT            (WSABASEERR+60)
	#define WSAECONNREFUSED         (WSABASEERR+61)
	#define WSAELOOP                (WSABASEERR+62)
	#define WSAENAMETOOLONG         (WSABASEERR+63)
	#define WSAEHOSTDOWN            (WSABASEERR+64)
	#define WSAEHOSTUNREACH         (WSABASEERR+65)
	#define WSAENOTEMPTY            (WSABASEERR+66)
	#define WSAEPROCLIM             (WSABASEERR+67)
	#define WSAEUSERS               (WSABASEERR+68)
	#define WSAEDQUOT               (WSABASEERR+69)
	#define WSAESTALE               (WSABASEERR+70)
	#define WSAEREMOTE              (WSABASEERR+71)

	#define WSAEDISCON              (WSABASEERR+101)

	/*
	* Extended Windows Sockets error constant definitions
	*/
	#define WSASYSNOTREADY          (WSABASEERR+91)
	#define WSAVERNOTSUPPORTED      (WSABASEERR+92)
	#define WSANOTINITIALISED       (WSABASEERR+93)

	/*
	* Error return codes from gethostbyname() and gethostbyaddr()
	* (when using the resolver). Note that these errors are
	* retrieved via WSAGetLastError() and must therefore follow
	* the rules for avoiding clashes with error numbers from
	* specific implementations or language run-time systems.
	* For this reason the codes are based at WSABASEERR+1001.
	* Note also that [WSA]NO_ADDRESS is defined only for
	* compatibility purposes.
	*/


	/* Authoritative Answer: Host not found */
	#define WSAHOST_NOT_FOUND       (WSABASEERR+1001)

	/* Non-Authoritative: Host not found, or SERVERFAIL */
	#define WSATRY_AGAIN            (WSABASEERR+1002)

	/* Non recoverable errors, FORMERR, REFUSED, NOTIMP */
	#define WSANO_RECOVERY          (WSABASEERR+1003)

	/* Valid name, no data record of requested type */
	#define WSANO_DATA              (WSABASEERR+1004)

	/*
	* WinSock error codes are also defined in winerror.h
	* Hence the IFDEF.
	*/
#endif /* ifdef WSABASEERR */

#endif

