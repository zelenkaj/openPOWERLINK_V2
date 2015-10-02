/**
********************************************************************************
\file   edrv2veth.c

\brief  Wrapper module to connect uIP stack to Virtual Ethernet driver

This module converts the driver interface of the IP stack to the interface
provided from the Virtual Ethernet driver NoOs.

\ingroup module_ip
*******************************************************************************/

/*------------------------------------------------------------------------------
* License Agreement
*
* Copyright 2013 BERNECKER + RAINER, AUSTRIA, 5142 EGGELSBERG, B&R STRASSE 1
* All rights reserved.
*
* Redistribution and use in source and binary forms,
* with or without modification,
* are permitted provided that the following conditions are met:
*
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer
*     in the documentation and/or other materials provided with the
*     distribution.
*   * Neither the name of the B&R nor the names of its contributors
*     may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <user/vethapi-noos.h>

#include "edrv2veth.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define GET_TYPE_BASE(typ, element, ptr)    \
    ((typ*)( ((size_t)ptr) - (size_t)&((typ*)0)->element ))

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
 * \brief Instance structure of the wrapper module
 */
typedef struct
{
    IP_STACK_H              pIpSocket;
} tEdrv2VethInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tEdrv2VethInstance  edrv2vethInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static void edrv2veth_changeAddress( UINT32 ipAddr_p, UINT32 subNetMask_p, UINT16 mtu_p );
static void edrv2veth_changeGateway( UINT32 defGateway_p );
static tOplkError edrv2veth_receiveHandler(UINT8* pFrame_p, UINT32 frameSize_p);
static void edrv2veth_freePacket(ip_packet_typ *pPacket);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief Initialize wrapper module

\param  pEthMac[out]   Pointer to the address where the MAC should be stored

\return tOplkError
\retval kErrorOk      On success

\ingroup module_ip
*/
//------------------------------------------------------------------------------
tOplkError edrv2veth_init (eth_addr* pEthMac)
{
    tOplkError ret = kErrorOk;

    OPLK_MEMSET(&edrv2vethInstance_l, 0 , sizeof(tEdrv2VethInstance));

    //copy default MAC address
    OPLK_MEMCPY(pEthMac, veth_apiGetEthMac(), sizeof(eth_addr));

    // register virtual Ethernet driver address changed callbacks
    veth_apiRegDefaultGatewayCb(edrv2veth_changeGateway);
    veth_apiRegNetAddressCb(edrv2veth_changeAddress);

    // register frame receive handler
    veth_apiRegReceiveHandlerCb(edrv2veth_receiveHandler);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Exit wrapper module

\ingroup module_ip
*/
//------------------------------------------------------------------------------
void edrv2veth_exit (void)
{
    // Cleanup module
}

//------------------------------------------------------------------------------
/**
\brief Set the IP stack handler in the wrapper module

\param  pIpHandler_p    Pointer to the instance of the IP stack

\return tOplkError
\retval kErrorOk          On success
\retval kEplApiInvalidParam     Invalid IP stack instance

\ingroup module_ip
*/
//------------------------------------------------------------------------------
tOplkError edrv2veth_setIpHandler(IP_STACK_H pIpHandler_p)
{
    tOplkError ret = kErrorOk;

    if(pIpHandler_p != NULL)
    {
        edrv2vethInstance_l.pIpSocket = pIpHandler_p;
    } else
    {
        ret = kErrorApiInvalidParam;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Forward a frame to the Virtual Ethernet driver

\param  hEth       Handler of the IP stack
\param  pPacket    Pointer to the packet to transmit
\param  pFctFree   Callback to free the packet

\return unsigned long
\retval 0                   Transmit successful
\retval Other               Error while transmitting

\ingroup module_ip
*/
//------------------------------------------------------------------------------
ULONG edrv2veth_transmit(void *hEth, ip_packet_typ *pPacket, IP_BUF_FREE_FCT *pFctFree)
{
    tOplkError ret = kErrorOk;
    unsigned long plkLen;

    UNUSED_PARAMETER(hEth);

    // forward packet to virtual Ethernet driver
    ret = veth_apiTransmit(pPacket->data, pPacket->length);
    if(ret != kErrorOk)
    {
        // return error!
        plkLen = 0;
    }
    else
    {
        // return number of bytes sent!
        plkLen = pPacket->length;

        // call free function
        pFctFree(pPacket);
    }

    return plkLen;
}


//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Change IP address, Subnet mask and MTU of the IP stack

\param  ipAddr_p       The IP address of the node to set
\param  subNetMask_p   The Subnet mask of the node to set
\param  mtu_p          The MTU of t he node to set

\ingroup module_ip
*/
//------------------------------------------------------------------------------
static void edrv2veth_changeAddress(UINT32 ipAddr_p, UINT32 subNetMask_p, UINT16 mtu_p)
{
    struct in_addr IpAddr;

    UNUSED_PARAMETER(mtu_p);

    IpAddr.S_un.S_addr = htonl(ipAddr_p);

    ipChangeAddress(edrv2vethInstance_l.pIpSocket, &IpAddr);

    IpAddr.S_un.S_addr = htonl(subNetMask_p);

    ipSetNetmask(edrv2vethInstance_l.pIpSocket, &IpAddr);
}

//------------------------------------------------------------------------------
/**
\brief Change the default gateway of the IP stack

\param  defGateway_p       The default gateway of the node to set

\ingroup module_ip
*/
//------------------------------------------------------------------------------
static void edrv2veth_changeGateway( UINT32 defGateway_p )
{
    struct in_addr IpGateway;

    IpGateway.S_un.S_addr = htonl(defGateway_p);

    ipSetGateway(edrv2vethInstance_l.pIpSocket, &IpGateway);
}

//------------------------------------------------------------------------------
/**
\brief Receive handler of the wrapper module

Handles an incoming frame from the Virtual Eternet driver and forwards it
to the IP stack.

\param  pFrame_p       Pointer to the incoming payload
\param  frameSize_p    Size of the incoming payload

\return tOplkError
\retval kErrorOk          On success

\ingroup module_ip
*/
//------------------------------------------------------------------------------
static tOplkError edrv2veth_receiveHandler(UINT8* pFrame_p, UINT32 frameSize_p)
{
    tOplkError     ret = kErrorOk;
    INT            rcvStatus;
    ip_packet_typ* pPacket;

    UNUSED_PARAMETER(frameSize_p);

    // TODO: Address manipulation only works when used with openMAC!
    //       (The length field needs to be before the packet!)
    pPacket = GET_TYPE_BASE(ip_packet_typ, data, pFrame_p);

    // Forward incoming data to IP stack
    rcvStatus = ipPacketReceive(edrv2vethInstance_l.pIpSocket,
                                pPacket, edrv2veth_freePacket);
    if(rcvStatus != 0)
    {
        // Call free function now
        edrv2veth_freePacket(pPacket);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief Free the packet buffer after transmission

\param  pPacket       Pointer to the packet buffer for freeing

\ingroup module_ip
*/
//------------------------------------------------------------------------------
static void edrv2veth_freePacket(ip_packet_typ* pPacket)
{
    tOplkError ret = kErrorOk;

    ret = veth_apiReleaseRxFrame(pPacket->data, pPacket->length);
    if(ret != kErrorOk)
    {
        PRINTF("%s(Err/Warn): Error while freeing the Veth receive buffer\n",
                __func__);
    }
}

/// \}
