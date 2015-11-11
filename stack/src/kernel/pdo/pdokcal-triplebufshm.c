/**
********************************************************************************
\file   pdokcal-triplebufshm.c

\brief  Shared memory triple buffer implementation for kernel PDO CAL module

This file contains an implementation for the kernel PDO CAL module which uses
a shared memory region between user and kernel layer. PDOs are transfered
through triple buffering between the layers. Therefore, reads and writes to
the PDOs can occur completely asynchronously.

This file contains no specific shared memory implementation. This is encapsulated
in the pdokcalmem-XX.c modules.

A critical part is when switching the buffers. To implement safe operation
without locking, the buffer switching has to be performed in an atomic operation.

\ingroup module_pdokcal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <common/oplkinc.h>
#include <common/target.h>
#include <kernel/pdokcal.h>

#include <oplk/benchmark.h>

#include <system.h>
#include <altera_avalon_dma_regs.h>
#include <sys/alt_irq.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PDO_DMA_TRANSFER_TX
#define PDO_DMA_TRANSFER_RX

#if defined(PDO_DMA_TRANSFER_TX) || defined(PDO_DMA_TRANSFER_RX)
#define PDO_DMA_TRANSFER
#endif

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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct
{
    UINT8*  pRead;
    UINT8*  pWrite;
    size_t  length;
    BOOL    fRx;
    UINT    channelId;
} tPdoDmaDesc;

typedef struct
{
    UINT            writeIndex;
    UINT            readIndex;
    tPdoDmaDesc     aBuffer[512];
} tPdoDmaDescFifo;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tPdoMemRegion*       pPdoMem_l;
static size_t               pdoMemRegionSize_l;
static BYTE*                pTripleBuf_l[3];

static tPdoDmaDescFifo      pdoDmaFifo_l;
static tPdoDmaDesc          pdoDmaCurrent_l;
static BOOL                 fPdoDmaActive_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void setupPdoMemInfo(tPdoChannelSetup* pPdoChannels_p, tPdoMemRegion* pPdoMemRegion_p);
static tOplkError initDma(void);
static void pdoCopy(tPdoDmaDesc* pPdoDmaDesc_p);
static void startDmaCopy(tPdoDmaDesc* pPdoDmaDesc_p);
static void dmaIsr(void* pArg_p);
static BOOL getNextDmaCopy(tPdoDmaDesc* pPdoDmaDesc_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get address of PDO memory reagion

The function returns the address of the PDO memory region.

\param ppPdoMemBase     Double pointer to the PDO memory.
\param pPdoMemSize_p    Pointer to the size of PDO memory.

\note pPdoMemSize_p is optional, caller can specify NULL if the size is not required.

\return Returns the address of the PDO memory region.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_getPdoMemRegion(UINT8** ppPdoMemBase, size_t* pPdoMemSize_p)
{
    if (ppPdoMemBase == NULL)
        return kErrorInvalidOperation;

    if (pPdoMemSize_p != NULL)
        *pPdoMemSize_p = pdoMemRegionSize_l;

    *ppPdoMemBase = (UINT8*)pPdoMem_l;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize PDO memory

The function initializes the memory needed to transfer PDOs.

\param  pPdoChannels        Pointer to PDO channel configuration.
\param  rxPdoMemSize_p      Size of RX PDO buffers.
\param  txPdoMemSize_p      Size of TX PDO buffers.

\return The function returns a tOplkError error code.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_initPdoMem(tPdoChannelSetup* pPdoChannels, size_t rxPdoMemSize_p,
                              size_t txPdoMemSize_p)
{
    BYTE*   pMem;
    size_t  pdoMemSize;

#if defined(PDO_DMA_TRANSFER)
    if (initDma() != kErrorOk)
        return kErrorNoResource;
#endif

    pdoMemSize = txPdoMemSize_p + rxPdoMemSize_p;

    if (pPdoMem_l != NULL)
        pdokcal_freeMem((BYTE*)pPdoMem_l, pdoMemRegionSize_l);

    pdoMemRegionSize_l = (pdoMemSize * 3) + sizeof(tPdoMemRegion);
    if (pdokcal_allocateMem(pdoMemRegionSize_l, &pMem) != kErrorOk)
    {
        return kErrorNoResource;
    }

    pPdoMem_l = (tPdoMemRegion*)pMem;

    pTripleBuf_l[0] = (BYTE*)pPdoMem_l + sizeof(tPdoMemRegion);
    pTripleBuf_l[1] = pTripleBuf_l[0] + pdoMemSize;
    pTripleBuf_l[2] = pTripleBuf_l[1] + pdoMemSize;

    DEBUG_LVL_PDO_TRACE("%s() PdoMem:%p size:%d Triple buffers at: %p/%p/%p\n",
                        __func__, pPdoMem_l, pdoMemRegionSize_l,
                        pTripleBuf_l[0], pTripleBuf_l[1], pTripleBuf_l[2]);

    OPLK_MEMSET(pPdoMem_l, 0, pdoMemRegionSize_l);
    setupPdoMemInfo(pPdoChannels, pPdoMem_l);

    OPLK_ATOMIC_INIT(pPdoMem_l);

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up PDO memory

The function cleans the memory allocated for PDO buffers.

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
void pdokcal_cleanupPdoMem(void)
{
    DEBUG_LVL_PDO_TRACE("%s()\n", __func__);

    if (pPdoMem_l != NULL)
        pdokcal_freeMem((BYTE*)pPdoMem_l, pdoMemRegionSize_l);

    pPdoMem_l = NULL;
    pdoMemRegionSize_l = 0;
    pTripleBuf_l[0] = NULL;
    pTripleBuf_l[1] = NULL;
    pTripleBuf_l[2] = NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Write RXPDO to PDO memory

The function writes a received RXPDO into the PDO memory range.

\param  channelId_p             Channel ID of PDO to write.
\param  pPayload_p              Pointer to received PDO payload.
\param  pdoSize_p               Size of received PDO.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_writeRxPdo(UINT channelId_p, BYTE* pPayload_p, UINT16 pdoSize_p)
{
    BYTE*           pPdo;
    OPLK_ATOMIC_T   temp;
    tPdoDmaDesc     desc;

    // Invalidate data cache for addressed rxChannelInfo
    OPLK_DCACHE_INVALIDATE(&(pPdoMem_l->rxChannelInfo[channelId_p]), sizeof(tPdoBufferInfo));

    pPdo = pTripleBuf_l[pPdoMem_l->rxChannelInfo[channelId_p].writeBuf] +
           pPdoMem_l->rxChannelInfo[channelId_p].channelOffset;
    //TRACE("%s() chan:%d wi:%d\n", __func__, channelId_p, pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);

#if !defined(PDO_DMA_TRANSFER_RX)
    OPLK_MEMCPY(pPdo, pPayload_p, pdoSize_p);

    OPLK_DCACHE_FLUSH(pPdo, pdoSize_p);

    temp = pPdoMem_l->rxChannelInfo[channelId_p].writeBuf;
    OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->rxChannelInfo[channelId_p].cleanBuf,
                         temp,
                         pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);

    pPdoMem_l->rxChannelInfo[channelId_p].newData = 1;

    // Flush data cache for variables changed in this function
    OPLK_DCACHE_FLUSH(&(pPdoMem_l->rxChannelInfo[channelId_p].writeBuf), sizeof(OPLK_ATOMIC_T));
    OPLK_DCACHE_FLUSH(&(pPdoMem_l->rxChannelInfo[channelId_p].newData), sizeof(UINT8));
#else
    UNUSED_PARAMETER(temp);

    desc.pWrite = (UINT8*)pPdo;
    desc.pRead = (UINT8*)pPayload_p;
    desc.length = (size_t)pdoSize_p;
    desc.fRx = TRUE;
    desc.channelId = channelId_p;
    pdoCopy(&desc);
#endif

    //TRACE("%s() chan:%d new wi:%d\n", __func__, channelId_p, pPdoMem_l->rxChannelInfo[channelId_p].writeBuf);
    //TRACE("%s() *pPayload_p:%02x\n", __func__, *pPayload_p);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read TXPDO from PDO memory

The function reads a TXPDO to be sent from the PDO memory range.

\param  channelId_p             Channel ID of PDO to read.
\param  pPayload_p              Pointer to PDO payload which will be transmitted.
\param  pdoSize_p               Size of PDO to be transmitted.

\return Returns an error code

\ingroup module_pdokcal
*/
//------------------------------------------------------------------------------
tOplkError pdokcal_readTxPdo(UINT channelId_p, BYTE* pPayload_p, UINT16 pdoSize_p)
{
    BYTE*           pPdo;
    OPLK_ATOMIC_T   readBuf;
    tPdoDmaDesc     desc;

    // Invalidate data cache for addressed txChannelInfo
    OPLK_DCACHE_INVALIDATE(&(pPdoMem_l->txChannelInfo[channelId_p]), sizeof(tPdoBufferInfo));

    if (pPdoMem_l->txChannelInfo[channelId_p].newData)
    {
        readBuf = pPdoMem_l->txChannelInfo[channelId_p].readBuf;
        OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->txChannelInfo[channelId_p].cleanBuf,
                             readBuf,
                             pPdoMem_l->txChannelInfo[channelId_p].readBuf);
        pPdoMem_l->txChannelInfo[channelId_p].newData = 0;

        // Flush data cache for variables changed in this function
        OPLK_DCACHE_FLUSH(&(pPdoMem_l->txChannelInfo[channelId_p].readBuf), sizeof(OPLK_ATOMIC_T));
        OPLK_DCACHE_FLUSH(&(pPdoMem_l->txChannelInfo[channelId_p].newData), sizeof(UINT8));
    }

    //TRACE("%s() pPdo_p:%p pPayload:%p size:%d value:%d\n", __func__,
    //        pPdo_p, pPayload_p, pdoSize_p, *pPdo_p);
    //TRACE("%s() chan:%d ri:%d\n", __func__, channelId_p, pPdoMem_l->txChannelInfo[channelId_p].readBuf);
    pPdo = pTripleBuf_l[pPdoMem_l->txChannelInfo[channelId_p].readBuf] +
           pPdoMem_l->txChannelInfo[channelId_p].channelOffset;

#if !defined(PDO_DMA_TRANSFER_TX)
    OPLK_DCACHE_INVALIDATE(pPdo, pdoSize_p);

    OPLK_MEMCPY(pPayload_p, pPdo, pdoSize_p);
#else
    desc.pWrite = (UINT8*)pPayload_p;
    desc.pRead = (UINT8*)pPdo;
    desc.length = (size_t)pdoSize_p;
    desc.fRx = FALSE;
    desc.channelId = channelId_p;
    pdoCopy(&desc);
#endif

    return kErrorOk;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Setup PDO memory info

The function sets up the PDO memory info. For each channel the offset in the
shared buffer and the size are stored.

\param  pPdoChannels_p      Pointer to PDO channel setup.
\param  pPdoMemRegion_p     Pointer to shared PDO memory region.

\return The function returns the size of the used PDO memory
*/
//------------------------------------------------------------------------------
static void setupPdoMemInfo(tPdoChannelSetup* pPdoChannels_p, tPdoMemRegion* pPdoMemRegion_p)
{
    UINT                channelId;
    UINT                offset;
    tPdoChannel*        pPdoChannel;

    offset = 0;
    for (channelId = 0, pPdoChannel = pPdoChannels_p->pRxPdoChannel;
         channelId < pPdoChannels_p->allocation.rxPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE("RPDO %d at offset:%d\n", channelId, offset);
        pPdoMemRegion_p->rxChannelInfo[channelId].channelOffset = offset;
        pPdoMemRegion_p->rxChannelInfo[channelId].readBuf = 0;
        pPdoMemRegion_p->rxChannelInfo[channelId].writeBuf = 1;
        pPdoMemRegion_p->rxChannelInfo[channelId].cleanBuf = 2;
        pPdoMemRegion_p->rxChannelInfo[channelId].newData = 0;
        offset += pPdoChannel->nextChannelOffset - pPdoChannel->offset;
    }

    for (channelId = 0, pPdoChannel = pPdoChannels_p->pTxPdoChannel;
         channelId < pPdoChannels_p->allocation.txPdoChannelCount;
         channelId++, pPdoChannel++)
    {
        //TRACE("TPDO %d at offset:%d\n", channelId, offset);
        pPdoMemRegion_p->txChannelInfo[channelId].channelOffset = offset;
        pPdoMemRegion_p->txChannelInfo[channelId].readBuf = 0;
        pPdoMemRegion_p->txChannelInfo[channelId].writeBuf = 1;
        pPdoMemRegion_p->txChannelInfo[channelId].cleanBuf = 2;
        pPdoMemRegion_p->txChannelInfo[channelId].newData = 0;
        offset += pPdoChannel->nextChannelOffset - pPdoChannel->offset;
    }
    pPdoMemRegion_p->pdoMemSize = offset;

    OPLK_DCACHE_FLUSH(pPdoMemRegion_p, sizeof(tPdoMemRegion));
}

static tOplkError initDma(void)
{
    OPLK_MEMSET(&pdoDmaFifo_l, 0, sizeof(pdoDmaFifo_l));
    OPLK_MEMSET(&pdoDmaCurrent_l, 0, sizeof(pdoDmaCurrent_l));

    fPdoDmaActive_l = FALSE;

    // Clear DMA status register
    IOWR_ALTERA_AVALON_DMA_STATUS(DMA_0_BASE, 0);

    // Clear DMA control register
    IOWR_ALTERA_AVALON_DMA_CONTROL(DMA_0_BASE, 0);

    if (alt_ic_isr_register(0, 4, dmaIsr, NULL, NULL))
        return kErrorNoResource;

    return kErrorOk;
}

static void pdoCopy(tPdoDmaDesc* pPdoDmaDesc_p)
{
    if (fPdoDmaActive_l)
    {
        // Push to fifo
        if ((pdoDmaFifo_l.writeIndex - pdoDmaFifo_l.readIndex) < tabentries(pdoDmaFifo_l.aBuffer))
        {
            UINT writeIndex = pdoDmaFifo_l.writeIndex & (tabentries(pdoDmaFifo_l.aBuffer)-1);

            OPLK_MEMCPY(&pdoDmaFifo_l.aBuffer[writeIndex], pPdoDmaDesc_p, sizeof(*pPdoDmaDesc_p));
            pdoDmaFifo_l.writeIndex++;
        }
        else
        {
            BENCHMARK_TOGGLE(7);
            DEBUG_LVL_ERROR_TRACE("%s DMA fifo full!\n", __func__);
        }

        // Check if DMA is still busy, otherwise push next transfer
        if (!(IORD_ALTERA_AVALON_DMA_STATUS(DMA_0_BASE) & ALTERA_AVALON_DMA_STATUS_BUSY_MSK))
        {
            tPdoDmaDesc desc;

            if (getNextDmaCopy(&desc))
            {
                startDmaCopy(&desc);
            }
            else
            {
                // Very unusual, but maybe?!
                fPdoDmaActive_l = FALSE;
            }
        }
    }
    else
    {
        fPdoDmaActive_l = TRUE;
        startDmaCopy(pPdoDmaDesc_p);
    }
}

#define PDO_DMA_ALIGN(x)    ((((UINT32)x) + 3) & ~3)

static void startDmaCopy(tPdoDmaDesc* pPdoDmaDesc_p)
{
    UINT32  dmaControl;

    BENCHMARK_SET(5);

    // Check for busy DMA, shouldn't be!
    if (IORD_ALTERA_AVALON_DMA_STATUS(DMA_0_BASE) & ALTERA_AVALON_DMA_STATUS_BUSY_MSK)
    {
        DEBUG_LVL_ERROR_TRACE("%s DMA is still busy!\n", __func__);
        BENCHMARK_TOGGLE(7);
        BENCHMARK_RESET(5);
        return;
    }

    IOWR_ALTERA_AVALON_DMA_STATUS(DMA_0_BASE, 0);
    IOWR_ALTERA_AVALON_DMA_CONTROL(DMA_0_BASE, 0);

    // Remember DMA descriptor
    OPLK_MEMCPY(&pdoDmaCurrent_l, pPdoDmaDesc_p, sizeof(pdoDmaCurrent_l));

    if (((UINT32)pPdoDmaDesc_p->pRead) & 3)
    {
        __asm("break");
    }

    if (((UINT32)pPdoDmaDesc_p->pWrite) & 3)
    {
        __asm("break");
    }

    // Align addresses to 32 bit
    pPdoDmaDesc_p->pRead = (UINT8*)PDO_DMA_ALIGN(pPdoDmaDesc_p->pRead);
    pPdoDmaDesc_p->pWrite = (UINT8*)PDO_DMA_ALIGN(pPdoDmaDesc_p->pWrite);
    pPdoDmaDesc_p->length = (size_t)PDO_DMA_ALIGN(pPdoDmaDesc_p->length);

    if (((UINT32)pPdoDmaDesc_p->pRead) & 3)
    {
        __asm("break");
    }

    if (((UINT32)pPdoDmaDesc_p->pWrite) & 3)
    {
        __asm("break");
    }

    if (((UINT32)pPdoDmaDesc_p->length) & 3)
    {
        __asm("break");
    }

    // Set transfer config to DMA registers
    IOWR_ALTERA_AVALON_DMA_RADDRESS(DMA_0_BASE, (UINT32)pPdoDmaDesc_p->pRead);
    IOWR_ALTERA_AVALON_DMA_WADDRESS(DMA_0_BASE, (UINT32)pPdoDmaDesc_p->pWrite);
    IOWR_ALTERA_AVALON_DMA_LENGTH(DMA_0_BASE, (UINT32)pPdoDmaDesc_p->length);

    // Setup DMA control register settings
    dmaControl = ALTERA_AVALON_DMA_CONTROL_WORD_MSK |   /* Word transfer (32 bit) */
                 ALTERA_AVALON_DMA_CONTROL_I_EN_MSK |   /* Interrupt enable when done */
                 ALTERA_AVALON_DMA_CONTROL_LEEN_MSK |   /* End transfer when length reaches zero */
                 0;

    IOWR_ALTERA_AVALON_DMA_CONTROL(DMA_0_BASE, dmaControl);

    dmaControl = IORD_ALTERA_AVALON_DMA_CONTROL(DMA_0_BASE);

    dmaControl |= ALTERA_AVALON_DMA_CONTROL_GO_MSK;

    IOWR_ALTERA_AVALON_DMA_CONTROL(DMA_0_BASE, dmaControl);

    BENCHMARK_RESET(5);
}

static void dmaIsr(void* pArg_p)
{
    tPdoDmaDesc desc;

    UNUSED_PARAMETER(pArg_p);

    BENCHMARK_SET(6);

    // Clear DMA status register
    IOWR_ALTERA_AVALON_DMA_STATUS(DMA_0_BASE, 0);
    IOWR_ALTERA_AVALON_DMA_CONTROL(DMA_0_BASE, 0);

    // Handle Rx PDOs
    if (pdoDmaCurrent_l.fRx)
    {
        OPLK_ATOMIC_T   temp;
        UINT            channelId = pdoDmaCurrent_l.channelId;

        temp = pPdoMem_l->rxChannelInfo[channelId].writeBuf;
        OPLK_ATOMIC_EXCHANGE(&pPdoMem_l->rxChannelInfo[channelId].cleanBuf,
                             temp,
                             pPdoMem_l->rxChannelInfo[channelId].writeBuf);

        pPdoMem_l->rxChannelInfo[channelId].newData = 1;

        // Flush data cache for variables changed in this function
        OPLK_DCACHE_FLUSH(&(pPdoMem_l->rxChannelInfo[channelId].writeBuf), sizeof(OPLK_ATOMIC_T));
        OPLK_DCACHE_FLUSH(&(pPdoMem_l->rxChannelInfo[channelId].newData), sizeof(UINT8));

        //TODO: Release Rx buffer
    }

    if (getNextDmaCopy(&desc))
    {
        startDmaCopy(&desc);
    }
    else
    {
        fPdoDmaActive_l = FALSE;
    }

    BENCHMARK_RESET(6);
}

static BOOL getNextDmaCopy(tPdoDmaDesc* pPdoDmaDesc_p)
{
    if (pPdoDmaDesc_p == NULL)
        return FALSE;

    if ((pdoDmaFifo_l.writeIndex - pdoDmaFifo_l.readIndex) > 0)
    {
        tPdoDmaDesc desc;
        UINT        readIndex = pdoDmaFifo_l.readIndex & (tabentries(pdoDmaFifo_l.aBuffer)-1);

        OPLK_MEMCPY(pPdoDmaDesc_p, &pdoDmaFifo_l.aBuffer[readIndex], sizeof(desc));
        pdoDmaFifo_l.readIndex++;

        return TRUE;
    }

    return FALSE;
}

///\}
