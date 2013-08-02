/**
********************************************************************************
\file   powerlink-module.h

\brief  Header file for openPOWERLINK Linux kernel module

This file contains the necessary definitions for using the openPOWERLINK
Linux module.
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
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

#ifndef _INC_powerlink_module_H_
#define _INC_powerlink_module_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <dll.h>
#include <dllcal.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PLK_CLASS_NAME    "plk"
#define PLK_DEV_NAME      "plk" // used for "/dev" and "/proc" entry
#define PLK_DRV_NAME      "plk"
#define PLK_DEV_FILE      "/dev/plk"
#define PLK_IOC_MAGIC     '='

//------------------------------------------------------------------------------
//  Commands for <ioctl>
//------------------------------------------------------------------------------
#define PLK_CMD_CTRL_EXECUTE_CMD                _IOWR(PLK_IOC_MAGIC, 0, tCtrlCmd)
#define PLK_CMD_CTRL_STORE_INITPARAM            _IOW (PLK_IOC_MAGIC, 1, tCtrlInitParam)
#define PLK_CMD_CTRL_READ_INITPARAM             _IOR (PLK_IOC_MAGIC, 2, tCtrlInitParam)
#define PLK_CMD_CTRL_GET_STATUS                 _IOR (PLK_IOC_MAGIC, 3, UINT16)
#define PLK_CMD_CTRL_GET_HEARTBEAT              _IOR (PLK_IOC_MAGIC, 4, UINT16)
#define PLK_CMD_POST_EVENT                      _IOW (PLK_IOC_MAGIC, 5, tEplEvent)
#define PLK_CMD_GET_EVENT                       _IOR (PLK_IOC_MAGIC, 6, tEplEvent)
#define PLK_CMD_DLLCAL_ASYNCSEND                _IO  (PLK_IOC_MAGIC, 7)
#define PLK_CMD_ERRHND_WRITE                    _IOW (PLK_IOC_MAGIC, 8, tErrHndIoctl)
#define PLK_CMD_ERRHND_READ                     _IOR (PLK_IOC_MAGIC, 9, tErrHndIoctl)
#define PLK_CMD_PDO_SYNC                        _IO  (PLK_IOC_MAGIC, 10)

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
typedef struct
{
    tDllCalQueue            queue;
    void*                   pData;
    size_t                  size;
} tIoctlDllCalAsync;

typedef struct
{
    void*                   pData;
    size_t                  size;
} tIoctlBufInfo;

typedef struct
{
    UINT32                  offset;
    UINT32                  errVal;
} tErrHndIoctl;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------


#endif /* _INC_powerlink-module_H_ */



