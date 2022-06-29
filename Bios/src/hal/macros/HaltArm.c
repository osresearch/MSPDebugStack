/*
 * \ingroup MODULMACROS
 *
 * \file HaltArm.c
 *
 * \brief <FILEBRIEF>
 *
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "JTAG_defs.h"
#include "error_def.h"
#include "global_variables.h"

/**
  Halt: Halts the Core
*/
#if defined(MSP430_UIF) || defined(MSP_FET)
extern ARMConfigSettings armConfigSettings;
#endif

HAL_FUNCTION(_hal_HaltArm)
{
    short retVal = 0;
#if defined(MSP430_UIF) || defined(MSP_FET)
    unsigned short* syncWithRunVarAddress = getTargetRunningVar();
    if(syncWithRunVarAddress)
    {
        *syncWithRunVarAddress = 0x0000;
    }
    // Set system and debug power up if cleared
    if( powerUpArm() != 0)
    {
        return HALERR_UNDEFINED_ERROR;
    }

    if(checkWakeup() != 0)
    {
        retVal = HALERR_UNDEFINED_ERROR;
    }

    unsigned char retry = MAX_RETRY;

    unsigned long data = DBGKEY | C_HALT | C_DEBUGEN;
    IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, WRITE);

    do
    {
        IHIL_Write_Read_Mem_Ap(0, DHCSR, &data, READ);
    } while(!(data & S_HALT) && --retry);

    if(!retry)
    {
        retVal = HALERR_UNDEFINED_ERROR;
    }

    // Read out why did we halt?
    IHIL_Write_Read_Mem_Ap(0, DFSR, &data, READ);
    STREAM_put_long(data);
#endif
    return retVal;
}
