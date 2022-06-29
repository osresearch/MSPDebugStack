/*
 * HalGlobalVars.c
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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

//! \ingroup MODULHAL
//! \file HalGlobalVars.c
//! \brief
//!
//! \author  Detlef Fink (03/10/2011)

#include "hw_compiler_specific.h"
#include <string.h>
#include "stream.h"
#include "hal.h"
#include "HalGlobalVars.h"


//extern unsigned char TCLK_saved;
#ifndef UNIX
extern struct stream_funcs *_stream_Funcs;
#endif
extern HAL_INFOS hal_infos_in_ram_;
extern unsigned char mclk_modules[32];
extern unsigned short activeDevice;
extern unsigned char numOfDevices;
extern unsigned short TCE;
extern unsigned long _hal_mclkCntrl0;
extern unsigned long  lAddr;
extern unsigned long  lLen;
extern unsigned long lLenWBlk;
extern unsigned short ret_len;
extern DevicePowerSettings devicePowerSettings;
// ARM specific
extern unsigned long cswValues[4];
extern ARMConfigSettings armConfigSettings;

void globalVarsInit(void)
{
    memset(&hal_infos_in_ram_, 0, sizeof(HAL_INFOS));

    //SyncJtagAssertPor_SaveContextXv2.c
    memset(mclk_modules, 0, sizeof(mclk_modules));
    memset(cswValues, 0, sizeof(cswValues));
    armConfigSettings.scsBase = 0xE000E000; // Default location of the SCS
    activeDevice = 0;
    numOfDevices = 0;
    TCE = 0;
    _hal_mclkCntrl0 = 0x0417;
    // Disable device power settings by default
    memset(&devicePowerSettings, 0, sizeof(devicePowerSettings));
}
