/**
* \ingroup MODULMACROS
*
* \file SingleStep.c
*
* \brief Do everything that is needed to perform a SingleStep
*
*/
/*
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

#include "error_def.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"

/**
  SingleStep
  Do everything that is needed to perform a SingleStep.
  inData:  <wdtAddr(16)> <wdtCtrl(16)> <PC(32)> <SR(16)><eemCtrl(16)> <mdb(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
  eemCtrl: EEM control bits (EEM_EN and optional EEM_CLK_EN)
  mdb: value to be put on the Memory Data Bus before release (in case !0)
*/
HAL_FUNCTION(_hal_SingleStep)
{
    short RetState = HALERR_UNDEFINED_ERROR;
    unsigned short i = 0;
    unsigned short bpCntrlType = BPCNTRL_IF;
    unsigned short extraStep = 0;

    // local variables to enable preserve of EEM trigger block 0
    unsigned short trig0_Bckup_cntrl;
    unsigned short trig0_Bckup_mask;
    unsigned short trig0_Bckup_comb;
    unsigned short trig0_Bckup_cpuStop;
    unsigned short trig0_Bckup_value;

    StreamSafe stream_tmp;
    unsigned char stream_in_release[18];
    unsigned char stream_in_sync[4];
    for(i=0;i<sizeof(stream_in_release);i++)
    {
        STREAM_get_byte(&stream_in_release[i]);
    }
    STREAM_discard_bytes(2);

    for(i=0;i<sizeof(stream_in_sync);i++)
    {
        STREAM_get_byte(&stream_in_sync[i]);
    }


    if (stream_in_release[8] & STATUS_REG_CPUOFF)
    {
      // If the CPU is OFF, only step after the CPU has been awakened by an interrupt.
      // This permits single step to work when the CPU is in LPM0 and 1 (as well as 2-4).
      bpCntrlType = BPCNTRL_NIF;
      // Emulation logic requires an additional step when the CPU is OFF (but only if there is not a pending interrupt).
      cntrl_sig_capture();
      if (!(SetReg_16Bits(0x0000)  & CNTRL_SIG_INTR_REQ))
      {
        extraStep = 1;
      }
    }

    { // Preserve breakpoint block 0
      eem_data_exchange();
      // read control register
      SetReg_16Bits(MX_CNTRL + MX_READ);     // shift in control register address with read request
      trig0_Bckup_cntrl = SetReg_16Bits(0x0000);                 // dummy shift to read out content remember content locally
      // read mask register
      SetReg_16Bits(MX_MASK + MX_READ);
      trig0_Bckup_mask = SetReg_16Bits(0x0000);
      // read combination register
      SetReg_16Bits(MX_COMB + MX_READ);
      trig0_Bckup_comb = SetReg_16Bits(0x0000);
      // read CPU stop reaction register
      SetReg_16Bits(MX_CPUSTOP + MX_READ);
      trig0_Bckup_cpuStop = SetReg_16Bits(0x0000);
      // read out trigger block value register
      SetReg_16Bits(MX_BP + MX_READ);
      trig0_Bckup_value = SetReg_16Bits(0x0000);
    }

    { // Configure "Single Step Trigger" using Trigger Block 0
      eem_data_exchange();
      // write control register
      SetReg_16Bits(MX_CNTRL + MX_WRITE);
      SetReg_16Bits(BPCNTRL_EQ | BPCNTRL_RW_DISABLE | bpCntrlType | BPCNTRL_MAB);
      // write mask register
      SetReg_16Bits(MX_MASK  + MX_WRITE);
      SetReg_16Bits((unsigned short)BPMASK_DONTCARE);
      // write combination register
      SetReg_16Bits(MX_COMB + MX_WRITE);
      SetReg_16Bits(0x0001);
      // write CPU stop reaction register
      SetReg_16Bits(MX_CPUSTOP + MX_WRITE);
      SetReg_16Bits(0x0001);
    }

    // TODO: Take care about the CPU cycles counts of the instructions to be single stepped
    //       Not implemented yet in DLLv3, refer to DLLv2 source code for implementation

    // now restore context and release CPU from JTAG control
    STREAM_internal_stream(stream_in_release, sizeof(stream_in_release), MESSAGE_NO_OUT, 0, &stream_tmp);
    RetState = HAL_RestoreContext_ReleaseJtag(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
    STREAM_external_stream(&stream_tmp);

    if(RetState != 0)
    { // return error if not successful
      return RetState;
    }

    while(1) // this is not an endless loop, it will break if no "extra step" is required
    {
      // Wait for EEM stop reaction
      eem_read_control();
      do
      {
        i++;
      }
      while(!(SetReg_16Bits(0x0000) & 0x0080) && i < 50); // Wait for breakpoint hit

      // Timout?
      if(i >= 50)
      {
        return (HALERR_SINGLESTEP_WAITFOREEM_TIMEOUT);
      }

      // check if an extra step was required
      if(extraStep)
      {
        extraStep = 0;  // reset the "extra step" flag

        // release the CPU once again for an "extra step" from JTAG control
        addr_capture();
        cntrl_sig_release();
      }
      else
      {
        break;
      }
    }

    { // Restore Trigger Block 0
      eem_data_exchange();
      // write control register
      SetReg_16Bits(MX_CNTRL + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_cntrl);
      // write mask register
      SetReg_16Bits(MX_MASK  + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_mask);
      // write combination register
      SetReg_16Bits(MX_COMB + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_comb);
      // write CPU stop reaction register
      SetReg_16Bits(MX_CPUSTOP + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_cpuStop);
      // write trigger block value register
      SetReg_16Bits(MX_BP + MX_WRITE);
      SetReg_16Bits(trig0_Bckup_value);
    }

    // now sync the CPU again to JTAG control and save the current context
    STREAM_internal_stream(stream_in_sync, sizeof(stream_in_sync), MESSAGE_OUT_TO_DLL, 0, &stream_tmp);
    RetState = HAL_SyncJtag_Conditional_SaveContext(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
    STREAM_external_stream(&stream_tmp);

    if(RetState != 0)
    { // return error if not successful
      return RetState;
    }

    return RetState;
}
