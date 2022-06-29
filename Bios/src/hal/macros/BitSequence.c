/**
* \ingroup MODULMACROS
*
* \file BitSequence.c
*
* \brief Streaming of single JTAG signals
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#include "error_def.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  BitSequence
  Streaming of single JTAG signals. Used to manually set the level of one or
  more JTAG signal to a dedicated level. E.g. can be used to manually pull
  the RST pin low.
  inData:  <count(8)> (<writeBits(16)> <maskBits(16)> <delay(16)>){*}
  outData: -

          count: number of following bit manipulations
          maskBits: the mask to apply when setting bits
          writeBits: the values to set the bits to
  Bit definition:
          Bit 0 - TMS
          Bit 1 - TDI
          Bit 2 - TDO (writing this bit has no effect since its an input)
          Bit 3 - TCK
          Bit 4 - ------
          Bit 5 - SELT#
          Bit 6 - TGTRST
          Bit 7 - ------
          Bit 8 - TEST
          Bit 9 - ------
          Bit 10- ENTDI2TDO
          Bit 11- VCCTON
          Bit 12- TDIOFF
          Bit 13- VF2TDI
          Bit 14- VF2TEST
          Bit 15- ------
*/

#define PinTMS  0x01             //Bit0
#define PinTDI  0x02             //Bit1
#define PinTDO  0x04             //Bit2
#define PinTCK  0x08             //Bit3
#define PinTGTRST 0x040          //Bit6
#define PinTest 0x0100           //Bit8

HAL_FUNCTION(_hal_BitSequence)
{
    unsigned char numBitSequences;
    unsigned short writeBits;
    unsigned short maskBits;
    unsigned short delay;

    if(STREAM_get_byte(&numBitSequences) < 0)
    {
        return HALERR_NO_NUM_BITS;
    }

    while(numBitSequences)
    {
        if(STREAM_get_word(&writeBits) < 0)
        {
            return HALERR_ARRAY_SIZE_MISMATCH;
        }

        if(STREAM_get_word(&maskBits) < 0)
        {
            return HALERR_ARRAY_SIZE_MISMATCH;
        }

        if(STREAM_get_word(&delay) < 0)
        {
            return HALERR_ARRAY_SIZE_MISMATCH;
        }

        // Set reset bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTGTRST)
        {
            if(writeBits & PinTGTRST)
            {
                IHIL_SetReset(1);
            }
            else
            {
                IHIL_SetReset(0);
            }
        }

        // Set test bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTest)
        {
            if(writeBits & PinTest)
            {
                IHIL_SetTest(1);
            }
            else
            {
                IHIL_SetTest(0);
            }
        }

        // Set TMS bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTMS)
        {
            if(writeBits & PinTMS)
            {
                IHIL_SetTMS(1);
            }
            else
            {
                IHIL_SetTMS(0);
            }
        }

        // Set TCK bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTCK)
        {
            if(writeBits & PinTCK)
            {
                IHIL_SetTCK(1);
            }
            else
            {
                IHIL_SetTCK(0);
            }
        }

        // Set TDI bit using macros - automatically handles SBW/JTAG mode
        if(maskBits & PinTDI)
        {
            if(writeBits & PinTDI)
            {
                IHIL_SetTDI(1);
            }
            else
            {
                IHIL_SetTDI(0);
            }
        }


        --numBitSequences;

        IHIL_Delay_1ms(delay);
    }
    return 0;
}

