/**
* \ingroup MODULMACROS
*
* \file WaitForStorage.c
*
* \brief Check the state storage register for storage events and return storage * data on change
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
#include "JtagId.h"

unsigned short lastTraceWritePos = 0;

/**
  WaitForStorage
  Check the state storage register for storage events and return storage data on change.
  This is best used with the ExecLoop command type.
*/

HAL_FUNCTION(_hal_WaitForStorage)
{
    short RetState = 2;
    unsigned short sStStorCtrl = 0;

    if(!jtagIdIsValid(cntrl_sig_capture()))
    {
        return 2;
    }
    eem_data_exchange();
    SetReg_16Bits(0x9F);
    sStStorCtrl = SetReg_16Bits(0);

    //Storage written bit
    if (sStStorCtrl & 0x100)
    {
        //Variable watch mode
        if ( (sStStorCtrl & 0x6) == 0x4 )
        {
            unsigned short readPos = 0;

            STREAM_put_word(VARIABLE_WATCH_FLAG);

            //Reset storage
            SetReg_16Bits(0x9E);
            SetReg_16Bits(sStStorCtrl | 0x40);

            for (readPos = 0; readPos < 8; ++readPos)
            {
                //Slice bits autoincrement on reading

                SetReg_16Bits(0x9A);
                SetReg_16Bits(readPos << 2);

                //Read MAB value
                SetReg_16Bits(0x9D);
                STREAM_put_long(SetReg_16Bits(0));
                //Read MDB value

                SetReg_16Bits(0x9D);
                STREAM_put_word(SetReg_16Bits(0));
            }

            RetState = 1;
        }

        //Trace mode
        else
        {
            short writePos = 0;
            short newEntries = 0;

            SetReg_16Bits(0x9B);

            writePos = SetReg_16Bits(0) >> 10;

            //store until full bit
            if (sStStorCtrl & 0x8)
            {
                //storage full bit (or writePos = 0, if written after polling storage control)
                if ((sStStorCtrl & 0x200) || (writePos == 0))
                {
                    //Disable and reset bits when full to avoid further events
                    //Enable bit will be set again with next reset from DLL
                    SetReg_16Bits(0x9E);
                    SetReg_16Bits((sStStorCtrl | 0x40) & ~0x1);

                    newEntries = 8 - lastTraceWritePos;
                }
                else if (writePos != lastTraceWritePos)
                {
                    newEntries = writePos - lastTraceWritePos;
                }
            }
            //Store continuously (only history mode: stop on trigger)
            else
            {
                static unsigned int noChangeSince = 0;

                if (writePos != lastTraceWritePos)
                {
                    noChangeSince = 0;
                }
                else if (++noChangeSince == 20)
                {
                    newEntries = (sStStorCtrl & 0x200) ? 8 : writePos;
                }
            }

            lastTraceWritePos = writePos;

            if (newEntries > 0)
            {
                unsigned short readPos = (writePos - newEntries) & 0x7;

                STREAM_put_word(STATE_STORAGE_FLAG);
                STREAM_put_word(newEntries);

                do
                {
                    //Slice bits autoincrement on reading

                    SetReg_16Bits(0x9A);
                    SetReg_16Bits(readPos << 2);

                    //Read MAB value
                    SetReg_16Bits(0x9D);
                    STREAM_put_long(SetReg_16Bits(0));

                    //Read MDB value
                    SetReg_16Bits(0x9D);
                    STREAM_put_word(SetReg_16Bits(0));

                    //Read Ctrl value
                    SetReg_16Bits(0x9D);
                    STREAM_put_word(SetReg_16Bits(0));

                    if (++readPos > 7)
                    {
                        readPos = 0;
                    }

                } while (readPos != writePos);

                RetState = 1;
            }
        }
    }

    return RetState;
}
