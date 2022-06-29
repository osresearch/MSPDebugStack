/*
 * stream.c
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
//! \ingroup MODULBIOS
//! \file stream.c
//! \brief managment of input and output data between USB and HAL, and HAL and HAL
//! \details Get/put a data stream (message packet) form/to the bios rx/tx layer. Monitor both streams
//! and send response packets and request next message packets. Also is stream.c the interface to HAL. On
//! HAL init the call addresses are handover to the HAL.
//! \author Detlef Fink (09/21/2010)


#include "bios.h"
#include "USB_API/USB_Common/types.h"          // Basic Type declarations
#include "usbConstructs.h"
#include "hw_compiler_specific.h"		// included from bios.h?, not needed for LPT
#include "stream.h"
#include "string.h"

struct _StreamInput_
{
    unsigned char  *data_start;
    unsigned char  *data_end;
    unsigned char  *data_ptr;
};
typedef struct _StreamInput_ StreamInput;
StreamInput stream_input_;

// prototyes
unsigned char _stream_out_change_type(unsigned char res_type);
unsigned char _stream_out_change_msg_id(unsigned char msg_id);
short _stream_out_init(unsigned char msg_id, unsigned char res_type);
short _stream_flush(void);
short _stream_put_bytes(void* data, unsigned short size);
short _stream_put_byte(unsigned char data);
short _stream_put_word(unsigned short data);
short _stream_put_long(unsigned long data);
short _stream_in_init(void *ptr, unsigned char no);
short _stream_internal_stream(unsigned char *data_in, unsigned short size_in, unsigned char *data_out, unsigned short size_out, StreamSafe *tmp_record);
short _stream_external_stream(StreamSafe *tmp_record);
short _stream_get_byte(unsigned char *data );
short _stream_get_word(unsigned short *data);
short _stream_get_long(unsigned long *data);
short _stream_get_buffer(void **buffer, unsigned short *size);
short _stream_discard_bytes(unsigned short count);
short _stream_memcpy(unsigned char* destination, unsigned char* source, unsigned short count);
short _stream_getFlags(void);
short _stream_putFlags(short flags, short mask);

short _stream_getSharedVariable(unsigned short IdMemoryType, unsigned short** Address);
short _stream_deleteSharedVariable(unsigned short IdMemoryType);

short _stream_send_debug(char *buffer, unsigned short size);

// stream functions
struct stream_funcs _stream_Funcs =
{
    _stream_out_init,
    _stream_flush,
    _stream_put_byte,
    _stream_put_bytes,
    _stream_put_word,
    _stream_put_long,
    _stream_in_init,
    _stream_internal_stream,
    _stream_external_stream,
    _stream_out_change_type,
    _stream_get_byte,
    _stream_get_word,
    _stream_get_long,
    _stream_get_buffer,
    _stream_discard_bytes,
    _stream_memcpy,
    // used form downgrader
    BIOS_LedOn,
    BIOS_LedOff,
    BIOS_LedBlink,
    BIOS_LedFlash,
    BIOS_LedAlternate,
    _stream_send_debug,
    _stream_getSharedVariable,
    _stream_deleteSharedVariable
};

#define SHARED_VARIABLES_SIZE   128 // words -> 256 bytes

__no_init unsigned short sharedVariables_[128] @ "SHARED_VARIABLES";
#pragma required = sharedVariables_

void STREAM_resetSharedVariables()
{
    short i = 0;
    for(i = 0; i < SHARED_VARIABLES_SIZE; i++)
    {
        sharedVariables_[i] = 0x00;
    }
}

#pragma optimize = low
short _stream_deleteSharedVariable(unsigned short IdMemoryType)
{
    short i = 0;
    for(i = 0; i < SHARED_VARIABLES_SIZE ; i = i+2 )
    {
        // check if Shared Memory Type is already requested
        if(sharedVariables_[i] == IdMemoryType)
        {
            // if yes set to init value 0x00
            sharedVariables_[i] = 0x00;
            sharedVariables_[i+1] = 0x00;
            return 1;
        }
    }
    return 0;
}

#pragma optimize = low
short _stream_getSharedVariable(unsigned short IdMemoryType, unsigned short** Address)
{
    short i = 0;
    for(i = 0; i < SHARED_VARIABLES_SIZE ; i = i+2 )
    {
        // check if Shared Memory Type is already requested
        if(sharedVariables_[i] == IdMemoryType)
        {
            // if yes just return the address of the requested variable
            *Address = &sharedVariables_[i+1];
            return 1;
        }
    }

    // if Shared Memory Type was not requested -> request it
    for(i = 0; i < SHARED_VARIABLES_SIZE ; i = i+2 )
    {
        // check for free space to place requested variable
        if(sharedVariables_[i] == 0x00)
        {
          // set Shared Memory Type of variable
          sharedVariables_[i] = IdMemoryType;
          // return the address of the requested variable
          *Address = &sharedVariables_[i+1];
          sharedVariables_[i+1] = 0x0000;
          return 1;
        }
    }
    *Address = 0;
    return 0;
}

//! \brief Change the type of the active response
//! \param[in] res_type new type of response
//! \return type before change
unsigned char _stream_out_change_type(unsigned char res_type)
{
    unsigned char ret_value;

    ret_value = bios_tx_record_.data[bios_tx_record_.active][1];
    bios_tx_record_.data[bios_tx_record_.active][1] = res_type;
    return(ret_value);
}

//! \brief Change the message id of the active response
//! \param[in] msg_id new message id of response
//! \return message id before change
unsigned char _stream_out_change_msg_id(unsigned char msg_id)
{
    unsigned char ret_value;

    ret_value = bios_tx_record_.data[bios_tx_record_.active][2];
    bios_tx_record_.data[bios_tx_record_.active][2] = msg_id;
    return(ret_value);
}

//! \brief Setup a transmit buffer (response)
//! \details Setup a response with given id and response typ. The function blocks until a output buffer is free.
//! \param[in] msg_id id of response
//! \param[in] res_typ response type of response
//! \return always 0
short _stream_out_init(unsigned char msg_id, unsigned char res_type)
{
    while((bios_tx_record_.state[bios_tx_record_.active] & (BIOS_TX_BUSY | BIOS_TX_TO_SEND | BIOS_TX_WAIT_ON_ACK)) && (bios_global_timer_[BIOS_TIMER_TX].count > 1));

    if ((bios_tx_record_.state[bios_tx_record_.active] & (BIOS_TX_BUSY | BIOS_TX_TO_SEND | BIOS_TX_WAIT_ON_ACK)) && res_type != RESPTYP_STATUS)
    {
        BIOS_UsbTxError();
        return(EXCEPTION_TX_NO_BUFFER);
    } // status message could not be send. Do not clear TX buffer - perhaps other message send acitve
    else if((bios_tx_record_.state[bios_tx_record_.active] & (BIOS_TX_BUSY | BIOS_TX_TO_SEND | BIOS_TX_WAIT_ON_ACK)) && res_type == RESPTYP_STATUS)
    {
        return(EXCEPTION_TX_NO_BUFFER);
    }
    bios_tx_record_.data[bios_tx_record_.active][1] = res_type;
    bios_tx_record_.data[bios_tx_record_.active][2] = msg_id;
    bios_tx_record_.data[bios_tx_record_.active][3] = 0x00;
    bios_tx_record_.count[bios_tx_record_.active] = 3; // header + crc sended ever
    return(0);
}

//! \brief Finish writing to transmit buffer
//! \details Calculate the CRC, sets the output state machine and start transmiting. Inserts between payload and crc a byte if the payload is odd.
//! \return status (error/ok) of function
short _stream_flush(void)
{
    static unsigned char resp_id = 0;
    unsigned short i;
    unsigned short crc;

    if(!(bios_tx_record_.state[bios_tx_record_.active] &  BIOS_TX_TO_SEND))
    {
        DIAG_SUPPRESS(Pa082)
        if(bios_tx_record_.data[bios_tx_record_.active][MESSAGE_CMDTYP_POS] != RESPTYP_DATA)
        {
            bios_tx_record_.data[bios_tx_record_.active][MESSAGE_MSG_ID_POS] &= ~0x80;
        }
        else if((bios_tx_record_.data[bios_tx_record_.active][MESSAGE_MSG_ID_POS] & 0x80) || resp_id)
        {
            if(resp_id < 255)
            {
                resp_id++;
            }
            else
            {
                resp_id = 1;
            }
            bios_tx_record_.data[bios_tx_record_.active][3] = resp_id;
        }
        bios_tx_record_.data[bios_tx_record_.active][0] = bios_tx_record_.count[bios_tx_record_.active];
        if(!(bios_tx_record_.data[bios_tx_record_.active][0] & 0x01)) // payload must be a multiply of word size for CRC
        {
            bios_tx_record_.data[bios_tx_record_.active][++bios_tx_record_.count[bios_tx_record_.active]] = 0x00;
        }
        crc = 0x0000;
        for(i = 0; i < (bios_tx_record_.count[bios_tx_record_.active]+1) / 2; i++)
        {
            crc ^= bios_tx_record_.datas[bios_tx_record_.active][i];
        }
        crc ^= 0xFFFF;
        bios_tx_record_.datas[bios_tx_record_.active][bios_tx_record_.count[bios_tx_record_.active] / 2 + 1] = crc;
        bios_tx_record_.state[bios_tx_record_.active] |=  BIOS_TX_TO_SEND;
        bios_tx_record_.state[bios_tx_record_.active] &= ~BIOS_TX_BUSY;
        if(!(bios_tx_record_.data[bios_tx_record_.active][MESSAGE_MSG_ID_POS] & 0x80))
        {
            resp_id = 0;
        }
        if(!(bios_tx_record_.data[bios_tx_record_.active][MESSAGE_MSG_ID_POS] & 0x40))
        {
            if(bios_tx_record_.data[bios_tx_record_.active][MESSAGE_CMDTYP_POS] == RESPTYP_DATA)
            {
                bios_tx_record_.state[bios_tx_record_.active] |= BIOS_TX_WAIT_ON_ACK;
            }
        }
        else
        {
            bios_tx_record_.state[bios_tx_record_.active] &= ~BIOS_TX_WAIT_ON_ACK;
        }

        DIAG_DEFAULT(Pa082)
        BIOS_PrepareTx(bios_tx_record_.count[bios_tx_record_.active] + 3);
        bios_tx_record_.active++;
        if(bios_tx_record_.active >= BIOS_TX_QUEUS)
        {
            bios_tx_record_.active = 0;
        }
        BIOS_StartTx();
    }
    return(0);
}

//! \brief Write one or more bytes to the output(transmit) buffer
//! \details Write data to the output buffer. If the output buffer is full, the buffer is flush and a new buffer are allocated.
//! \param[in] *data pointer to data to write to the buffer
//! \param[in] size count of data to write to buffer
//! \return status (error/ok) of function
short _stream_put_bytes(void *data, unsigned short size)
{
    unsigned short size_counter;
    unsigned char tmp_id;
    unsigned char tmp_type;
    int ret_value = 0;
    unsigned char *pData = data;
    unsigned short current_count;

    if(bios_tx_record_.ext_data == NULL)
    {
        size_counter = size;
        do
        {
            current_count = bios_tx_record_.count[bios_tx_record_.active];
            if((current_count + size_counter) >= (BIOS_TX_SIZE-3))
            {
                tmp_type = bios_tx_record_.data[bios_tx_record_.active][1];
                tmp_id = bios_tx_record_.data[bios_tx_record_.active][2];

                bios_tx_record_.data[bios_tx_record_.active][2] |= 0x80; // mark one or more packet follow
                memcpy(&bios_tx_record_.data[bios_tx_record_.active][current_count+1],pData,(BIOS_TX_SIZE-3) - current_count);
                bios_tx_record_.count[bios_tx_record_.active] = (BIOS_TX_SIZE-3);

                BIOS_LedFlash(BIOS_LED_MODE,20);
                _stream_flush();
                ret_value = _stream_out_init(tmp_id,tmp_type);
                size_counter -= ((BIOS_TX_SIZE-3) - current_count);
                pData += ((BIOS_TX_SIZE-3) - current_count); // Advance the data pointer;
            }
            else
            {
                memcpy(&bios_tx_record_.data[bios_tx_record_.active][current_count+1],pData,size_counter);
                bios_tx_record_.count[bios_tx_record_.active] = current_count + size_counter;
                size_counter = 0;
            }

        } while(size_counter);
    }
    else if(bios_tx_record_.ext_data != (unsigned short*)0xFFFF)
    {
        DIAG_SUPPRESS(Pa082)
        for(size_counter = 0;
            (size_counter < size) && (bios_tx_record_.ext_counter < bios_tx_record_.ext_size);
            size_counter++, bios_tx_record_.ext_counter++)
        {
            *((unsigned char*)bios_tx_record_.ext_data+bios_tx_record_.ext_counter) = *((unsigned char*)data+size_counter);
        }
        DIAG_DEFAULT(Pa082)
    }
    return(ret_value);
}

//! \brief Write one byte to the output(transmit) buffer, by using _stream_put_bytes
//! \param[in] data byto to write into the buffer
//! \return status (error/ok) of function
short _stream_put_byte(unsigned char data)
{
    return(_stream_put_bytes(&data,sizeof(unsigned char)));
}

//! \brief Write a short value (16 bit value ) to the output(transmit) buffer, by using _stream_put_bytes
//!  \param[in] data short value to write into the buffer
//! \return status (error/ok) of function
short _stream_put_word(unsigned short data)
{
    return(_stream_put_bytes(&data,sizeof(unsigned short)));
}

//! \brief Write a long value (32 bit value ) to the output(transmit) buffer, by using _stream_put_bytes
//! \param[in] data long value to write into the buffer
//! \return status (error/ok) of function
short _stream_put_long(unsigned long data)
{
    return(_stream_put_bytes(&data,sizeof(unsigned long)));
}

//! \brief Setup rx buffer for using by HAL
//! \param[in] *ptr pointer to BiosRxRecord structure
//! \param[in] no number(index) of rx buffer to process
//! \return status (error/ok) of function
short _stream_in_init(void *ptr, unsigned char no)
{
    BiosRxRecord *ptr_tmp;
    ptr_tmp = (BiosRxRecord*)ptr;

    stream_input_.data_start =(*ptr_tmp).data[no];
    stream_input_.data_end = stream_input_.data_start + stream_input_.data_start[MESSAGE_SIZE_POS];
    stream_input_.data_ptr = &stream_input_.data_start[MESSAGE_PAYLOAD_POS];
    return(0);
}

//! \brief Set up a internal stream
//! \details and safe the active stream paramters/status to a external buffer.
//! \param[in] *data_in, pointer to new stream data, they haven't a header and CRC (matchable to payload of stream from USB)
//! \param[in] size_in, size of data
//! \param[out] *data_out, pointer to a output buffer
//! \n NULL = not output data, if output data generate they are droped
//! \n 1 = output data are routed to originale output
//! \param[out] size_out, size of output buffer
//! \n must be 0, if data_out NULL or 1
//! \param[out] *tmp_record, pointer to a buffer to store current stream settings
//! \return always 0
short _stream_internal_stream(unsigned char *data_in, unsigned short size_in, unsigned char *data_out, unsigned short size_out, StreamSafe *tmp_record)
{
    // safe Rx
    memcpy((StreamSafe*)tmp_record, (StreamInput*)&stream_input_, sizeof(stream_input_));
    // safe Tx
    (*tmp_record).ext_data = bios_tx_record_.ext_data;
    (*tmp_record).ext_size = bios_tx_record_.ext_size;
    (*tmp_record).ext_counter = bios_tx_record_.ext_counter;
    stream_input_.data_start = data_in;
    stream_input_.data_end = stream_input_.data_start + size_in;
    stream_input_.data_ptr = data_in;
    // set temporary output for stream
    if(data_out != (unsigned char*)0x0001)
    {
        if(data_out == NULL)
        {
            bios_tx_record_.ext_data = (unsigned short*)0xFFFF;
        }
        else
        {
            bios_tx_record_.ext_data = (unsigned short*)data_out;
        }
        bios_tx_record_.ext_size = size_out;
        bios_tx_record_.ext_counter = 0;
    }
    return(0);
}

//! \brief retore stream to previous stream
//! \param[in] *tmp_record, pointer to a buffer with stored stream settings
//! \return always 0
short _stream_external_stream(StreamSafe *tmp_record)
{
    // restore Rx
    memcpy((StreamInput*)&stream_input_, (StreamSafe*)tmp_record, sizeof(stream_input_));
    // restore Tx
    bios_tx_record_.ext_data = (*tmp_record).ext_data;
    bios_tx_record_.ext_size = (*tmp_record).ext_size;
    bios_tx_record_.ext_counter = (*tmp_record).ext_counter;
    return(0);
}

//! \brief Get a byte from active input buffer
//! \details Get a byte from active buffer. If buffer is empty, the next message is requested if it is a multipacket message.
//! \param[out] *data pointer to a byte to return value
//! \return 0 = more data avaible, 1 = last data, -1 = failed
short _stream_get_byte(unsigned char *data)
{
    short ret_value = 0;

    if(stream_input_.data_ptr > stream_input_.data_end)
    {
        ret_value = -1;
    }
    else
    {
        *data = *stream_input_.data_ptr++;
        if(stream_input_.data_ptr > stream_input_.data_end)
        {
            ret_value = 1;
        }
    }
    return(ret_value);
}

//! \brief Get a short from active input buffer (stream)
//! \details Get a short (16 bit value) from active buffer. If buffer is empty, the next message is requested
//! \param[out] *data pointer to short to return value
//! \return status, 0 = ok, 1 = last data, -1 = faild
short _stream_get_word(unsigned short *data)
{
    short ret_value = 0;

    unsigned short Word;
    unsigned char tmp_char;

    if((PTR_FOR_CMP)stream_input_.data_ptr & 1)
    {
        ret_value = _stream_get_byte(&tmp_char);
        Word = tmp_char;
        if(!ret_value)
        {
            ret_value = _stream_get_byte(&tmp_char);
            Word += tmp_char << 8;
        }
        else
        {
            ret_value = -1;
        }
        *data = Word;
    }
    else
    {
        if(stream_input_.data_ptr > stream_input_.data_end)
        {
            ret_value = -2;
        }
        else
        {
            *data = *(unsigned short*)stream_input_.data_ptr;
            stream_input_.data_ptr +=2;
            if(stream_input_.data_ptr > stream_input_.data_end)
            {
                ret_value = 1;
            }
        }
    }
    return(ret_value);
}



//! \brief Get a long from active input buffer (stream)
//! \details Get a long (32 bit value) from active buffer. If buffer is empty, the next message is requested
//! \param[out] *data pointer to a long to return value
//! \return status, 0 = ok, 1 = last data, -1 = faild
short _stream_get_long(unsigned long *data)
{
    short ret_value = 0;

    unsigned char *tmp_char;

    if((PTR_FOR_CMP)stream_input_.data_ptr & 1)
    {
        tmp_char = (unsigned char*)data;
        *data = 0;
        if(_stream_get_byte(tmp_char) != 0)
        {
            ret_value = -1;
            goto stream_get_long_exit;
        }
        if(_stream_get_byte(tmp_char+1) != 0)
        {
            ret_value = -2;
            goto stream_get_long_exit;
        }
        if(_stream_get_byte(tmp_char+2) != 0)
        {
            ret_value = -3;
            goto stream_get_long_exit;
        }
        ret_value = _stream_get_byte(tmp_char+3);
    }
    else
    {
        if(stream_input_.data_ptr > stream_input_.data_end)
        {
            ret_value = -5;
        }
        else
        {
            *data = *(unsigned long*)stream_input_.data_ptr;
            stream_input_.data_ptr +=4;
            if(stream_input_.data_ptr > stream_input_.data_end)
            {
                ret_value = 1;
            }
        }
    }
stream_get_long_exit:
    return(ret_value);
}

//! \brief Get a pointer to the data buffer
short _stream_get_buffer(void **buffer, unsigned short *size)
{
    *buffer = stream_input_.data_ptr;
    *size = stream_input_.data_end - stream_input_.data_ptr + 1;

    stream_input_.data_ptr = stream_input_.data_end + 4;

    return 1;
}

//! \brief Skip byte(s) to read form rx buffer
//! \details If more bytes discarded as in the buffer, the next message is requested.
//! \param[in] count count of bytes to skip
//! \return status, 0 = ok, 1 = last data, -1 = faild
short _stream_discard_bytes(unsigned short count)
{
    short ret_value = -1;
    unsigned char dummy;

    for(;count; count--)
    {
        ret_value = _stream_get_byte(&dummy);
        if(ret_value == -1)
        {
            break;
        }
    }
    return(ret_value);
}

//! \brief Copy count byte form source to destination
//! \param[out] destination copy to ...
//! \param[in] source copy from ...
//! \param[in] count count of byte to copy
//! \return status
short _stream_memcpy(unsigned char* destination, unsigned char* source, unsigned short count)
{
    while(count)
    {
        *destination++ = *source++;
        --count;
    }
    return(0);
}

short _stream_send_debug(char *buffer, unsigned short size)
{
    return cdcSendDataInBackground((BYTE *)buffer, size, 1, 3);
}

static unsigned short * ITick = 0;
static unsigned short * TimerTick = 0;
static unsigned short * TimeTick = 0;

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
   switch (__even_in_range(TA0IV, TA0IV_TA0IFG))
   {
     case TA0IV_TA0IFG:
            if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIME_TICK, &TimeTick))
            {
              if(*(unsigned short*)TimeTick == 0)
              {
                    ++(*(unsigned short*)TimeTick);
              }
              ++(*(unsigned short*)TimeTick);
            }
            break;
     default:
            break;
   }
}

// MSP-FET specific interrupts -------------------------------------------------
#ifdef MSP_FET

#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
{
    if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIMER_TICK, &TimerTick))
    {
        (*(unsigned short*)TimerTick)++;
    }
}
#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR_(void)
{
    switch (__even_in_range(TB0IV, TB0IV_TB0IFG))
   {
       case TB0IV_TB0IFG:
           if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_I_TICK, &ITick))
           {
               ++(*(unsigned short*)ITick);
           }
           break;
       default:
           break;
   }
}
#endif

// eZ-FET specific interrupts -------------------------------------------------
#ifdef eZ_FET

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR_(void)
{
    if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_TIMER_TICK, &TimerTick))
    {
        (*(unsigned short*)TimerTick)++;
    }
}

#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
{
   switch (__even_in_range(TA2IV, TA2IV_TA2IFG))
   {
     case TA2IV_TA2IFG:
            if(STREAM_getSharedVariable(ID_SHARED_MEMORY_TYPE_I_TICK, &ITick))
            {
                ++(*(unsigned short*)ITick);
            }
            break;
     default:
            break;
   }
}
#endif
