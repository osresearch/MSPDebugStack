/*
 * HalExecBuffered.cpp
 *
 * Sending and receiving on hal level.
 *
 * Copyright (C) 2009 - 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <pch.h>

#include "HalExecBuffered.h"
#include "HalResponse.h"
#include "FetControl.h"
#include "IoChannel.h"
#include "MessageData.h"
#include <boost/thread.hpp>

using namespace TI::DLL430;
using namespace std;


static const uint8_t HEAD_SIZE = 3;
static const uint8_t DATA_SIZE = 251;

static const uint8_t ACT_WAIT = 0x10;
static const uint8_t ACT_ACK_NEEDED = 0x08;
static const uint8_t ACT_ACK_RCV = 0x02;
static const uint8_t ACT_NACK_RCV = 0x04;
static const uint8_t ACT_DATA_COMPLETE = 0x20;

HalExecBuffered::HalExecBuffered ()
 : trans(0)
 , cmd_timeout(3000)
 , hal_error(false)
 , elem(0)
 , async(false)
 , cont(false)
 , tout(0)
 , tmp_channel(0)
 , extClientHandle(0)
 , loopCmdId(0)
{
	memset(buf, 0, sizeof(buf));
}

void HalExecBuffered::createMessage(std::vector<uint8_t>& tdata,
									uint8_t type,
									uint8_t response,
									uint16_t addr,
									bool hasaddr,
									uint8_t * tbuf)
{
	int offset=1;

	tbuf[0] = (uint8_t)(HEAD_SIZE+tdata.size()) & 0xFF;
	tbuf[1] = type;
	tbuf[2] = response;
	tbuf[3] = 0;			// timeout

	if (hasaddr)
	{
		tbuf[4] = addr & 0xFF;
		tbuf[5] = (addr >> 8) & 0xFF;
		offset +=2;
		tbuf[0] +=2;
	}

	if (tdata.size() > 0)
	{
		memcpy(&tbuf[HEAD_SIZE+offset], &tdata[0], tdata.size());
	}
}

void HalExecBuffered::sendAck(uint8_t response, IoChannel& chan, std::vector<uint8_t>& tdata)
{
	uint8_t ack_buf[259];

	ack_buf[0] = (uint8_t)(HEAD_SIZE + tdata.size()) & 0xFF;
	ack_buf[1] = 0x91;
	ack_buf[2] = response;
	ack_buf[3] = 0;

	if (tdata.size() > 0)
	{
		memcpy(&ack_buf[4], &tdata[0], tdata.size());
	}

	chan.write(ack_buf, ack_buf[0] + 1);
}


bool HalExecBuffered::sendAsync(HalExecElement& el, FetControl& fetCtrl, IoChannel& chan, bool continued)
{
	trans=&el;

	int respid=fetCtrl.createResponseId(true);

	if (!fetCtrl.registerResponseHandler(respid, responseHandlerPtr))
	{
		return false;
	}

	el.addTransaction(respid);

	if (continued)
	{
		// report events until kill
		createMessage(el.inData, 0x8A, respid, el.functionId, true, buf);
	}
	else
	{
		// kill loop after event
		createMessage(el.inData, 0x82, respid, el.functionId, true, buf);
	}

	uint8_t length=buf[0]+1;

	// write to output channel and wait for answer
	const size_t len = chan.write(buf, length);
	if (len!=length)
		return false;

	// wait for ACK
	waitForSingleEvent(3000, el, respid);

	if (hal_error)
	{
#ifndef NDEBUG
		printf("unable to activate loop\n");
#endif
		return false;
	}

	return true;
}

// local method for Multi telegram I/O, called by public send function
// HalExecElement is the buffer for input and output data (std::vector)

bool HalExecBuffered::sendElement(HalExecElement& el, FetControl& fetCtrl, IoChannel& chan)
{
	static const uint8_t FOLLOWER = 0x80;

	trans = &el;

	// iterators for payload, init with there own start and end
	std::vector<uint8_t>::iterator start_it = el.inData.begin();
	std::vector<uint8_t>::iterator end_it = el.inData.end();

	size_t pos=0;

	uint8_t ctrl=0x00;
	bool leading=true;

	// always send the first telegram,
	// and check if data completely transmitted -> if not, FOLLOWER flag is set
	while ((ctrl & FOLLOWER) || leading)
	{
		ctrl=0x00;
		// create a new session with new session id
		const int respid = fetCtrl.createResponseId(false);

		if (!fetCtrl.registerResponseHandler(respid, responseHandlerPtr))
		{
			return false;
		}

		el.addTransaction(respid);

		// check HAL function id, needed only for first telegram
		// some telegrams do not need a HAL function ID
		const bool send_addr = (el.getAddrFlag() && leading);

		// calc possible payload size
		size_t space=(DATA_SIZE-(HEAD_SIZE+2+2));
		if (send_addr)
			space-=2;

		// compare possible size with length of rest of data
		if ((el.inData.size()-pos) > space)
		{
			// follower needed
			ctrl |= FOLLOWER;
			end_it=(el.inData.begin()+space+pos);
		}
		else
		{
			// end of payload reached
			end_it=el.inData.end();
		}

		// copy data section to send...
		std::vector<uint8_t> tmp(start_it, end_it);

		// ...and create a telegram
		createMessage(tmp, el.msgType, respid|ctrl, el.functionId, send_addr, buf);

		// save length (could be changed by CRC or channel)
		uint8_t length=buf[0]+1;

		// write to output channel and wait for answer
		const size_t len = chan.write(buf, length);
		if (len != length)
		{
			return false;
		}

		// loop over respond data
		do
		{
			if (this->waitForSingleEvent(cmd_timeout, el, respid))
			{
				if (hal_error)
				{
					hal_error=false;
					return false;
				}
				if (el.checkTransaction(respid, ACT_NACK_RCV))
					return false;
			}
			else
			{
				return false;
			}
			el.changeTransaction(respid, 0x1f, false);	// delete all flags
		} while (!el.checkTransaction(respid, ACT_DATA_COMPLETE));

		leading = false;
		start_it=end_it;
		pos+=space;
	}
	return true;
}

// HalExecCommand call this public function with a list of
// HAL commands, which can be sync, or async

bool HalExecBuffered::send (list_type& list, FetControl& fetCtrl, IoChannel& chan)
{
	if (!fetCtrl.hasCommunication())
	{
		return false;
	}

	elem = &list;

	tmp_channel = &chan;

	for (auto& it : list)
	{
		if (async)
		{
			if (!this->sendAsync(*it, fetCtrl, chan, cont))
				return false;
		}
		else
		{
			if (!this->sendElement(*it, fetCtrl, chan))
			{
				return false;
			}
		}
	}
	return true;
}

// private method for synchronized transmission
// always called, if the answer is expected at once
// returns true if answer is recieved or false if
// timeout happened

bool HalExecBuffered::waitForSingleEvent(int timeout, HalExecElement& el, uint8_t id)
{
	const auto pred = [&el, id]() { return el.checkTransaction(id, ACT_WAIT) != 0; };

	boost::unique_lock<boost::mutex> lock(dataMutex);
#ifndef NDEBUG
	dataCondition.wait(lock, pred);
	return true;
#else
	return dataCondition.wait_for(lock, boost::chrono::milliseconds(timeout), pred);
#endif
}


bool HalExecBuffered::checkException(HalResponse& resp)
{
#ifndef NDEBUG
	for (unsigned int i=0;i<resp.getSize();i++)
		printf("%02x ", resp.at(i));

	printf("\n");
	static const uint16_t ERR_CODE_IMPL = 0x8001;
	static const uint16_t ERR_CODE_MSG_ID = 0x8002;
	static const uint16_t ERR_CODE_CRC = 0x8003;
	static const uint16_t ERR_CODE_RX = 0x8004;
	static const uint16_t ERR_CODE_TX = 0x8005;
#endif

	uint16_t code=((resp.at(4))<<8)+resp.at(3);
	switch (code)
	{
#ifndef NDEBUG
	case ERR_CODE_IMPL:
		printf("Code Error: \n");
		break;

	case ERR_CODE_MSG_ID:
		printf("MSG Error: sent %02x, expected %02x\n", resp.getId(), resp.at(5));
		break;

	case ERR_CODE_CRC:
		printf("CRC Error: expected %02x%02x\n", resp.get().at(6), resp.get().at(5));
		break;

	case ERR_CODE_RX:
		printf("RX Error: \n");
		break;

	case ERR_CODE_TX:
		printf("TX Error: \n");
		break;

	case 0x8006:
		hal_error=true;
		printf("Error: EXCEPTION_RX_OVERFLOW_ERR\n");
		break;

	case 0x8007:
		printf("Error: EXCEPTION_TX_NO_BUFFER\n");
		break;

	case 0x8008:
		printf("Error: EXCEPTION_COM_RESET\n");
		break;

	case 0x8009:
		printf("Error: EXCEPTION_RX_NO_BUFFER\n");
		break;

	case 0x800A:
		printf("Error: EXCEPTION_RX_TO_SMALL_BUFFER\n");
		break;

	case 0x800B:
		printf("Error: EXCEPTION_RX_LENGTH\n");
		break;
#endif
	// current HAL error codes, -1, -2, -3, ....
	case 0xFFFF:
	case 0xFFFE:
	case 0xFFFD:
	case 0xFFFC:
	case 0xFFFB:
	case 0xFFFA:

		hal_error=true;
		break;

	default:
		return false;
	}
	return true;
}

void HalExecBuffered::recv (FetControl& fetCtrl, HalResponse& resp)
{
	const bool msgComplete = resp.getIsComplete();
	tout=0;

	boost::lock_guard<boost::mutex> lock(dataMutex);
	const uint8_t maskedId = resp.getId() & 0x3f; //Without async bit

	switch (resp.getType())
	{
	case HalResponse::Type_Exception:
		this->recv_data(resp);
		this->checkException(resp);
		fetCtrl.unregisterResponseHandler(maskedId, responseHandlerPtr);
		trans->changeTransaction(maskedId, ACT_WAIT, true);
		trans->changeTransaction(maskedId, ACT_NACK_RCV, true);
		//log error
		break;

	case HalResponse::Type_DataRequest:
		fetCtrl.unregisterResponseHandler(maskedId, responseHandlerPtr);
		break;

	case HalResponse::Type_Data:
		trans->changeTransaction(maskedId, ACT_ACK_NEEDED, true);

		if (!this->isAsync())
		{
			this->recv_data(resp);
		}

		if (this->isAsync() || trans->checkTransaction(maskedId, ACT_ACK_NEEDED))
		{
			// if ACK needs additional data
			std::vector<uint8_t> ackdata;

			// add counter sent with 'timeout' field
			if (tout && !this->isAsync())
			{
				ackdata.push_back(tout);
			}
			sendAck(maskedId, *tmp_channel, ackdata);
		}

		if (this->isAsync() )
		{
			if ( resp.get().size() < 6 )
				break;

			if ( info_callback )
			{
				info_callback(  std::shared_ptr<MessageData>(new MessageData(resp.get().begin() + 3, resp.get().end())),
								extClientHandle);
			}

			if ( this->cont )
			{
				break;
			}

			loopCmdId=0;
			trans->changeTransaction(maskedId, ACT_WAIT, true);
		}
		if (msgComplete)
		{
			trans->changeTransaction(maskedId, ACT_DATA_COMPLETE, true);
			fetCtrl.unregisterResponseHandler(resp.getId(), responseHandlerPtr);
		}
		trans->changeTransaction(maskedId, ACT_WAIT, true);
		break;

	case HalResponse::Type_Acknoledge:
		this->recv_data(resp);

		trans->changeTransaction(maskedId, ACT_DATA_COMPLETE, true);
		trans->changeTransaction(maskedId, ACT_ACK_RCV, true);

		if (msgComplete)
		{
			fetCtrl.unregisterResponseHandler(maskedId, responseHandlerPtr);
			trans->changeTransaction(maskedId, ACT_WAIT, true);

			if (this->isAsync())
			{
				static const uint8_t LOOP_FLAG = 0x40;

				loopCmdId = maskedId | LOOP_FLAG;
				fetCtrl.registerResponseHandler(loopCmdId, responseHandlerPtr);
			}
		}
		break;

	case HalResponse::Type_Status:
		// currently not used, prepared for state messages of firmware
		this->recv_data(resp);
		break;

	default:
		trans->changeTransaction(maskedId, ACT_WAIT, true);
#ifndef NDEBUG
		printf("response: %2x\n", resp.getType());
#endif
	}
	dataCondition.notify_all();
}


// for update-read: collect data and create an outData-vector
bool HalExecBuffered::recv_data (HalResponse& resp)
{
	if (elem==nullptr)
		return false;

	if (resp.getSize())
	{
		trans->outData.insert(trans->outData.end(), resp.get().begin()+3, resp.get().end());
		tout=resp.at(2);
	}
	return true;
}

uint8_t HalExecBuffered::getResponseId() const
{
	return loopCmdId;
}

void HalExecBuffered::clearResponseId()
{
	loopCmdId = 0;
}

void HalExecBuffered::setAsyncMode(bool continued)
{
	async=true;
	cont=continued;
}

bool HalExecBuffered::isAsync()
{
	return async;
}

bool HalExecBuffered::isContinuous()
{
	return cont;
}


void HalExecBuffered::setCallBack(const EventCallback& callback, uint32_t clientHandle)
{
	info_callback = callback;
	extClientHandle = clientHandle;
}

void HalExecBuffered::setTimeout(uint32_t msec)
{
	cmd_timeout=msec;
}
