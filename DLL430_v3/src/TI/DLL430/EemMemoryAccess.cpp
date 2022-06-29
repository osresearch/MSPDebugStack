/*
 * EemMemoryAccess.cpp
 *
 * Memory class for EEM register access.
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
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
#include "EemMemoryAccess.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "IDeviceHandle.h"

using namespace TI::DLL430;

EemMemoryAccess::EemMemoryAccess(MemoryArea::Name name, IDeviceHandle* devHandle,
	uint32_t start,
	uint32_t size,
	uint32_t seg,
	uint32_t banks,
	bool mapped,
	uint8_t bits)
 : MemoryAreaBase(name, devHandle, start, size, seg, banks, mapped, false/*isprotected*/, 0xff)
 , maxAddress(0xFE)
 , words((bits + 15) / 16) //number of words that bits fit in
 , readPtr(nullptr)
{
}

bool EemMemoryAccess::doRead(uint32_t address, uint32_t* value)
{
	if (address > this->maxAddress)
		return false;

	this->queue.push_back(static_cast<uint8_t>(address | 0x1));

	readPtr = value;
	return true;
}

bool EemMemoryAccess::doWrite(uint32_t address, uint32_t value)
{
	if (address > this->maxAddress)
		return false;

	this->queue.push_back(static_cast<uint8_t>(address & ~0x1));
	for (uint8_t i = 0; i < this->words * 2; ++i)
		this->queue.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));

	return true;
}

bool EemMemoryAccess::preSync()
{
	if (queue.empty())
		return true;

	this->elements.clear();
	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_EemDataExchange));

	if (!el)
	{
		return false;
	}

	el->appendInputData8(static_cast<uint8_t>(queue.size()));
	el->appendInputData8(queue.begin(), queue.end());

	this->elements.emplace_back(el);
	return true;
}

bool EemMemoryAccess::postSync(const HalExecCommand& cmd)
{
	if (cmd.elements[0] == NULL)
	{
		return false;
	}

	const HalExecElement& el = *cmd.elements[0];

	//Write the read values to the queued buffer addresses
	if (this->readPtr)
	{
		uint32_t tmp = 0;
		for (uint8_t k = 0; k < this->words; ++k)
		{
			tmp |= (el.getOutputAt16(2 * k) << (k * 16));
		}
		*this->readPtr = tmp;
		this->readPtr = nullptr;
	}
	this->queue.clear();
	return true;
}

bool EemMemoryAccess::writeEemRegister(EemRegister reg, uint32_t value)
{
	return this->doWrite(reg, value);
}

bool EemMemoryAccess::readEemRegister(EemRegister reg, uint32_t* buffer)
{
	return this->doRead(reg, buffer);
}
