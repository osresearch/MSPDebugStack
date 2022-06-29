/*
 * AddressRangeCondition430.h
 *
 * Implementation of address range trigger conditions for 430
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

#pragma once


#include "TriggerCondition430.h"
#include "IInstructionRangeCondition.h"
#include "IAddressRangeCondition.h"

namespace TI { namespace DLL430 {

class Trigger430;

class AddressRangeCondition430 : public TriggerCondition430, public IInstructionRangeCondition, public IAddressRangeCondition
{
public:
	AddressRangeCondition430(TriggerManager430Ptr triggerManager, uint32_t minAddress, uint32_t maxAddress,
							 uint32_t minMask = 0xffffffff, uint32_t maxMask = 0xffffffff, AccessType = AT_FETCH, bool outside = false);

	virtual void setAccessType(AccessType accessType);
	virtual void setInside();
	virtual void setOutside();
	virtual void setMinAddress(uint32_t address, uint32_t mask = 0xffffffff);
	virtual void setMaxAddress(uint32_t address, uint32_t mask = 0xffffffff);

	virtual void addReaction(TriggerReaction reaction) { TriggerCondition430::addReaction(reaction); }
	virtual void removeReaction(TriggerReaction reaction) { TriggerCondition430::removeReaction(reaction); }

	virtual void combine(TriggerConditionPtr condition) { TriggerCondition430::combine(condition); }
	virtual uint32_t getId() const { return TriggerCondition430::getId(); }

private:
	Trigger430 *minTrigger_;
	Trigger430 *maxTrigger_;
};

}}
