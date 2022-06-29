/*
 * EmulationManager430.h
 *
 * Emulation manager holds and manages the different modules
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

#include "IEmulationManager.h"
#include "../TriggerManager/TriggerManager430.h"

namespace TI { namespace DLL430 {

class TriggerManager430;
class StateStorage430;

class EmulationManager430 : public IEmulationManager
{
public:
	static EmulationManagerPtr create(uint8_t emulationLevel);

	virtual TriggerConditionManagerPtr getTriggerConditionManager() const;
	virtual BreakpointManagerPtr getBreakpointManager() const;
	virtual ClockControlPtr getClockControl() const;
	virtual CycleCounterPtr getCycleCounter() const;
	virtual SequencerPtr getSequencer() const;
	virtual TracePtr getTrace() const;
	virtual VariableWatchPtr getVariableWatch() const;
	virtual SoftwareBreakpointsPtr getSoftwareBreakpoints() const;

	virtual bool hasTriggerConditionManager() const;
	virtual bool hasBreakpointManager() const;
	virtual bool hasClockControl() const;
	virtual bool hasCycleCounter() const;
	virtual bool hasSequencer() const;
	virtual bool hasTrace() const;
	virtual bool hasVariableWatch() const;
	virtual bool hasSoftwareBreakpoints() const;

	virtual void writeConfiguration() const;
	virtual void rewriteConfiguration() const;
	virtual void onEvent(MessageDataPtr messageData) const;

	virtual void reset() = 0;

	virtual void fillEMInfo(DEVICE_T *deviceData);
	//These functions will be removed after reimplementation of clock control and cycle counter
	virtual void writeRegister(uint32_t reg, uint32_t value) const;
	virtual uint32_t readRegister(uint32_t reg) const;

protected:
	EmulationManager430();

	BreakpointManagerPtr mBreakpointManager;
	ClockControlPtr mClockControl;
	CycleCounterPtr mCycleCounter;
	SequencerPtr mSequencer;
	TriggerConditionManagerPtr mTriggerConditionManager;
	TracePtr mTrace;
	VariableWatchPtr mVariableWatch;
	SoftwareBreakpointsPtr mSoftwareBreakpoints;

	TriggerManager430Ptr mTriggerManager;
};

}}
