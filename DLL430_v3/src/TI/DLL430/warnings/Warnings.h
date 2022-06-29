/*
 * Warnings.h
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
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
#include <boost/thread/mutex.hpp>
#include "MSP430.h"

namespace TI
{
	namespace DLL430
	{

		namespace WarningCode
		{
			const char WARNING_IP_PROTECTION[] = "IP protection is enabled on the device. Not all flash memory locations may be readable or writable";
			const char WARNING_FLASH_VCC[] = "Target device supply voltage is too low for Flash erase/programming";
			const char WARNING_LDO_SUPPLY[] = "On-board LDO is now used to supply the target with power";
			const char WARNING_DCDC_SUPPLY[] = "On-board DCDC logic is now used to supply the target with power";
		};

		class Warning
		{
		public:
			Warning();
			bool RegisterMessageCallback(MessageCallbackFn callback);
			void message(MESSAGE_LEVEL_T level, const char *str);
		private:
			Warning(const Warning&) {}
			Warning& operator =(const Warning&) { return *this; }
			MessageCallbackFn _callback;
			boost::mutex _mutex;
		};

		class WarningFactory
		{
		public:
			static Warning* instance();
		private:
			WarningFactory() {}
			WarningFactory(const WarningFactory&) {}
			WarningFactory& operator =(const WarningFactory&) { return *this; }
		};
	}
}
