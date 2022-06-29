/*
 * ARMRandomMemoryAccess.h
 *
 * Memory class for accessing RAM on ARM Cortex devices.
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

#include "MemoryAreaBase.h"

namespace TI
{
	namespace DLL430
	{
		class IDeviceHandle;
		class ArmRandomMemoryAccess : public MemoryAreaBase
		{
		public:
			ArmRandomMemoryAccess (
				MemoryArea::Name name,
				IDeviceHandle* devHandle,
				uint32_t start,
				uint32_t size,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				IMemoryManager* mm,
				uint8_t psa);
			virtual ~ArmRandomMemoryAccess ();

			virtual bool doWrite(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			virtual bool doWrite(uint32_t address, uint32_t value) OVERRIDE;
			virtual bool doRead(uint32_t address, uint8_t* buffer, size_t count) OVERRIDE;

			virtual bool isReadOnly() const OVERRIDE { return false; }

			virtual bool erase() OVERRIDE{ return true; }
			virtual bool erase(uint32_t start, uint32_t end, bool forceUnlock = false) OVERRIDE{ return true; }
			virtual bool verify(uint32_t address, const uint8_t* buffer, size_t count) OVERRIDE;
			virtual bool wakeup() OVERRIDE{ return false; };

		protected:
			bool postSync(const HalExecCommand&) OVERRIDE;
			bool uploadFunclet();
			void restoreRam();

			IMemoryManager* mm;
			std::vector<uint8_t> ramBackup;
		};

	}
}
