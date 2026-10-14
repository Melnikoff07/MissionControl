/*
 * Copyright (c) 2020-2026 ndeadly
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "dualsense_haptics.hpp"
#include "controller_management.hpp"
#include "../mcmitm_config.hpp"

namespace ams::controller::haptics {

    namespace {

        constexpr s32 ThreadPriority = 0;
        constexpr size_t ThreadStackSize = 0x2000;
        alignas(os::ThreadStackAlignment) constinit u8 g_thread_stack[ThreadStackSize];
        constinit os::ThreadType g_thread;

        // 32 frames at 3000 Hz
        constexpr u64 ReportPeriodNs = 1'000'000'000ULL * FramesPerReport / SampleRate;

        void HapticsThreadFunc(void *) {
            const u64 period = armNsToTicks(ReportPeriodNs);
            u64 next = armGetSystemTick();

            for (;;) {
                for (auto &controller : GetControllers()) {
                    controller->TickHaptics();
                }

                // Fixed rate without drift; if we fell far behind, resync instead of bursting
                next += period;
                u64 now = armGetSystemTick();
                if (now > next + period * 4) {
                    next = now;
                } else if (next > now) {
                    svcSleepThread(armTicksToNs(next - now));
                }
            }
        }

    }

    Result Initialize() {
        if (!mitm::GetGlobalConfig()->misc.dualsense_hd_haptics) {
            R_SUCCEED();
        }

        R_TRY(os::CreateThread(&g_thread,
            HapticsThreadFunc,
            nullptr,
            g_thread_stack,
            ThreadStackSize,
            ThreadPriority
        ));

        os::SetThreadNamePointer(&g_thread, "mc::DualsenseHaptics");
        os::StartThread(&g_thread);

        R_SUCCEED();
    }

}
