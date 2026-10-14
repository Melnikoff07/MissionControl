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
#pragma once
#include <switch.h>

namespace ams::controller {

    // One trigger's adaptive effect block as used by the DualSense output report (mode + 10 parameters)
    struct TriggerEffect {
        u8 mode;
        u8 params[10];
    } PACKED;

    struct TriggerEffects {
        TriggerEffect right;
        TriggerEffect left;
    } PACKED;

    // Raw analog trigger positions of a controller with analog triggers (0 = released, 255 = fully pressed)
    struct AnalogTriggerState {
        u8 left;
        u8 right;
        u8 valid;
        u8 reserved;
    };

    // Diagnostics for HD haptics streaming (mc command 11)
    struct HapticsStatus {
        u32 enabled;        // dualsense_hd_haptics set in the config
        u32 thread_running;
        u32 ticks;          // haptics thread iterations
        u32 reports_sent;   // 0x32 reports written successfully
        u32 reports_failed;
        u32 last_error;     // Result of the last failed write
        u32 prime_failed;   // feature reads 0x09/0x22 at connect that failed (bit 0 = 0x09, bit 1 = 0x22), last controller
        u32 chirps;         // startup test buzzes started (one per connected controller)
        u32 fallbacks;      // controllers that gave up on HD haptics after write failures
        u32 input_reports;  // 0x31 input reports from DualSense controllers in haptics mode
        u32 input_mic;      // of those, reports flagged as microphone audio (skipped)
        u32 input_other;    // input reports with other ids
        u8  last_input[8];  // first bytes of the last 0x31 input report
        u32 input_phase[6]; // 0x31 input reports per startup phase: connect, delay, variant 1-3, after the test
        u32 rearms;         // feature 0x05 re-reads after input reports stopped
        u32 rearm_failed;
    };

    static_assert(sizeof(TriggerEffect) == 11);
    static_assert(sizeof(TriggerEffects) == 22);

}
