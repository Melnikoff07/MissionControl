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
        u32 input_phase[4]; // 0x31 input reports per startup phase: connect, pause, test buzz, after the test
        u32 voices;         // pattern voices received over mc command 12
    };

    // Current input of the first controller that reports it (mc command 14): Switch button bits as in SwitchButton
    // (Y = bit 0 ... L = bit 22, ZL = bit 23) and raw analog trigger positions
    struct ControllerInput {
        u32 buttons;
        u8 left_trigger;
        u8 right_trigger;
        u8 valid;
        u8 reserved;
    };

    // One voice of a haptic pattern for the DualSense voice coils (mc command 12). A voice is a sine or low-passed noise
    // whose frequency sweeps from freq_start to freq_end, shaped by an attack / hold / decay envelope after a delay.
    // Patterns are built from several voices; they are mixed with the game's own rumble.
    struct HapticVoice {
        u8  slot;           // 0-11: a voice replaces the one playing in its slot; amplitude 0 on both sides stops the slot
        u8  flags;          // HapticVoiceFlag_*
        u8  amp_left;       // 0-255
        u8  amp_right;
        u16 freq_start;     // Hz: sine frequency, or the noise low-pass cutoff
        u16 freq_end;
        u16 delay_ms;
        u16 attack_ms;
        u16 hold_ms;        // a looping voice holds until it is replaced or stopped
        u16 decay_ms;
    };

    enum HapticVoiceFlag : u8 {
        HapticVoiceFlag_Noise = 0x01,
        HapticVoiceFlag_Loop  = 0x02,
    };

    constexpr u32 HapticVoiceSlots = 12;

    struct HapticVoices {
        HapticVoice voices[4];
        u32 count;
    };

    static_assert(sizeof(HapticVoice) == 16);
    static_assert(sizeof(HapticVoices) == 68);
    static_assert(sizeof(TriggerEffect) == 11);
    static_assert(sizeof(TriggerEffects) == 22);

}
