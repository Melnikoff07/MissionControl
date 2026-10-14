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
#include "dualsense_controller.hpp"
#include "controller_management.hpp"
#include "../mcmitm_config.hpp"
#include "../async/async.hpp"
#include "dualsense_haptics.hpp"
#include <cmath>
#include <stratosphere.hpp>

namespace ams::controller {

    namespace {

        constexpr u8 TriggerMax = UINT8_MAX;

        constexpr u16 TouchpadWidth = 1920;
        constexpr u16 TouchpadHeight = 1080;

        constexpr u8 PlayerLedFlags[] = {
            // Mimic the Switch's player LEDs
            0x01,
            0x03,
            0x0B,
            0x1B,
            0x11,
            0x09,
            0x19,
            0x0A
        };

        constexpr u8 NewPlayerLedFlags[] = {
            0x04,
            0x02,
            0x05,
            0x03,
            0x07,
            0x07,
            0x07,
            0x07
        };

        constexpr RGBColour PlayerLedBaseColours[] = {
            {0x00, 0x00, 0x00}, // off
            // Same colours used by PS4
            {0x00, 0x00, 0x04}, // blue
            {0x04, 0x00, 0x00}, // red
            {0x00, 0x04, 0x00}, // green
            {0x02, 0x00, 0x02}, // pink
            // New colours for controllers 5-8
            {0x00, 0x02, 0x02}, // cyan
            {0x03, 0x01, 0x00}, // orange
            {0x02, 0x02, 0x00}, // yellow
            {0x01, 0x00, 0x03}  // purple
        };

        constexpr SwitchBatteryLevel BatteryLookup[] = {
            SwitchBatteryLevel::Critical,
            SwitchBatteryLevel::Low,
            SwitchBatteryLevel::Low,
            SwitchBatteryLevel::Low,
            SwitchBatteryLevel::Medium,
            SwitchBatteryLevel::Medium,
            SwitchBatteryLevel::Medium,
            SwitchBatteryLevel::Full,
            SwitchBatteryLevel::Full,
            SwitchBatteryLevel::Full,
            SwitchBatteryLevel::Full,
        };

        constexpr u8 Step = 4;
        constexpr u8 LedBrightnessMultipliers[] = { 0, 1, 1 * Step, 2 * Step, 3 * Step, 4 * Step, 5 * Step, 6 * Step, 7 * Step, 8 * Step };

        constexpr u32 CrcSeed = 0x8C36CCAE; // CRC32 of {0xa2, 0x31} bytes at beginning of output report

        // HD haptics over Bluetooth: the DualSense plays signed 8-bit stereo PCM at 3000 Hz sent in sequenced packets,
        // packet 0x11 (header 0x91) with stream control fields and packet 0x12 (header 0x92) with 32 stereo frames.
        // Layout from the SAxense research by egormanga (https://github.com/egormanga/SAxense) and awalol's DS5Dongle
        // (https://github.com/awalol/DS5Dongle). Their hosts use the 142-byte report 0x32, but on the Switch a lightbar
        // probe showed 0x32 never reaches the controller intact. So the packets go into the 78-byte 0x31 report, with
        // packet 0x11 cut down to the fields DS5Dongle found sufficient (buffer length and packet counter).
        // The controller keeps sending input reports while this streams.
        constexpr size_t HapticsSampleBytes = 64;          // 32 stereo frames
        constexpr u8 HapticsBufferLength = 48;             // DS5Dongle default
        constexpr u32 HapticsSilentReportsBeforeStop = 10; // keep sending silence ~100 ms, then stop
        constexpr u32 HapticsChirpReports = 141;           // ~1.5 s test sweep

        // Packet 0x11 stream control: flags, buffer length, packet counter. Only bit 6 (counter) is documented as
        // required. Without bit 1 the controller stops sending input reports (tested on the console), so keep it.
        constexpr u8 HapticsControlFlags = 0x62;

        // Startup test, in haptics thread ticks (~94 per second): a pause with the audio path armed, then the sweep
        constexpr u32 HapticsTestDelayTicks = 188;  // ~2 s: the console sets the player colour after connecting
        constexpr u32 HapticsTestTicks = HapticsTestDelayTicks + HapticsChirpReports + HapticsSilentReportsBeforeStop;

        // Output report valid_flag0 bits that switch the controller to rumble emulation. While they are set the voice
        // coils play the motor values and ignore the haptics stream, so they stay clear in HD haptics mode.
        constexpr u8 FlagsRumbleEmulation = 0x01 | 0x02;
        constexpr float TwoPi = 6.28318530718f;

        // Output report valid_flag0 bits that tell the controller to apply the trigger effect blocks
        constexpr u8 FlagRightTriggerEffect = 0x04;
        constexpr u8 FlagLeftTriggerEffect  = 0x08;

        // Adaptive trigger effect modes. Byte layouts follow community reverse engineering of the
        // DualSense (Nielk1's TriggerEffectGenerator, used by DSX). Zoned modes split the trigger
        // travel into 10 zones (0-9) and take strengths/amplitudes in the range 1-8.
        constexpr u8 TriggerEffectModeSimpleFeedback  = 0x01;
        constexpr u8 TriggerEffectModeSimpleWeapon    = 0x02;
        constexpr u8 TriggerEffectModeOff             = 0x05;
        constexpr u8 TriggerEffectModeSimpleVibration = 0x06;
        constexpr u8 TriggerEffectModeFeedback        = 0x21;
        constexpr u8 TriggerEffectModeBow             = 0x22;
        constexpr u8 TriggerEffectModeWeapon          = 0x25;
        constexpr u8 TriggerEffectModeVibration       = 0x26;

        constexpr void PutU16(u8 *dst, u16 value) {
            dst[0] = value & 0xFF;
            dst[1] = value >> 8;
        }

        constexpr void PutU32(u8 *dst, u32 value) {
            for (int i = 0; i < 4; ++i) {
                dst[i] = (value >> (8 * i)) & 0xFF;
            }
        }

        constexpr DualsenseTriggerEffect TriggerEffectOff() {
            return { TriggerEffectModeOff, {} };
        }

        // Constant resistance from start position (0-255) to the end of travel
        constexpr DualsenseTriggerEffect SimpleFeedback(u8 start, u8 force) {
            return { TriggerEffectModeSimpleFeedback, { start, force } };
        }

        // Resistance between start and end positions (0-255)
        constexpr DualsenseTriggerEffect SimpleWeapon(u8 start, u8 end, u8 force) {
            return { TriggerEffectModeSimpleWeapon, { start, end, force } };
        }

        // Vibration from start position (0-255) to the end of travel
        constexpr DualsenseTriggerEffect SimpleVibration(u8 frequency, u8 amplitude, u8 start) {
            return { TriggerEffectModeSimpleVibration, { frequency, amplitude, start } };
        }

        // Resistance per zone, strengths[i] in 0-8 where 0 means no resistance in that zone
        constexpr DualsenseTriggerEffect Feedback(const u8 (&strengths)[10]) {
            u16 active_zones = 0;
            u32 zone_strengths = 0;
            for (int i = 0; i < 10; ++i) {
                if (strengths[i] > 0) {
                    active_zones |= 1 << i;
                    zone_strengths |= static_cast<u32>((strengths[i] - 1) & 0x07) << (3 * i);
                }
            }

            DualsenseTriggerEffect effect = { TriggerEffectModeFeedback, {} };
            PutU16(&effect.params[0], active_zones);
            PutU32(&effect.params[2], zone_strengths);
            return effect;
        }

        // Resistance from start zone (0-8) to end zone (start+1 - 8), then snaps back with snap_force (1-8)
        constexpr DualsenseTriggerEffect Bow(u8 start, u8 end, u8 strength, u8 snap_force) {
            DualsenseTriggerEffect effect = { TriggerEffectModeBow, {} };
            PutU16(&effect.params[0], (1 << start) | (1 << end));
            PutU16(&effect.params[2], ((strength - 1) & 0x07) | (((snap_force - 1) & 0x07) << 3));
            return effect;
        }

        // Resistance from start zone (2-7) that gives way at end zone (start+1 - 8), like a gun trigger breaking
        constexpr DualsenseTriggerEffect Weapon(u8 start, u8 end, u8 strength) {
            DualsenseTriggerEffect effect = { TriggerEffectModeWeapon, {} };
            PutU16(&effect.params[0], (1 << start) | (1 << end));
            effect.params[2] = (strength - 1) & 0x07;
            return effect;
        }

        // Vibration from start zone (0-9) to the end of travel, amplitude 1-8, frequency in Hz
        constexpr DualsenseTriggerEffect Vibration(u8 start, u8 amplitude, u8 frequency) {
            u16 active_zones = 0;
            u32 zone_amplitudes = 0;
            for (int i = start; i < 10; ++i) {
                active_zones |= 1 << i;
                zone_amplitudes |= static_cast<u32>((amplitude - 1) & 0x07) << (3 * i);
            }

            DualsenseTriggerEffect effect = { TriggerEffectModeVibration, {} };
            PutU16(&effect.params[0], active_zones);
            PutU32(&effect.params[2], zone_amplitudes);
            effect.params[8] = frequency;
            return effect;
        }

        struct TriggerTestPreset {
            RGBColour colour;
            DualsenseTriggerEffect right;
            DualsenseTriggerEffect left;
        };

        // Presets for trigger test mode (dualsense_trigger_test_mode). Preset 0 is "off", these are 1-7.
        constexpr TriggerTestPreset TriggerTestPresets[] = {
            // 1 red: constant resistance from ~25% travel (the mode MissionControl already uses for trigger travel)
            { {0x40, 0x00, 0x00}, SimpleFeedback(0x40, 0xFF), SimpleFeedback(0x40, 0xFF) },
            // 2 orange: R2 resistance rising along the travel (bow draw), L2 even medium resistance (shield)
            { {0x40, 0x14, 0x00}, Feedback({0, 1, 2, 3, 4, 5, 6, 7, 8, 8}), Feedback({0, 0, 4, 4, 4, 4, 4, 4, 4, 4}) },
            // 3 yellow: resistance that gives way with a click (parry)
            { {0x30, 0x30, 0x00}, Weapon(2, 6, 8), Weapon(3, 5, 8) },
            // 4 green: bow, resistance that snaps back at the end
            { {0x00, 0x40, 0x00}, Bow(1, 7, 6, 8), Bow(1, 4, 3, 8) },
            // 5 cyan: zoned vibration, R2 strong and fast, L2 weak and slow (weapon about to break)
            { {0x00, 0x30, 0x30}, Vibration(0, 8, 40), Vibration(0, 4, 15) },
            // 6 blue: simple vibration
            { {0x00, 0x00, 0x40}, SimpleVibration(30, 0xFF, 0x00), SimpleVibration(10, 0xFF, 0x00) },
            // 7 purple: simple resistance section in the middle of the travel
            { {0x30, 0x00, 0x30}, SimpleWeapon(0x50, 0xA0, 0xFF), SimpleWeapon(0x50, 0xA0, 0xFF) },
        };

        constexpr u8 TriggerTestPresetCount = std::size(TriggerTestPresets);

    }

    Result DualsenseController::Initialize() {
        auto config = mitm::GetGlobalConfig();

        // HD haptics: the controller must never see the rumble emulation flags, not even in the first report,
        // and its audio path is armed the way DS5Dongle (a working Bluetooth haptics host) does it
        const bool hd_haptics = config->misc.dualsense_hd_haptics;
        m_hd_haptics    = hd_haptics;
        m_packet_output = hd_haptics;
        m_haptics_audio_pending = hd_haptics;
        m_haptics_gain  = config->misc.dualsense_haptics_gain / 100.0f;
        const u8 rumble_flags = hd_haptics ? 0x00 : FlagsRumbleEmulation;

        m_feature_flags0 = rumble_flags;
        m_feature_flags1 = 0x54;
        m_rumble_intensity = 0x08 - config->misc.dualsense_vibration_intensity; // User setting is inverse of how the controller sets intensity

        R_TRY(this->PushRumbleLedState());
        R_TRY(EmulatedSwitchController::Initialize());

        // Same feature read order as DS5Dongle: 0x09, 0x20, 0x22, 0x05. On the DualSense a feature read can switch
        // modes (0x05 enables the full input report), and 0x09/0x22 are the reads MissionControl never did.
        u32 prime_failed = 0;
        bluetooth::HidReport feature;
        if (m_hd_haptics && R_FAILED(this->GetReport(0x09, BtdrvBluetoothHhReportType_Feature, &feature))) {
            prime_failed |= 1;
        }

        // Request controller firmware version info
        R_TRY(this->GetVersionInfo(&m_version_info));

        if (m_hd_haptics && R_FAILED(this->GetReport(0x22, BtdrvBluetoothHhReportType_Feature, &feature))) {
            prime_failed |= 2;
        }

        // Request motion calibration data from DualSense
        R_TRY(this->GetCalibrationData(&m_motion_calibration));

        m_lightbar_brightness = config->misc.dualsense_lightbar_brightness;
        m_base_trigger_effect = TriggerEffectOff();
        if (config->misc.dualsense_adaptive_trigger_travel > 0) {
            m_feature_flags0 = rumble_flags | FlagRightTriggerEffect | FlagLeftTriggerEffect;
            m_base_trigger_effect = SimpleFeedback(static_cast<u8>(UINT8_MAX * config->misc.dualsense_adaptive_trigger_travel / 100.0f), 0xFF);
            m_trigger_effects_always_send = true;
            m_trigger_threshold = (config->misc.analog_trigger_activation_threshold / 100.0f) * (config->misc.dualsense_adaptive_trigger_travel / 100.0f);
        }
        m_trigger_effect_right = m_base_trigger_effect;
        m_trigger_effect_left  = m_base_trigger_effect;

        // In test mode trigger effects are only sent when they change, the controller keeps the last one
        m_trigger_test_mode = config->misc.dualsense_trigger_test_mode;
        if (m_trigger_test_mode) {
            m_feature_flags0 = rumble_flags;
            m_trigger_effects_always_send = false;
            m_trigger_effects_dirty = true;
        }

        // Pick up effects and lightbar colour that were requested over IPC before this controller connected
        TriggerEffects effects;
        if (GetTriggerEffectsOverride(&effects)) {
            R_TRY(this->SetTriggerEffectsOverride(&effects));
        }
        RGBColour colour;
        if (GetLightbarOverride(&colour)) {
            R_TRY(this->SetLightbarOverride(&colour));
        }

        if (m_hd_haptics) {
            // Arm the audio path again now that the feature reads are done, the order DS5Dongle uses
            m_haptics_audio_pending = true;
            R_DISCARD(this->PushRumbleLedState());

            haptics::NotePrimeResult(prime_failed);
            haptics::NoteChirp();
            m_haptics_test_ticks = 0;
            m_haptics_ready = true;
        }

        R_SUCCEED();
    }

    Result DualsenseController::SetVibration(const SwitchMotorData *motor_data) {
        if (m_hd_haptics) {
            // Played by TickHaptics on the voice coils instead of the simplified rumble
            std::scoped_lock lk(m_haptics_lock);
            m_haptics_motors = *motor_data;
            R_SUCCEED();
        }

        m_rumble_state.amp_motor_left  = static_cast<u8>(255 * std::max(motor_data->left_motor.low_band_amp, motor_data->right_motor.low_band_amp));
        m_rumble_state.amp_motor_right = static_cast<u8>(255 * std::max(motor_data->left_motor.high_band_amp, motor_data->right_motor.high_band_amp));
        return this->PushRumbleLedState();
    }

    Result DualsenseController::CancelVibration() {
        if (m_hd_haptics) {
            std::scoped_lock lk(m_haptics_lock);
            m_haptics_motors = {};
            R_SUCCEED();
        }

        m_rumble_state.amp_motor_left = 0;
        m_rumble_state.amp_motor_right = 0;
        return this->PushRumbleLedState();
    }

    Result DualsenseController::SetPlayerLed(SwitchPlayerNumber player_number) {
        u8 player_index = static_cast<u8>(player_number);
        u16 fw_version = *reinterpret_cast<u16 *>(&m_version_info.data[43]);

        auto config = mitm::GetGlobalConfig();
        if (!config->misc.dualsense_enable_player_leds || (player_number == SwitchPlayerNumber::Invalid)) {
            m_led_flags = 0x00;
        } else if (fw_version < 0x0282) {
            m_led_flags = PlayerLedFlags[player_index];
        } else {
            m_led_flags = NewPlayerLedFlags[player_index];
        }

        // Disable LED fade-in
        m_led_flags |= 0x20;

        RGBColour colour = PlayerLedBaseColours[player_index];
        u8 multiplier = LedBrightnessMultipliers[m_lightbar_brightness];
        colour.r *= multiplier;
        colour.g *= multiplier;
        colour.b *= multiplier;
        m_player_colour = colour;

        // Keep a colour requested over IPC, only update the player LEDs
        if (m_lightbar_override_active) {
            R_RETURN(this->PushRumbleLedState());
        }

        R_RETURN(this->SetLightbarColour(colour));
    }

    Result DualsenseController::SetLightbarColour(RGBColour colour) {
        m_lightbar_colour = colour;
        R_RETURN(this->PushRumbleLedState());
    }

    Result DualsenseController::SetTriggerEffects(const DualsenseTriggerEffect &right, const DualsenseTriggerEffect &left) {
        m_trigger_effect_right = right;
        m_trigger_effect_left  = left;
        m_trigger_effects_dirty = true;
        R_RETURN(this->PushRumbleLedState());
    }

    Result DualsenseController::SetTriggerEffectsOverride(const TriggerEffects *effects) {
        // An IPC request takes over from a running test preset
        if (m_trigger_test_preset != 0) {
            m_trigger_test_preset = 0;
            m_lightbar_colour = m_trigger_test_saved_colour;
        }

        if (effects == nullptr) {
            R_RETURN(this->SetTriggerEffects(m_base_trigger_effect, m_base_trigger_effect));
        }

        R_RETURN(this->SetTriggerEffects(effects->right, effects->left));
    }

    Result DualsenseController::SetLightbarOverride(const RGBColour *colour) {
        m_lightbar_override_active = colour != nullptr;
        R_RETURN(this->SetLightbarColour(colour ? *colour : m_player_colour));
    }

    bool DualsenseController::GetAnalogTriggers(AnalogTriggerState *out) const {
        out->left  = m_analog_left_trigger;
        out->right = m_analog_right_trigger;
        out->valid = 1;
        return true;
    }

    bool DualsenseController::GetControllerInput(ControllerInput *out) const {
        out->buttons       = m_buttons.Value();
        out->left_trigger  = m_analog_left_trigger;
        out->right_trigger = m_analog_right_trigger;
        out->valid         = 1;
        return true;
    }

    void DualsenseController::TickHaptics() {
        if (!m_hd_haptics || !m_haptics_ready) {
            return;
        }

        SwitchMotorData motors;
        {
            std::scoped_lock lk(m_haptics_lock);
            motors = m_haptics_motors;
        }

        // Pattern voices sent by a client (e.g. the adaptive_triggers module): one frame buffer per side
        float voice_left[haptics::FramesPerReport];
        float voice_right[haptics::FramesPerReport];
        bool voices_playing = this->MixVoices(voice_left, voice_right);

        if (m_haptics_test_ticks < HapticsTestTicks) {
            // Startup test: a 100 -> 300 Hz sweep on both coils after a pause
            u32 t = m_haptics_test_ticks++;
            if (t < HapticsTestDelayTicks) {
                return;
            }
            u32 sweep = t - HapticsTestDelayTicks;
            motors = {};
            if (sweep < HapticsChirpReports) {
                float freq = 100.0f + 200.0f * float(sweep) / HapticsChirpReports;
                motors.left_motor  = { .low_band_amp = 1.0f, .low_band_freq = freq, .high_band_amp = 0.0f, .high_band_freq = 320.0f };
                motors.right_motor = motors.left_motor;
            }
            if (t + 1 == HapticsTestTicks) {
                m_haptics_silent_reports = HapticsSilentReportsBeforeStop;
            }
        } else {
            bool silent = !voices_playing &&
                          motors.left_motor.low_band_amp  <= 0.0f && motors.left_motor.high_band_amp  <= 0.0f &&
                          motors.right_motor.low_band_amp <= 0.0f && motors.right_motor.high_band_amp <= 0.0f;
            if (silent) {
                if (m_haptics_silent_reports >= HapticsSilentReportsBeforeStop) {
                    return;
                }
                ++m_haptics_silent_reports;
            } else {
                m_haptics_silent_reports = 0;
            }
        }

        // Two bands per coil, phase continuous across reports. Switch games use small amplitudes (the Joy-Con actuators
        // are much more sensitive than the DualSense voice coils), so amplitudes go through a square root: weak
        // effects get a lot stronger, full strength stays full. A soft clip instead of a hard one keeps strong
        // effects from turning harsh when the gain is high.
        u8 samples[HapticsSampleBytes];
        const SwitchVibrationValues *coil[2] = { &motors.left_motor, &motors.right_motor };
        for (u32 frame = 0; frame < haptics::FramesPerReport; ++frame) {
            for (int c = 0; c < 2; ++c) {
                float *phase_lo = &m_haptics_phase[c * 2];
                float *phase_hi = &m_haptics_phase[c * 2 + 1];
                float lo_freq = std::clamp(coil[c]->low_band_freq,  20.0f, 1400.0f);
                float hi_freq = std::clamp(coil[c]->high_band_freq, 20.0f, 1400.0f);

                float lo_amp = std::sqrt(std::clamp(coil[c]->low_band_amp,  0.0f, 1.0f));
                float hi_amp = std::sqrt(std::clamp(coil[c]->high_band_amp, 0.0f, 1.0f));
                float v = lo_amp * std::sin(*phase_lo) + hi_amp * std::sin(*phase_hi) + (c == 0 ? voice_left : voice_right)[frame];
                *phase_lo = std::fmod(*phase_lo + TwoPi * lo_freq / haptics::SampleRate, TwoPi);
                *phase_hi = std::fmod(*phase_hi + TwoPi * hi_freq / haptics::SampleRate, TwoPi);

                float s = std::clamp(std::tanh(v * m_haptics_gain) * 127.0f, -128.0f, 127.0f);
                samples[frame * 2 + c] = static_cast<u8>(static_cast<s8>(s));
            }
        }

        Result rc = this->WriteHapticsReport(samples);
        haptics::NoteReportResult(rc);
        if (R_FAILED(rc)) {
            this->HandleHapticsWriteFailure(rc);
        }
    }

    void DualsenseController::PlayHaptics(const HapticVoices *voices) {
        if (!m_hd_haptics) {
            return;
        }

        std::scoped_lock lk(m_haptics_lock);
        if (voices == nullptr) {
            for (auto &v : m_voices) {
                v.active = false;
            }
            return;
        }

        constexpr u32 SamplesPerMs = haptics::SampleRate / 1000;
        for (u32 i = 0; i < std::min<u32>(voices->count, 4); ++i) {
            const HapticVoice &in = voices->voices[i];
            ActiveVoice &v = m_voices[in.slot % HapticVoiceSlots];
            v = {};
            if (in.amp_left == 0 && in.amp_right == 0) {
                continue;
            }
            v.active     = true;
            v.noise      = in.flags & HapticVoiceFlag_Noise;
            v.loop       = in.flags & HapticVoiceFlag_Loop;
            v.amp_left   = in.amp_left  / 255.0f;
            v.amp_right  = in.amp_right / 255.0f;
            v.freq_start = std::clamp<float>(in.freq_start, 10.0f, 1400.0f);
            v.freq_end   = std::clamp<float>(in.freq_end,   10.0f, 1400.0f);
            v.delay      = in.delay_ms  * SamplesPerMs;
            v.attack     = in.attack_ms * SamplesPerMs;
            v.hold       = in.hold_ms   * SamplesPerMs;
            v.decay      = in.decay_ms  * SamplesPerMs;
        }
    }

    bool DualsenseController::MixVoices(float *left, float *right) {
        std::memset(left,  0, sizeof(float) * haptics::FramesPerReport);
        std::memset(right, 0, sizeof(float) * haptics::FramesPerReport);

        bool playing = false;
        std::scoped_lock lk(m_haptics_lock);
        for (auto &v : m_voices) {
            if (!v.active) {
                continue;
            }
            playing = true;

            for (u32 frame = 0; frame < haptics::FramesPerReport; ++frame) {
                u32 t = v.position++;
                if (t < v.delay) {
                    continue;
                }
                t -= v.delay;

                // Envelope: linear attack, hold, decay with a quadratic tail so hits fade like a thump
                float env;
                u32 sustain_end = v.attack + v.hold;
                if (t < v.attack) {
                    env = float(t) / v.attack;
                } else if (v.loop || t < sustain_end) {
                    env = 1.0f;
                } else if (t < sustain_end + v.decay) {
                    float x = 1.0f - float(t - sustain_end) / v.decay;
                    env = x * x;
                } else {
                    v.active = false;
                    break;
                }

                // Frequency sweep over the whole voice (a looping voice reaches freq_end when its hold ends)
                u32 sweep_len = v.loop ? sustain_end : sustain_end + v.decay;
                float progress = sweep_len > 0 ? std::min(1.0f, float(t) / sweep_len) : 1.0f;
                float freq = v.freq_start + (v.freq_end - v.freq_start) * progress;

                float value;
                if (v.noise) {
                    // White noise through a one-pole low-pass, scaled back up to a similar loudness as a sine
                    m_noise_seed = m_noise_seed * 1664525u + 1013904223u;
                    float white = float(static_cast<s32>(m_noise_seed)) / 2147483648.0f;
                    float a = 1.0f - std::exp(-TwoPi * freq / haptics::SampleRate);
                    v.noise_state += a * (white - v.noise_state);
                    value = std::clamp(v.noise_state * 1.2f / std::sqrt(a), -1.0f, 1.0f);
                } else {
                    value = std::sin(v.phase);
                    v.phase = std::fmod(v.phase + TwoPi * freq / haptics::SampleRate, TwoPi);
                }

                left[frame]  += env * v.amp_left  * value;
                right[frame] += env * v.amp_right * value;
            }
        }
        return playing;
    }

    Result DualsenseController::WriteHapticsReport(const u8 *samples) {
        DualsenseReportData report = {};
        report.id = 0x31;
        u8 *data = report.output0x31.data;

        size_t i = 1;
        data[i++] = 0x91;   // packet 0x11, sized
        data[i++] = 3;
        data[i++] = HapticsControlFlags;
        data[i++] = HapticsBufferLength;
        data[i++] = m_haptics_counter++;
        data[i++] = 0x92;   // packet 0x12, sized
        data[i++] = HapticsSampleBytes;
        std::memcpy(&data[i], samples, HapticsSampleBytes);
        i += HapticsSampleBytes;
        AMS_ABORT_UNLESS(i <= sizeof(report.output0x31.data));

        std::scoped_lock lk(m_output_lock);
        data[0] = m_output_seq << 4;
        m_output_seq = (m_output_seq + 1) & 0x0F;
        report.output0x31.crc = crc32CalculateWithSeed(CrcSeed, report.output0x31.data, sizeof(report.output0x31.data));
        R_RETURN(this->WriteDataReport(&report, sizeof(report.output0x31) + sizeof(report.id)));
    }

    void DualsenseController::HandleHapticsWriteFailure(Result rc) {
        AMS_UNUSED(rc);

        // Twice the console froze after failed haptics writes (e.g. the controller switched off mid stream), so
        // give up on HD haptics at the first failure until the controller reconnects. Rumble emulation also parks
        // the voice coils; it goes out with the next regular report, no extra write to a link that is failing.
        m_hd_haptics = false;
        m_feature_flags0 |= FlagsRumbleEmulation;
        haptics::NoteFallback();
    }

    void DualsenseController::ParseInputReport(const u8 *report_buffer, size_t size) {
        auto report = reinterpret_cast<const DualsenseReportData *>(report_buffer);

        if (m_packet_output) {
            // With the audio path armed, a 0x31 report flagged with bit 1 of its second byte carries microphone audio
            // instead of the controller state (as handled by DS5Dongle); it must not be read as buttons and sticks
            bool mic = report->id == 0x31 && size > 1 && (report_buffer[1] & 0x02);
            u32 phase = 0;
            if (m_haptics_ready) {
                u32 t = m_haptics_test_ticks;
                phase = t < HapticsTestDelayTicks ? 1 : t < HapticsTestTicks ? 2 : 3;
            }
            haptics::NoteInputReport(report_buffer, size, mic, phase);
            if (mic) {
                return;
            }
        }

        switch(report->id) {
            case 0x01:
                this->MapInputReport0x01(report); break;
            case 0x31:
                this->MapInputReport0x31(report); break;
            default:
                break;
        }
    }

    void DualsenseController::MapInputReport0x01(const DualsenseReportData *src) {
        auto dpad = DirectionalPad(static_cast<DirectionalPadType<0, 8>>(src->input0x01.buttons.dpad));

        m_left_stick.SetValuesFrom(
            src->input0x01.left_stick.GetX(),
            src->input0x01.left_stick.GetYInverted()
        );

        m_right_stick.SetValuesFrom(
            src->input0x01.right_stick.GetX(),
            src->input0x01.right_stick.GetYInverted()
        );

        SwitchButtons button_state = m_buttons;
        button_state.Assign(SwitchButton::Down,    dpad.IsDown());
        button_state.Assign(SwitchButton::Up,      dpad.IsUp());
        button_state.Assign(SwitchButton::Right,   dpad.IsRight());
        button_state.Assign(SwitchButton::Left,    dpad.IsLeft());
        button_state.Assign(SwitchButton::A,       src->input0x01.buttons.circle);
        button_state.Assign(SwitchButton::B,       src->input0x01.buttons.cross);
        button_state.Assign(SwitchButton::X,       src->input0x01.buttons.triangle);
        button_state.Assign(SwitchButton::Y,       src->input0x01.buttons.square);
        button_state.Assign(SwitchButton::R,       src->input0x01.buttons.R1);
        button_state.Assign(SwitchButton::L,       src->input0x01.buttons.L1);
        button_state.Assign(SwitchButton::ZR,      src->input0x01.right_trigger > (m_trigger_threshold * TriggerMax));
        button_state.Assign(SwitchButton::ZL,      src->input0x01.left_trigger  > (m_trigger_threshold * TriggerMax));
        m_analog_left_trigger  = src->input0x01.left_trigger;
        m_analog_right_trigger = src->input0x01.right_trigger;
        button_state.Assign(SwitchButton::Minus,   src->input0x01.buttons.share);
        button_state.Assign(SwitchButton::Plus,    src->input0x01.buttons.options);
        button_state.Assign(SwitchButton::StickL,  src->input0x01.buttons.L3);
        button_state.Assign(SwitchButton::StickR,  src->input0x01.buttons.R3);
        button_state.Assign(SwitchButton::Capture, src->input0x01.buttons.mute);
        button_state.Assign(SwitchButton::Home,    src->input0x01.buttons.ps);
        m_buttons = button_state;
    }

    void DualsenseController::MapInputReport0x31(const DualsenseReportData *src) {
        m_power_info.SetPowered(src->input0x31.powered);
        m_power_info.SetCharging(src->input0x31.powered && !src->input0x31.charge_complete);
        m_power_info.SetBatteryLevel(BatteryLookup[std::clamp<u8>(src->input0x31.battery_level, 0, sizeof(BatteryLookup) - 1)]);

        auto dpad = DirectionalPad(static_cast<DirectionalPadType<0, 8>>(src->input0x31.buttons.dpad));

        m_left_stick.SetValuesFrom(
            src->input0x31.left_stick.GetX(),
            src->input0x31.left_stick.GetYInverted()
        );

        m_right_stick.SetValuesFrom(
            src->input0x31.right_stick.GetX(),
            src->input0x31.right_stick.GetYInverted()
        );

        SwitchButtons button_state = m_buttons;
        button_state.Assign(SwitchButton::Down,    dpad.IsDown());
        button_state.Assign(SwitchButton::Up,      dpad.IsUp());
        button_state.Assign(SwitchButton::Right,   dpad.IsRight());
        button_state.Assign(SwitchButton::Left,    dpad.IsLeft());
        button_state.Assign(SwitchButton::A,       src->input0x31.buttons.circle);
        button_state.Assign(SwitchButton::B,       src->input0x31.buttons.cross);
        button_state.Assign(SwitchButton::X,       src->input0x31.buttons.triangle);
        button_state.Assign(SwitchButton::Y,       src->input0x31.buttons.square);
        button_state.Assign(SwitchButton::R,       src->input0x31.buttons.R1);
        button_state.Assign(SwitchButton::L,       src->input0x31.buttons.L1);
        button_state.Assign(SwitchButton::ZR,      src->input0x31.right_trigger > (m_trigger_threshold * TriggerMax));
        button_state.Assign(SwitchButton::ZL,      src->input0x31.left_trigger  > (m_trigger_threshold * TriggerMax));
        m_analog_left_trigger  = src->input0x31.left_trigger;
        m_analog_right_trigger = src->input0x31.right_trigger;
        button_state.Assign(SwitchButton::Minus,   src->input0x31.buttons.share);
        button_state.Assign(SwitchButton::Plus,    src->input0x31.buttons.options);
        button_state.Assign(SwitchButton::StickL,  src->input0x31.buttons.L3);
        button_state.Assign(SwitchButton::StickR,  src->input0x31.buttons.R3);
        button_state.Assign(SwitchButton::Capture, src->input0x31.buttons.mute);
        button_state.Assign(SwitchButton::Home,    src->input0x31.buttons.ps);
        if (src->input0x31.buttons.touchpad) {
            for (int i = 0; i < 2; ++i) {
                const DualsenseTouchpadPoint *point = &src->input0x31.touch_points[i];

                bool active = point->contact & BIT(7) ? false : true;;
                if (active) {
                    u16 x = (point->x_hi << 8) | point->x_lo;

                    if (x < (0.15 * TouchpadWidth)) {
                        button_state.Set(SwitchButton::Minus);
                    } else if (x > (0.85 * TouchpadWidth)) {
                        button_state.Set(SwitchButton::Plus);
                    } else {
                        button_state.Set(SwitchButton::Capture);
                    }
                }
            }
        }
        m_buttons = button_state;

        if (m_trigger_test_mode) {
            this->HandleTriggerTestCombo(src->input0x31.buttons);
        }

        // A trigger effect that failed to reach the controller would otherwise stay stuck until the next change
        // (e.g. a short vibration pulse that never gets switched off): resend it, at most 20 times per second
        if (!m_trigger_effects_always_send && m_trigger_effects_dirty) {
            u64 now = armGetSystemTick();
            if (now - m_trigger_retry_tick >= armNsToTicks(50'000'000) && !m_trigger_retry_queued.exchange(true)) {
                m_trigger_retry_tick = now;
                async::QueueWork(new async::AsyncFunction([this]() -> ams::Result {
                    m_trigger_retry_queued = false;
                    R_RETURN(this->PushRumbleLedState());
                }));
            }
        }

        utils::Vec3d<float> accel = {
            .x = -src->input0x31.acc_z / float(m_motion_calibration.acc.z_max),
            .y = -src->input0x31.acc_x / float(m_motion_calibration.acc.x_max),
            .z =  src->input0x31.acc_y / float(m_motion_calibration.acc.y_max)
        };

        utils::Vec3d<float> gyro = {
            .x = -(src->input0x31.vel_z - m_motion_calibration.gyro.roll_bias)  / (float(m_motion_calibration.gyro.roll_max  - m_motion_calibration.gyro.roll_bias)  / m_motion_calibration.gyro.speed_max),
            .y = -(src->input0x31.vel_x - m_motion_calibration.gyro.pitch_bias) / (float(m_motion_calibration.gyro.pitch_max - m_motion_calibration.gyro.pitch_bias) / m_motion_calibration.gyro.speed_max),
            .z =  (src->input0x31.vel_y - m_motion_calibration.gyro.yaw_bias)   / (float(m_motion_calibration.gyro.yaw_max   - m_motion_calibration.gyro.yaw_bias)   / m_motion_calibration.gyro.speed_max)
        };

        m_sixaxis_processor.Update(accel, gyro);
    }

    void DualsenseController::HandleTriggerTestCombo(const DualsenseButtonData &buttons) {
        // Act only on the moment a D-pad direction is pressed while L3+R3 are held
        u8 dpad = buttons.dpad;
        bool dpad_changed = dpad != m_trigger_test_prev_dpad;
        m_trigger_test_prev_dpad = dpad;
        if (!dpad_changed || !(buttons.L3 && buttons.R3)) {
            return;
        }

        u8 preset = m_trigger_test_preset;
        switch (dpad) {
            case 2: // Right: next preset
                preset = (preset + 1) % (TriggerTestPresetCount + 1); break;
            case 6: // Left: previous preset
                preset = (preset + TriggerTestPresetCount) % (TriggerTestPresetCount + 1); break;
            case 4: // Down: effects off
                preset = 0; break;
            default:
                return;
        }

        if (preset == m_trigger_test_preset) {
            return;
        }

        if (m_trigger_test_preset == 0) {
            m_trigger_test_saved_colour = m_lightbar_colour;
        }
        m_trigger_test_preset = preset;

        // Don't write to the controller from the input report handler, hand it to the async worker
        async::QueueWork(new async::AsyncFunction([this, preset]() -> ams::Result {
            if (preset == 0) {
                m_lightbar_colour = m_trigger_test_saved_colour;
                R_RETURN(this->SetTriggerEffects(m_base_trigger_effect, m_base_trigger_effect));
            }

            const TriggerTestPreset &p = TriggerTestPresets[preset - 1];
            m_lightbar_colour = p.colour;
            R_RETURN(this->SetTriggerEffects(p.right, p.left));
        }));
    }

    Result DualsenseController::GetVersionInfo(DualsenseVersionInfo *version_info) {
        bluetooth::HidReport output;
        R_TRY(this->GetReport(0x20, BtdrvBluetoothHhReportType_Feature, &output));

        auto response = reinterpret_cast<DualsenseReportData *>(&output.data);
        std::memcpy(version_info, &response->feature0x20.version_info, sizeof(DualsenseVersionInfo));

        R_SUCCEED();
    }

    Result DualsenseController::GetCalibrationData(DualsenseImuCalibrationData *calibration) {
        bluetooth::HidReport output;
        R_TRY(this->GetReport(0x05, BtdrvBluetoothHhReportType_Feature, &output));

        auto response = reinterpret_cast<DualsenseReportData *>(&output.data);
        std::memcpy(calibration, &response->feature0x05.calibration, sizeof(DualsenseImuCalibrationData));

        R_SUCCEED();
    }

    Result DualsenseController::PushRumbleLedState() {
        std::scoped_lock lk(m_output_lock);

        bool send_trigger_effects = m_trigger_effects_always_send || m_trigger_effects_dirty.exchange(false);
        bool send_audio = m_haptics_audio_pending.exchange(false);

        DualsenseReportData report = {};
        report.id = 0x31;

        // The set state block (47 bytes, same layout as the USB output report 0x02) follows a short header
        u8 *state;
        if (m_packet_output) {
            // Sequenced packet format of the Linux driver and DS5Dongle: seq << 4, then packet 0x10 (set state).
            // The sequence number is shared with the haptics reports.
            report.output0x31.data[0] = m_output_seq << 4;
            report.output0x31.data[1] = 0x10;
            m_output_seq = (m_output_seq + 1) & 0x0F;
            state = &report.output0x31.data[2];
        } else {
            report.output0x31.data[0] = 0x02;
            state = &report.output0x31.data[1];
        }

        state[0]  = m_feature_flags0 | (send_trigger_effects ? (FlagRightTriggerEffect | FlagLeftTriggerEffect) : 0);
        state[1]  = m_feature_flags1;
        state[2]  = m_rumble_state.amp_motor_right;
        state[3]  = m_rumble_state.amp_motor_left;
        std::memcpy(&state[10], &m_trigger_effect_right, sizeof(DualsenseTriggerEffect));
        std::memcpy(&state[21], &m_trigger_effect_left,  sizeof(DualsenseTriggerEffect));
        state[36] = m_hd_haptics ? 0x00 : m_rumble_intensity;   // motor power reduction, haptics strength comes from the gain
        state[38] = 0x02 | 0x01;
        state[41] = 0x02;
        state[42] = 0x02;
        state[43] = m_led_flags;
        state[44] = m_lightbar_colour.r;
        state[45] = m_lightbar_colour.g;
        state[46] = m_lightbar_colour.b;

        if (send_audio) {
            // Arm the audio path once per connection like DS5Dongle: audio control (microphone disabled, so the
            // controller has no reason to send mic audio in place of input reports) and the mute/power save block
            // with everything on, haptics included
            state[0] |= 0x80;   // audio control valid
            state[1] |= 0x02;   // mute control valid
            state[7]  = 0x03;   // audio control: mic select 3 (disabled in DS5Dongle's config)
            state[9]  = 0x00;   // no power save, nothing muted
        }
        report.output0x31.crc = crc32CalculateWithSeed(CrcSeed, report.output0x31.data, sizeof(report.output0x31.data));

        Result rc = this->WriteDataReport(&report, sizeof(report.output0x31) + sizeof(report.id));
        if (R_FAILED(rc)) {
            // Didn't reach the controller, retry with the next report
            if (send_trigger_effects) {
                m_trigger_effects_dirty = true;
            }
            if (send_audio) {
                m_haptics_audio_pending = true;
            }
        }

        R_RETURN(rc);
    }

}
