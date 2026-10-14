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

        // Haptics output report 0x32 over Bluetooth. Layout from the SAxense research by egormanga
        // (https://github.com/egormanga/SAxense, https://apps.sdore.me/SAxense), cross-checked with
        // awalol/dualsense-bt-haptics:
        //   [0] 0x32, [1] seq << 4,
        //   packet 0x11 (header 0x91, length 7): FE 00 00 00 00 FF <counter>,
        //   packet 0x12 (header 0x92, length 64): 32 stereo frames of signed 8-bit PCM at 3000 Hz,
        //   zero padding, CRC32 (seeded with 0xA2) in the last 4 bytes. 142 bytes in total.
        constexpr size_t HapticsReportSize   = 142;
        constexpr size_t HapticsSamplesStart = 13;
        constexpr u32 HapticsSilentReportsBeforeStop = 10; // keep sending silence ~100 ms, then stop
        constexpr u32 HapticsChirpReports = 30;            // ~320 ms test sweep after connecting
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
        R_TRY(this->PushRumbleLedState());
        R_TRY(EmulatedSwitchController::Initialize());

        // Request controller firmware version info
        R_TRY(this->GetVersionInfo(&m_version_info));

        // Request motion calibration data from DualSense
        R_TRY(this->GetCalibrationData(&m_motion_calibration));

        m_feature_flags0 = 0x03;
        m_feature_flags1 = 0x54;

        auto config = mitm::GetGlobalConfig();
        m_hd_haptics   = config->misc.dualsense_hd_haptics;
        m_haptics_gain = config->misc.dualsense_haptics_gain / 100.0f;
        m_haptics_audio_init = m_hd_haptics && config->misc.dualsense_haptics_audio_init;
        if (m_hd_haptics) {
            m_haptics_chirp_reports = HapticsChirpReports;
        }
        m_lightbar_brightness = config->misc.dualsense_lightbar_brightness;
        m_rumble_intensity    = 0x08 - config->misc.dualsense_vibration_intensity; // User setting is inverse of how the controller sets intensity
        m_base_trigger_effect = TriggerEffectOff();
        if (config->misc.dualsense_adaptive_trigger_travel > 0) {
            m_feature_flags0 = 0x0F;
            m_base_trigger_effect = SimpleFeedback(static_cast<u8>(UINT8_MAX * config->misc.dualsense_adaptive_trigger_travel / 100.0f), 0xFF);
            m_trigger_effects_always_send = true;
            m_trigger_threshold = (config->misc.analog_trigger_activation_threshold / 100.0f) * (config->misc.dualsense_adaptive_trigger_travel / 100.0f);
        }
        m_trigger_effect_right = m_base_trigger_effect;
        m_trigger_effect_left  = m_base_trigger_effect;

        // In test mode trigger effects are only sent when they change, the controller keeps the last one
        m_trigger_test_mode = config->misc.dualsense_trigger_test_mode;
        if (m_trigger_test_mode) {
            m_feature_flags0 = 0x03;
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

    void DualsenseController::TickHaptics() {
        if (!m_hd_haptics) {
            return;
        }

        SwitchMotorData motors;
        {
            std::scoped_lock lk(m_haptics_lock);
            motors = m_haptics_motors;
        }

        // Startup sweep 100 -> 300 Hz on both coils, to check the stream works without a game
        if (m_haptics_chirp_reports > 0) {
            float progress = 1.0f - float(m_haptics_chirp_reports) / HapticsChirpReports;
            float freq = 100.0f + 200.0f * progress;
            motors.left_motor  = { .low_band_amp = 0.5f, .low_band_freq = freq, .high_band_amp = 0.0f, .high_band_freq = 320.0f };
            motors.right_motor = motors.left_motor;
            --m_haptics_chirp_reports;
        }

        bool silent = motors.left_motor.low_band_amp  <= 0.0f && motors.left_motor.high_band_amp  <= 0.0f &&
                      motors.right_motor.low_band_amp <= 0.0f && motors.right_motor.high_band_amp <= 0.0f;
        if (silent) {
            if (m_haptics_silent_reports >= HapticsSilentReportsBeforeStop) {
                return;
            }
            ++m_haptics_silent_reports;
        } else {
            m_haptics_silent_reports = 0;
        }

        u8 report[HapticsReportSize] = {};
        report[0]  = 0x32;
        report[1]  = m_haptics_seq << 4;
        report[2]  = 0x91;  // packet 0x11, sized
        report[3]  = 7;
        report[4]  = 0xFE;
        report[9]  = 0xFF;
        report[10] = m_haptics_counter++;
        report[11] = 0x92;  // packet 0x12, sized
        report[12] = haptics::FramesPerReport * 2;
        m_haptics_seq = (m_haptics_seq + 1) & 0x0F;

        // Two bands per coil, phase continuous across reports
        const SwitchVibrationValues *coil[2] = { &motors.left_motor, &motors.right_motor };
        for (u32 frame = 0; frame < haptics::FramesPerReport; ++frame) {
            for (int c = 0; c < 2; ++c) {
                float *phase_lo = &m_haptics_phase[c * 2];
                float *phase_hi = &m_haptics_phase[c * 2 + 1];
                float lo_freq = std::clamp(coil[c]->low_band_freq,  20.0f, 1400.0f);
                float hi_freq = std::clamp(coil[c]->high_band_freq, 20.0f, 1400.0f);

                float v = coil[c]->low_band_amp * std::sin(*phase_lo) + coil[c]->high_band_amp * std::sin(*phase_hi);
                *phase_lo = std::fmod(*phase_lo + TwoPi * lo_freq / haptics::SampleRate, TwoPi);
                *phase_hi = std::fmod(*phase_hi + TwoPi * hi_freq / haptics::SampleRate, TwoPi);

                float s = std::clamp(v * m_haptics_gain * 127.0f, -128.0f, 127.0f);
                report[HapticsSamplesStart + frame * 2 + c] = static_cast<u8>(static_cast<s8>(s));
            }
        }

        static const u32 crc_seed = [] {
            const u8 header[2] = { 0xA2, 0x32 };
            return crc32Calculate(header, sizeof(header));
        }();
        u32 crc = crc32CalculateWithSeed(crc_seed, &report[1], HapticsReportSize - 1 - 4);
        std::memcpy(&report[HapticsReportSize - 4], &crc, sizeof(crc));

        haptics::NoteReportResult(this->WriteDataReport(report, sizeof(report)));
    }

    void DualsenseController::ParseInputReport(const u8 *report_buffer, size_t size) {
        AMS_UNUSED(size);
        auto report = reinterpret_cast<const DualsenseReportData *>(report_buffer);

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
        bool send_trigger_effects = m_trigger_effects_always_send || m_trigger_effects_dirty.exchange(false);

        DualsenseReportData report = {};
        report.id = 0x31;
        report.output0x31.data[0]  = 0x02;
        report.output0x31.data[1]  = m_feature_flags0 | (send_trigger_effects ? (FlagRightTriggerEffect | FlagLeftTriggerEffect) : 0);
        report.output0x31.data[2]  = m_feature_flags1;
        report.output0x31.data[3]  = m_rumble_state.amp_motor_right;
        report.output0x31.data[4]  = m_rumble_state.amp_motor_left;
        std::memcpy(&report.output0x31.data[11], &m_trigger_effect_right, sizeof(DualsenseTriggerEffect));
        std::memcpy(&report.output0x31.data[22], &m_trigger_effect_left,  sizeof(DualsenseTriggerEffect));
        report.output0x31.data[37] = m_rumble_intensity;
        report.output0x31.data[39] = 0x02 | 0x01;
        report.output0x31.data[42] = 0x02;
        report.output0x31.data[43] = 0x02;
        report.output0x31.data[44] = m_led_flags;
        report.output0x31.data[45] = m_lightbar_colour.r;
        report.output0x31.data[46] = m_lightbar_colour.g;
        report.output0x31.data[47] = m_lightbar_colour.b;

        // Experimental: audio state as sent by awalol/dualsense-bt-haptics (taken from DSX) before streaming haptics.
        // Meaning of some fields is unknown, the values are copied as they are.
        if (m_haptics_audio_init) {
            report.output0x31.data[1] |= 0xF0;  // headphone, speaker, mic volume and audio control valid
            report.output0x31.data[2] |= 0xA3;  // mic mute LED, power save, audio control 2 valid (keeps lightbar/player LED/motor power bits)
            report.output0x31.data[5]  = 0x7F;  // headphone volume
            report.output0x31.data[6]  = 0x7F;  // speaker volume
            report.output0x31.data[7]  = 0xFF;  // mic volume
            report.output0x31.data[8]  = 0x09;  // audio control
            report.output0x31.data[38] = 0x0A;
            report.output0x31.data[39] |= 0x04;
        }
        report.output0x31.crc = crc32CalculateWithSeed(CrcSeed, report.output0x31.data, sizeof(report.output0x31.data));

        Result rc = this->WriteDataReport(&report, sizeof(report.output0x31) + sizeof(report.id));
        if (R_FAILED(rc) && send_trigger_effects) {
            // Effects didn't reach the controller, retry them with the next report
            m_trigger_effects_dirty = true;
        }

        R_RETURN(rc);
    }

}
