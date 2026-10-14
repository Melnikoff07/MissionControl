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
#include "emulated_switch_controller.hpp"
#include <atomic>

namespace ams::controller {

    struct DualsenseButtonData {
        u8 dpad     : 4;
        u8 square   : 1;
        u8 cross    : 1;
        u8 circle   : 1;
        u8 triangle : 1;

        u8 L1       : 1;
        u8 R1       : 1;
        u8 L2       : 1;
        u8 R2       : 1;
        u8 share    : 1;
        u8 options  : 1;
        u8 L3       : 1;
        u8 R3       : 1;
        
        u8 ps       : 1;
        u8 touchpad : 1;
        u8 mute     : 1;
        u8          : 0;

        u8 unk;
    } PACKED;

    using DualsenseTriggerEffect = TriggerEffect;

    struct DualsenseRumbleData {
        u8 amp_motor_left;
        u8 amp_motor_right;
    } PACKED;

    struct DualsenseImuCalibrationData {
        struct {
            s16 pitch_bias;
            s16 yaw_bias;
            s16 roll_bias;
            s16 pitch_max;
            s16 pitch_min;
            s16 yaw_max;
            s16 yaw_min;
            s16 roll_max;
            s16 roll_min;
            s16 speed_max;
            s16 speed_min;
        } gyro;
        
        struct {
            s16 x_max;
            s16 x_min;
            s16 y_max;
            s16 y_min;
            s16 z_max;
            s16 z_min;
        } acc;
    } PACKED;

    struct DualsenseTouchpadPoint {
        u8 contact;
        u8 x_lo;
        u8 x_hi : 4;
        u8 y_lo : 4;
        u8 y_hi;
    } PACKED;

    struct DualsenseVersionInfo {
        char data[64];
    } PACKED;

    struct DualsenseFeatureReport0x05 {
        DualsenseImuCalibrationData calibration;
        u32 crc;
    } PACKED;

    struct DualsenseFeatureReport0x20 {
        DualsenseVersionInfo version_info;
    } PACKED;

    struct DualsenseOutputReport0x31 {
        struct {
            u8 data[73];
        };
        u32 crc;
    } PACKED;

    struct DualsenseInputReport0x01 {
        AnalogStick<u8> left_stick;
        AnalogStick<u8> right_stick;
        DualsenseButtonData buttons;
        u8 left_trigger;
        u8 right_trigger;
    } PACKED;

    struct DualsenseInputReport0x31 {
        u8 _unk0;
        AnalogStick<u8> left_stick;
        AnalogStick<u8> right_stick;
        u8 left_trigger;
        u8 right_trigger;
        u8 counter;
        DualsenseButtonData buttons;
        u32 _unk1;
        s16 vel_x;
        s16 vel_y;
        s16 vel_z;
        s16 acc_x;
        s16 acc_y;
        s16 acc_z;
        s32 timestamp;
        u8 temperature;
        DualsenseTouchpadPoint touch_points[2];
        u8 _unk3[12];

        u8 battery_level   : 4;
        u8 powered         : 1;
        u8 charge_complete : 1;
        u8                 : 0;
    } PACKED;

    struct DualsenseReportData {
        u8 id;
        union {
            DualsenseFeatureReport0x05 feature0x05;
            DualsenseFeatureReport0x20 feature0x20;
            DualsenseOutputReport0x31 output0x31;
            DualsenseInputReport0x01 input0x01;
            DualsenseInputReport0x31 input0x31;
        };
    } PACKED;

    class DualsenseController final : public EmulatedSwitchController {

        public:
            static constexpr const HardwareID hardware_ids[] = {
                {0x054c, 0x0ce6},   // Sony Dualsense Controller
                {0x054c, 0x0df2}    // Sony Dualsense Edge Controller
            };

            DualsenseController(bluetooth::Address address, HardwareID id) : EmulatedSwitchController(address, id)
            , m_led_flags(0)
            , m_lightbar_colour({0, 0, 0})
            , m_lightbar_brightness(0)
            , m_rumble_state({0, 0})
            , m_trigger_effect_right({})
            , m_trigger_effect_left({})
            , m_base_trigger_effect({})
            , m_trigger_effects_always_send(false)
            , m_trigger_effects_dirty(false)
            , m_trigger_retry_queued(false)
            , m_trigger_retry_tick(0)
            , m_trigger_test_mode(false)
            , m_trigger_test_preset(0)
            , m_trigger_test_prev_dpad(0x08)
            , m_trigger_test_saved_colour({0, 0, 0})
            , m_analog_left_trigger(0)
            , m_analog_right_trigger(0)
            , m_lightbar_override_active(false)
            , m_player_colour({0, 0, 0})
            , m_hd_haptics(false)
            , m_packet_output(false)
            , m_output_seq(0)
            , m_haptics_audio_pending(false)
            , m_haptics_ready(false)
            , m_haptics_gain(1.0f)
            , m_haptics_motors({})
            , m_haptics_phase{}
            , m_haptics_counter(0)
            , m_haptics_silent_reports(0)
            , m_haptics_test_ticks(0)
            , m_voices{}
            , m_noise_seed(0x12345678) { }

            virtual Result Initialize() override;
            virtual Result SetVibration(const SwitchMotorData *motor_data) override;
            virtual Result CancelVibration() override;
            virtual Result SetPlayerLed(SwitchPlayerNumber player_number) override;
            Result SetLightbarColour(RGBColour colour);
            Result SetTriggerEffects(const DualsenseTriggerEffect &right, const DualsenseTriggerEffect &left);
            virtual Result SetTriggerEffectsOverride(const TriggerEffects *effects) override;
            virtual bool GetAnalogTriggers(AnalogTriggerState *out) const override;
            virtual Result SetLightbarOverride(const RGBColour *colour) override;
            virtual void TickHaptics() override;
            virtual void PlayHaptics(const HapticVoices *voices) override;

            virtual void ParseInputReport(const u8 *report_buffer, size_t size) override;

        private:
            void MapInputReport0x01(const DualsenseReportData *src);
            void MapInputReport0x31(const DualsenseReportData *src);

            Result GetVersionInfo(DualsenseVersionInfo *version_info);
            Result GetCalibrationData(DualsenseImuCalibrationData *calibration);
            void PrimeAudioFeatures();
            Result PushRumbleLedState();
            void HandleHapticsWriteFailure(Result rc);
            Result WriteHapticsReport(const u8 *samples);
            bool MixVoices(float *left, float *right);
            void HandleTriggerTestCombo(const DualsenseButtonData &buttons);

            u8 m_feature_flags0;
            u8 m_feature_flags1;
            u8 m_led_flags;
            RGBColour m_lightbar_colour;
            u8 m_lightbar_brightness;
            u8 m_rumble_intensity;

            DualsenseRumbleData m_rumble_state;

            DualsenseTriggerEffect m_trigger_effect_right;
            DualsenseTriggerEffect m_trigger_effect_left;
            DualsenseTriggerEffect m_base_trigger_effect;   // Effect from config (trigger travel limit), restored when test mode is off
            bool m_trigger_effects_always_send;             // Original behaviour: resend trigger effects with every output report
            std::atomic<bool> m_trigger_effects_dirty;      // Trigger effects changed and must be sent with the next output report
            std::atomic<bool> m_trigger_retry_queued;       // A resend of unsent trigger effects is queued
            u64 m_trigger_retry_tick;                       // Last resend, to rate limit retries

            bool m_trigger_test_mode;
            u8 m_trigger_test_preset;
            u8 m_trigger_test_prev_dpad;
            RGBColour m_trigger_test_saved_colour;

            std::atomic<u8> m_analog_left_trigger;
            std::atomic<u8> m_analog_right_trigger;

            bool m_lightbar_override_active;
            RGBColour m_player_colour;  // Colour from SetPlayerLed, restored when the override is cleared

            // HD haptics: the game's HD rumble synthesised as audio for the voice coils
            std::atomic<bool> m_hd_haptics;     // cleared when the controller falls back to the simplified rumble
            bool m_packet_output;               // 0x31 reports in the sequenced packet format, needed for haptics (set at connect)
            os::SdkMutex m_output_lock;         // one output report at a time, so sequence numbers go out in order
            u8 m_output_seq;                    // sequence number of all packet format reports, under m_output_lock
            std::atomic<bool> m_haptics_audio_pending;  // audio control / unmute still has to be sent with a 0x31 report
            std::atomic<bool> m_haptics_ready;  // connect sequence done, the haptics thread may stream
            float m_haptics_gain;
            os::SdkMutex m_haptics_lock;
            SwitchMotorData m_haptics_motors;   // latest decoded rumble, written by SetVibration
            float m_haptics_phase[4];           // oscillator phases: left low/high, right low/high
            u8 m_haptics_counter;
            u32 m_haptics_silent_reports;
            u32 m_haptics_test_ticks;           // startup test sweep after connecting

            // Pattern voices from mc command 12, under m_haptics_lock
            struct ActiveVoice {
                bool active;
                bool noise;
                bool loop;
                float amp_left;
                float amp_right;
                float freq_start;
                float freq_end;
                u32 delay;          // in samples
                u32 attack;
                u32 hold;
                u32 decay;
                u32 position;
                float phase;
                float noise_state;
            };
            ActiveVoice m_voices[HapticVoiceSlots];
            u32 m_noise_seed;

            DualsenseVersionInfo m_version_info;
            DualsenseImuCalibrationData m_motion_calibration;
    };

}
