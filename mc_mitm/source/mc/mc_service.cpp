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
#include "mc_service.hpp"
#include "../mcmitm_version.hpp"
#include "../bluetooth_mitm/btdrv_ext.h"
#include "../bluetooth_mitm/bluetooth/bluetooth_core.hpp"
#include "../controllers/controller_management.hpp"
#include "../controllers/dualsense_haptics.hpp"

namespace ams::mc {

    namespace {

        constexpr ncm::ProgramId MissionControlTitleId = ncm::ProgramId(0x010000000000bd00);

        constinit BtdrvExtCustomEventInfo g_event_info;
        constinit BtdrvExtCustomEventType g_current_event_type;

        // Session that last set trigger effects. Only touched from the mc service thread.
        constinit const MissionControlService *g_trigger_effects_owner = nullptr;
        constinit const MissionControlService *g_lightbar_owner = nullptr;
        constinit const MissionControlService *g_haptics_owner = nullptr;

        Result WaitCustomEventInfo() {
            // Wait on custom command reply event
            bluetooth::core::GetCustomDataEvent()->Wait();

            // Fetch custom command reply data
            R_TRY(bluetooth::core::GetEventInfo(MissionControlTitleId, reinterpret_cast<bluetooth::EventType *>(&g_current_event_type), &g_event_info, sizeof(g_event_info)));

            // Return status code
            R_RETURN(g_event_info.hci_command_response.status);
        }

    }

    MissionControlService::~MissionControlService() {
        // Client went away (closed its session or crashed): restore default trigger effects and lightbar
        if (g_trigger_effects_owner == this) {
            g_trigger_effects_owner = nullptr;
            controller::SetTriggerEffectsOverride(nullptr);
        }
        if (g_lightbar_owner == this) {
            g_lightbar_owner = nullptr;
            controller::SetLightbarOverride(nullptr);
        }
        if (g_haptics_owner == this) {
            g_haptics_owner = nullptr;
            controller::PlayHaptics(nullptr);
        }
    }

    Result MissionControlService::GetVersion(sf::Out<u32> version) {
        version.SetValue(mc_version);
        R_SUCCEED();
    }

    Result MissionControlService::GetBuildVersionString(sf::Out<mc::VersionString> version) {
        std::strncpy(version.GetPointer()->version, mc_build_name, sizeof(mc::VersionString));
        R_SUCCEED();
    }

    Result MissionControlService::GetBuildDateString(sf::Out<mc::DateString> date) {
        std::strncpy(date.GetPointer()->date, mc_build_date, sizeof(mc::DateString));
        R_SUCCEED();
    }

    Result MissionControlService::GetHciHandle(bluetooth::Address address, sf::Out<u16> out_handle) {
        R_TRY(btdrvextGetHciHandle(address));

        // Wait for response to custom event
        R_TRY(WaitCustomEventInfo());

        // Set the output handle
        out_handle.SetValue(g_event_info.get_hci_handle.handle);

        R_SUCCEED();
    }

    Result MissionControlService::SendHciCommand(u16 opcode, const sf::InPointerBuffer &buffer, const sf::OutPointerBuffer &out_buffer) {
        R_TRY(btdrvextSendHciCommand(opcode, buffer.GetPointer(), buffer.GetSize()));

        // Wait for response to custom event
        R_TRY(WaitCustomEventInfo());

        // Copy response to output buffer
        std::memcpy(out_buffer.GetPointer(), g_event_info.hci_command_response.data, g_event_info.hci_command_response.size);

        R_SUCCEED();
    }

    Result MissionControlService::DmSetConfig(const ams::mc::BsaSetConfig &set_config) {
        R_RETURN(btdrvextDmSetConfig(&set_config.config));
    }

    Result MissionControlService::SetTriggerEffects(const ams::controller::TriggerEffects &effects) {
        g_trigger_effects_owner = this;
        controller::SetTriggerEffectsOverride(&effects);
        R_SUCCEED();
    }

    Result MissionControlService::GetAnalogTriggers(sf::Out<ams::controller::AnalogTriggerState> out) {
        ams::controller::AnalogTriggerState state = {};
        controller::GetAnalogTriggers(&state);
        out.SetValue(state);
        R_SUCCEED();
    }

    Result MissionControlService::SetLightbarColour(const ams::controller::RGBColour &colour) {
        g_lightbar_owner = this;
        controller::SetLightbarOverride(&colour);
        R_SUCCEED();
    }

    Result MissionControlService::ClearLightbarColour() {
        g_lightbar_owner = nullptr;
        controller::SetLightbarOverride(nullptr);
        R_SUCCEED();
    }

    Result MissionControlService::PlayHaptics(const ams::controller::HapticVoices &voices) {
        g_haptics_owner = this;
        controller::PlayHaptics(&voices);
        R_SUCCEED();
    }

    Result MissionControlService::StopHaptics() {
        g_haptics_owner = nullptr;
        controller::PlayHaptics(nullptr);
        R_SUCCEED();
    }

    Result MissionControlService::GetControllerInput(sf::Out<ams::controller::ControllerInput> out) {
        ams::controller::ControllerInput input = {};
        controller::GetControllerInput(&input);
        out.SetValue(input);
        R_SUCCEED();
    }

    Result MissionControlService::GetHapticsStatus(sf::Out<ams::controller::HapticsStatus> out) {
        ams::controller::HapticsStatus status;
        controller::haptics::GetStatus(&status);
        out.SetValue(status);
        R_SUCCEED();
    }

    Result MissionControlService::ClearTriggerEffects() {
        g_trigger_effects_owner = nullptr;
        controller::SetTriggerEffectsOverride(nullptr);
        R_SUCCEED();
    }

}
