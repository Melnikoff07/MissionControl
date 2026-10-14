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

    static_assert(sizeof(TriggerEffect) == 11);
    static_assert(sizeof(TriggerEffects) == 22);

}
