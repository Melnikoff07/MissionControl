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
#include <stratosphere.hpp>
#include "trigger_effects.hpp"

namespace ams::controller::haptics {

    // DualSense voice coil haptics over Bluetooth stream at 3000 Hz in blocks of 32 stereo frames,
    // one output report every 10.67 ms. This starts the thread that ticks all connected controllers.
    constexpr u32 SampleRate      = 3000;
    constexpr u32 FramesPerReport = 32;

    Result Initialize();

    // Called by controllers after writing a haptics report
    void NoteReportResult(Result rc);
    void NotePrimeResult(u32 failed_mask);
    void NoteChirp();
    void NoteFallback();

    void GetStatus(HapticsStatus *out);

}
