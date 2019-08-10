/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2017.commands

import org.ghrobotics.frc2017.subsystems.Flywheel
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity
import org.ghrobotics.lib.utils.Source

class OpenLoopFlywheelCommand(private val percentSource: Source<Double>) : FalconCommand(Flywheel) {
    override fun execute() {
        Flywheel.setOpenLoop(percentSource())
    }

    override fun end(interrupted: Boolean) {
        Flywheel.setNeutral()
    }
}

class ClosedLoopFlywheelCommand(private val speed: SIUnit<AngularVelocity>) :
    FalconCommand(Flywheel) {
    override fun initialize() {
        Flywheel.setVelocity(speed.value)
    }

    override fun end(interrupted: Boolean) {
        Flywheel.setNeutral()
    }
}
