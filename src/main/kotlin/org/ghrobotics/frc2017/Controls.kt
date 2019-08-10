/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import org.ghrobotics.frc2017.commands.OpenLoopFlywheelCommand
import org.ghrobotics.frc2017.subsystems.Superstructure
import org.ghrobotics.lib.mathematics.units.derived.radian
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.*

@Suppress("MagicNumber")
object Controls {
    val driverController = xboxController(0) {
        axisButton(5, 0.1) { change(OpenLoopFlywheelCommand(source.map { -it })) }
        button(kBumperRight).change(Superstructure.shootFuel(200.radian.velocity))

        button(kBack).changeOn { Robot.activateEmergency() }
        button(kStart).changeOn { Robot.recoverFromEmergency() }
    }

    fun update() {
        driverController.update()
    }
}
