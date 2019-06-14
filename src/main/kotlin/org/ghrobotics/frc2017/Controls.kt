/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import org.ghrobotics.frc2017.commands.OpenLoopFlywheelCommand
import org.ghrobotics.frc2017.subsystems.Superstructure
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kBumperRight
import org.ghrobotics.lib.wrappers.hid.xboxController

object Controls {
    val driverController = xboxController(0) {
        axisButton(5, 0.1) { change(OpenLoopFlywheelCommand(source.map { -it })) }
        button(kBumperRight).change(Superstructure.shootFuel(200.0))
    }

    fun update() {
        driverController.update()
    }
}