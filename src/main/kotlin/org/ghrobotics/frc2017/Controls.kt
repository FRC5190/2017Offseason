/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kBumperRight
import org.ghrobotics.lib.wrappers.hid.xboxController
import org.ghrobotics.frc2017.subsytems.shooter.ShooterSubsystem

object Controls {
    val mainXbox = xboxController(0) {
        button(kBumperRight).change(ShooterSubsystem.shootCommand)
    }
}