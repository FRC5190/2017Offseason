/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot

/* ktlint-disable no-wildcard-imports */
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kBumperRight
import org.ghrobotics.lib.wrappers.hid.xboxController
import org.ghrobotics.robot.subsytems.drive.ShooterSubsystem

object Controls {
    val mainXbox = xboxController(0) {
        button(kBumperRight).change(ShooterSubsystem.shootCommand)
    }
}