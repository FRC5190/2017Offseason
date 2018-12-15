/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot

import org.ghrobotics.lib.wrappers.FalconRobotBase
import org.ghrobotics.robot.subsytems.drive.DriveSubsystem
import org.ghrobotics.robot.subsytems.drive.ShooterSubsystem

class Robot : FalconRobotBase() {

    // Initialize all systems.
    override suspend fun initialize() {
        +Controls.mainXbox

        +DriveSubsystem
        +ShooterSubsystem
    }
}