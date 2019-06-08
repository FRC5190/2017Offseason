/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.GlobalScope
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.wrappers.FalconRobotBase
import org.ghrobotics.frc2017.subsytems.drive.DriveSubsystem
import org.ghrobotics.frc2017.subsytems.shooter.ShooterSubsystem
import kotlin.properties.Delegates.observable

class Robot : FalconRobotBase() {

    var lowPowerMode by observable(false) { _, _, lowPowerMode ->
        DriveSubsystem.lowPowerMode = lowPowerMode
        ShooterSubsystem.lowPowerMode = lowPowerMode
    }

    // Initialize all systems.
    override suspend fun initialize() {
        +Controls.mainXbox

        +DriveSubsystem
        +ShooterSubsystem

        val lowPowerModeChooser = SendableChooser<Boolean>()
        lowPowerModeChooser.addDefault("High Performance", false)
        lowPowerModeChooser.addObject("Battery Saver", true)

        val shooterHeightChooser = SendableChooser<Boolean>()
        shooterHeightChooser.addDefault("Shoot Low", true)
        shooterHeightChooser.addObject("Shoot High", false)

        SmartDashboard.putData("Power Mode", lowPowerModeChooser)
        SmartDashboard.putData("Shooter Height", lowPowerModeChooser)
        SmartDashboard.setPersistent("Power Mode")
        SmartDashboard.setPersistent("Shooter Height")

        lowPowerMode = lowPowerModeChooser.selected

        GlobalScope.launchFrequency(1) {
            ShooterSubsystem.lowShoot = shooterHeightChooser.selected

            val newMode = lowPowerModeChooser.selected

            if (lowPowerMode != newMode) lowPowerMode = newMode
        }
    }
}