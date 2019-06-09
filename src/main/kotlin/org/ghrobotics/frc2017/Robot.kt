/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.ghrobotics.frc2017.subsystems.Agitator
import org.ghrobotics.frc2017.subsystems.Drivetrain
import org.ghrobotics.frc2017.subsystems.Flywheel
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    init {
        +Drivetrain
        +Agitator
        +Flywheel
    }

    override fun robotPeriodic() {
        Shuffleboard.update()
    }
}