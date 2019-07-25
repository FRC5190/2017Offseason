/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.ghrobotics.frc2017.subsystems.Drivetrain
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    init {
        Drivetrain
    }

    val velocity = Shuffleboard.getTab("5190").add("Flywheel Vel", 0.0)
    val voltage = Shuffleboard.getTab("5190").add("Flywheel Voltage", 0.0)

    val slowMode = Shuffleboard.getTab("5190").add("Slow Mode", false)

    override fun robotPeriodic() {
//        Shuffleboard.update()
//        Controls.update()
//
//        velocity.entry.setDouble(Flywheel.speed_SI)
//        voltage.entry.setDouble(Flywheel.voltage)
//        slowMode.entry.setBoolean(emergencyActive)
    }
}