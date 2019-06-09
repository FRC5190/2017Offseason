/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.command.Scheduler
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.ghrobotics.frc2017.subsystems.Agitator
import org.ghrobotics.frc2017.subsystems.Drivetrain
import org.ghrobotics.frc2017.subsystems.Flywheel
import org.ghrobotics.frc2017.subsystems.SubsystemHandler
import org.ghrobotics.lib.commands.FalconSubsystem

object Robot : TimedRobot() {

    init {
        +Drivetrain
        +Agitator
        +Flywheel
    }

    override fun robotInit() {
        SubsystemHandler.lateInit()
    }

    override fun autonomousInit() {
        SubsystemHandler.autoReset()
    }

    override fun teleopInit() {
        SubsystemHandler.teleopReset()
    }

    override fun disabledInit() {
        SubsystemHandler.zeroOutputs()
    }


    override fun robotPeriodic() {
        Shuffleboard.update()
        Scheduler.getInstance().run()
    }

    operator fun FalconSubsystem.unaryPlus() {
        SubsystemHandler.addSubsystem(this)
    }
}