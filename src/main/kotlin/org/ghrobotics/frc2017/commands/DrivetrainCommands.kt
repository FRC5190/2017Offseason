package org.ghrobotics.frc2017.commands

import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import org.ghrobotics.frc2017.subsystems.Drivetrain

open class TeleopDriveCommand : SendableCommandBase() {

    init {
        addRequirements(Drivetrain)
        name = "Teleop Drive Command"
    }

    override fun execute() {
        println("command running")
    }
}