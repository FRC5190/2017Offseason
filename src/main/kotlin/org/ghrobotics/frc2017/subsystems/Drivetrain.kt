package org.ghrobotics.frc2017.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.team254.lib.physics.DifferentialDrive
import edu.wpi.first.wpilibj.experimental.command.SendableSubsystemBase
import org.ghrobotics.frc2017.commands.TeleopDriveCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem

object Drivetrain : SendableSubsystemBase() {

    val command = TeleopDriveCommand()

    init {
        defaultCommand = command
    }

    override fun periodic() {
        if (this.currentCommand != null) {
            println(this.currentCommand.name)
        }
    }
}