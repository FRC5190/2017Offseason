/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot.subsytems.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX
import org.ghrobotics.robot.Controls

class ManualDriveCommand : FalconCommand(DriveSubsystem) {

    companion object {
        private const val deadband = 0.02
        private val leftSource = Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val rightSource = Controls.mainXbox.getY(GenericHID.Hand.kRight).withDeadband(deadband)

        private val front = Controls.mainXbox.getY(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val rotation = Controls.mainXbox.getX(GenericHID.Hand.kLeft).withDeadband(deadband)
        private val quickTurn = Controls.mainXbox.getRawButton(kX)
    }

    override suspend fun execute() {
//        DriveSubsystem.set(
//            ControlMode.PercentOutput,
//            -leftSource(),
//            -rightSource()
//        )
        DriveSubsystem.curvatureDrive(-front(), rotation(), quickTurn())
    }
}
