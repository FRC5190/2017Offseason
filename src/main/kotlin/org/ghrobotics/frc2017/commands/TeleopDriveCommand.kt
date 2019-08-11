/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2017.commands

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2017.Controls
import org.ghrobotics.frc2017.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX

open class TeleopDriveCommand : FalconCommand(Drivetrain) {

  override fun execute() {
    val curvature = rotationSource()
    val linear = -speedSource()

    Drivetrain.curvatureDrive(linear, curvature, quickTurnSource())
  }

  companion object {
    private const val kDeadband = 0.05
    val speedSource by lazy {
      Controls.driverController.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband)
    }
    val rotationSource by lazy {
      Controls.driverController.getX(GenericHID.Hand.kLeft).withDeadband(kDeadband)
    }
    val quickTurnSource by lazy { Controls.driverController.getRawButton(kX) }
  }
}
