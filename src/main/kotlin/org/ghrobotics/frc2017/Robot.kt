/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

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
        Controls.update()
    }
}
