/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2017.subsystems

import edu.wpi.first.wpilibj.experimental.command.Command
import edu.wpi.first.wpilibj.experimental.command.WaitUntilCommand
import org.ghrobotics.frc2017.Constants
import org.ghrobotics.frc2017.commands.ClosedLoopFlywheelCommand
import org.ghrobotics.frc2017.commands.OpenLoopAgitatorCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.AngularVelocity

@Suppress("SpacingAroundUnaryOperators")
object Superstructure {
    fun shootFuel(speed: SIUnit<AngularVelocity>): Command {
        return parallel {
            +ClosedLoopFlywheelCommand(speed)
            +sequential {
                +WaitUntilCommand {
                    (Flywheel.speed - speed).absoluteValue <
                            Constants.kFlywheelClosedLoopVelocityTolerance
                }
                @Suppress("MagicNumber")
                +OpenLoopAgitatorCommand { 0.25 }
            }
        }
    }
}
