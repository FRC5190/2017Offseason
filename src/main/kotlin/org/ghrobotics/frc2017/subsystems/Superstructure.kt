package org.ghrobotics.frc2017.subsystems

import org.ghrobotics.frc2017.Constants
import org.ghrobotics.frc2017.commands.ClosedLoopFlywheelCOmmand
import org.ghrobotics.frc2017.commands.OpenLoopAgitatorCommand
import org.ghrobotics.lib.commands.ConditionCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derivedunits.AngularVelocity
import kotlin.math.absoluteValue

object Superstructure {

    fun shootFuel(speed: AngularVelocity) = shootFuel(speed.value)
    fun shootFuel(speed_SI: Double): FalconCommand {
        return parallel {

            +ClosedLoopFlywheelCOmmand(speed_SI)
            +sequential {
                +ConditionCommand {
                    (Flywheel.speed_SI - speed_SI).absoluteValue < Constants.kFlywheelClosedLoopVelocityTolerance
                }
                +OpenLoopAgitatorCommand { 0.4 }
            }

        }
    }

}