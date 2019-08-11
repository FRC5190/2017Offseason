package org.ghrobotics.frc2017.subsystems

import edu.wpi.first.wpilibj.experimental.command.Command
import edu.wpi.first.wpilibj.experimental.command.WaitUntilCommand
import org.ghrobotics.frc2017.Constants
import org.ghrobotics.frc2017.commands.ClosedLoopFlywheelCommand
import org.ghrobotics.frc2017.commands.OpenLoopAgitatorCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derivedunits.AngularVelocity
import kotlin.math.absoluteValue

object Superstructure {

    fun shootFuel(speed: AngularVelocity) = shootFuel(speed.value)
    fun shootFuel(speed_SI: Double): Command {
        return parallel {

            +ClosedLoopFlywheelCommand(speed_SI)
            +sequential {
                +WaitUntilCommand {
                    (Flywheel.speed_SI - speed_SI).absoluteValue < Constants.kFlywheelClosedLoopVelocityTolerance
                }
                +OpenLoopAgitatorCommand { 0.4 }
            }

        }
    }

}