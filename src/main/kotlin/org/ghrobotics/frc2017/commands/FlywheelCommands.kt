package org.ghrobotics.frc2017.commands

import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import org.ghrobotics.frc2017.subsystems.Flywheel
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derivedunits.AngularVelocity
import org.ghrobotics.lib.utils.Source

class OpenLoopFlywheelCommand(private val percentSource: Source<Double>) : FalconCommand(Flywheel) {

    override fun execute() {
        Flywheel.setOpenLoop(percentSource())
    }

    override fun end(interrupted: Boolean) {
        Flywheel.setNeutral()
    }

}

class ClosedLoopFlywheelCommand(private val speed_SI: Double) : FalconCommand(Flywheel) {
    constructor(speed: AngularVelocity) : this(speed.value)

    override fun initialize() {
        Flywheel.setVelocity(speed_SI)
    }

    override fun end(interrupted: Boolean) {
        Flywheel.setNeutral()
    }
}