package org.ghrobotics.frc2017.commands

import org.ghrobotics.frc2017.subsystems.Flywheel
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derivedunits.AngularVelocity
import org.ghrobotics.lib.utils.Source

class OpenLoopFlywheelCommand(private val percentSource: Source<Double>) : FalconCommand(Flywheel) {
    override suspend fun execute() {
        Flywheel.setOpenLoop(percentSource())
    }

    override suspend fun dispose() {
        Flywheel.zeroOutputs()
    }
}

class ClosedLoopFlywheelCOmmand(private val speed_SI: Double) : FalconCommand(Flywheel) {
    constructor(speed: AngularVelocity) : this(speed.value)

    override suspend fun initialize() {
        Flywheel.setVelocity(speed_SI)
    }

    override suspend fun dispose() {
        Flywheel.zeroOutputs()
    }
}