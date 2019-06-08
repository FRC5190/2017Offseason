package org.ghrobotics.frc2017.commands

import org.ghrobotics.frc2017.subsystems.Agitator
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.Source

class OpenLoopAgitatorCommand(private val percentSource: Source<Double>) : FalconCommand(Agitator) {
    override suspend fun execute() {
        Agitator.setOpenLoop(percentSource())
    }

    override suspend fun dispose() {
        Agitator.zeroOutputs()
    }
}