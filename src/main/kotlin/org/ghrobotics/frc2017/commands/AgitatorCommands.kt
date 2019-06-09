package org.ghrobotics.frc2017.commands

import edu.wpi.first.wpilibj.experimental.command.SendableCommandBase
import org.ghrobotics.frc2017.subsystems.Agitator
import org.ghrobotics.lib.utils.Source

class OpenLoopAgitatorCommand(private val percentSource: Source<Double>) : SendableCommandBase() {

    init {
        addRequirements(Agitator)
    }

    override fun execute() {
        Agitator.setOpenLoop(percentSource())
    }

    override fun end(interrupted: Boolean) {
        Agitator.setNeutral()
    }
}