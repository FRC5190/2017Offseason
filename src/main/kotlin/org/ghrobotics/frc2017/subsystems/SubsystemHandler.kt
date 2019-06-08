package org.ghrobotics.frc2017.subsystems

import org.ghrobotics.lib.commands.FalconSubsystem
import java.util.concurrent.CopyOnWriteArrayList

object SubsystemHandler {

    private val subsystems = CopyOnWriteArrayList<FalconSubsystem>()

    private var alreadyStarted = false

    fun addSubsystem(subsystem: FalconSubsystem) {
        if (alreadyStarted) throw IllegalStateException("You cannot add a subsystem after the initialize stage")
        subsystems.add(subsystem)
        println("[FalconSubsystem Handler] Added ${subsystem.javaClass.simpleName}")
    }

    fun lateInit() = subsystems.forEach { it.lateInit() }

    fun autoReset() = subsystems.forEach { it.autoReset() }

    fun teleopReset() = subsystems.forEach { it.teleopReset() }

    // https://www.chiefdelphi.com/forums/showthread.php?t=166814
    fun zeroOutputs() = subsystems.forEach { it.zeroOutputs() }
}