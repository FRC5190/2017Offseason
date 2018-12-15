package org.ghrobotics.robot.subsytems.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.wrappers.NativeFalconSRX
import org.ghrobotics.robot.Constants

object ShooterSubsystem : FalconSubsystem() {

    private val shooterMaster = NativeFalconSRX(Constants.kShooterMaster)
    private val agitatorMaster = NativeFalconSRX(Constants.kAgitatorMaster)

    val doNothingCommand = object : FalconCommand(this@ShooterSubsystem) {
        override suspend fun initialize() {
            shooterMaster.set(ControlMode.PercentOutput, 0.0)
            agitatorMaster.set(ControlMode.PercentOutput, 0.0)
        }
    }

    val shootCommand = sequential {
        +object : FalconCommand(this@ShooterSubsystem) {
            init {
                withTimeout(500.millisecond)
            }

            override suspend fun initialize() {
                shooterMaster.set(ControlMode.PercentOutput, 0.8)
            }
        }
        +object : FalconCommand(this@ShooterSubsystem) {
            override suspend fun initialize() {
                agitatorMaster.set(ControlMode.PercentOutput, 0.4)
            }
        }
    }

    init {
        listOf(shooterMaster, agitatorMaster).forEach {
            it.peakCurrentLimit = 0.amp
            it.peakCurrentLimitDuration = 0.millisecond
            it.continuousCurrentLimit = 30.amp
            it.currentLimitingEnabled = true

            it.openLoopRamp = 250.millisecond
        }

        shooterMaster.inverted = true
        shooterMaster.peakForwardOutput = 1.0
        shooterMaster.peakReverseOutput = 0.0

        defaultCommand = doNothingCommand
    }
}

