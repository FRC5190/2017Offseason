/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017.subsytems.shooter

import com.ctre.phoenix.motorcontrol.ControlMode
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.wrappers.NativeFalconSRX
import org.ghrobotics.frc2017.Constants
import kotlin.properties.Delegates

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
                shooterMaster.set(ControlMode.PercentOutput, if (lowShoot) 0.8 else 1.0)
            }
        }
        +object : FalconCommand(this@ShooterSubsystem) {
            override suspend fun initialize() {
                agitatorMaster.set(ControlMode.PercentOutput, 0.4)
            }
        }
    }

    var lowPowerMode by Delegates.observable(false) { _, _, lowPowerMode ->
        listOf(shooterMaster, agitatorMaster).forEach {
            it.peakCurrentLimit = 0.amp
            it.peakCurrentLimitDuration = 0.millisecond
            it.continuousCurrentLimit = if (lowPowerMode) 30.amp else 40.amp
            it.currentLimitingEnabled = true
        }
    }

    var lowShoot = true

    init {
        listOf(shooterMaster, agitatorMaster).forEach {
            it.openLoopRamp = 250.millisecond
        }

        shooterMaster.inverted = true
        shooterMaster.peakForwardOutput = 1.0
        shooterMaster.peakReverseOutput = 0.0

        defaultCommand = doNothingCommand
    }
}

