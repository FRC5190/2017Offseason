/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.robot.subsytems.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.wrappers.NativeFalconSRX
import org.ghrobotics.robot.Constants
import kotlin.properties.Delegates

object DriveSubsystem : FalconSubsystem() {

    private val leftMaster = NativeFalconSRX(Constants.kLeftMasterId)
    private val rightMaster = NativeFalconSRX(Constants.kRightMasterId)

    private val leftSlave1 = NativeFalconSRX(Constants.kLeftSlaveId1)
    private val rightSlave1 = NativeFalconSRX(Constants.kRightSlaveId1)

    private val allMasters = arrayOf(leftMaster, rightMaster)

    private val leftMotors = arrayOf(leftMaster, leftSlave1)
    private val rightMotors = arrayOf(rightMaster, rightSlave1)

    private val allMotors = arrayOf(*leftMotors, *rightMotors)

    var lowPowerMode by Delegates.observable(false) { _, _, lowPowerMode ->
        allMotors.forEach {
            it.voltageCompensationSaturation = 12.volt * (if (lowPowerMode) 0.7 else 1.0)
            it.voltageCompensationEnabled = true

            it.peakCurrentLimit = 0.amp
            it.peakCurrentLimitDuration = 0.millisecond
            it.continuousCurrentLimit = if (lowPowerMode) 20.amp else 40.amp
            it.currentLimitingEnabled = true

            it.openLoopRamp = if (lowPowerMode) 250.millisecond else 0.second
        }
    }

    init {
        mutableListOf(leftSlave1).forEach {
            it.follow(leftMaster)
            it.inverted = false
        }
        mutableListOf(rightSlave1).forEach {
            it.follow(rightMaster)
            it.inverted = true
        }

        leftMotors.forEach { it.inverted = false }
        rightMotors.forEach { it.inverted = true }

        allMotors.forEach {
            it.peakForwardOutput = 1.0
            it.peakReverseOutput = -1.0

            it.nominalForwardOutput = 0.0
            it.nominalReverseOutput = 0.0

            it.brakeMode = NeutralMode.Brake
        }

        defaultCommand = ManualDriveCommand()
    }

    override fun zeroOutputs() {
        set(ControlMode.PercentOutput, 0.0, 0.0)
    }

    fun set(controlMode: ControlMode, leftOutput: Double, rightOutput: Double) {
        leftMaster.set(controlMode, leftOutput)
        rightMaster.set(controlMode, rightOutput)
    }
}