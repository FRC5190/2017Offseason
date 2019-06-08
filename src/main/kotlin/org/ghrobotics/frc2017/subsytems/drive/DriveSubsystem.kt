/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017.subsytems.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derivedunits.volt
import org.ghrobotics.lib.mathematics.units.millisecond
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.wrappers.NativeFalconSRX
import org.ghrobotics.frc2017.Constants
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

    val kDefaultQuickStopThreshold = 0.2
    val kDefaultQuickStopAlpha = 0.1

    private var m_quickStopThreshold = kDefaultQuickStopThreshold
    private var m_quickStopAlpha = kDefaultQuickStopAlpha
    private var m_quickStopAccumulator = 0.0

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
        leftMaster.set(controlMode, leftOutput * 0.6)
        rightMaster.set(controlMode, rightOutput * 0.6)
    }

    fun curvatureDrive(xSpeed: Double, zRotation: Double, isQuickTurn: Boolean) {
        var xSpeed = xSpeed
        var zRotation = zRotation

        xSpeed = limit(xSpeed)
        xSpeed = applyDeadband(xSpeed, 0.02)

        zRotation = limit(zRotation)
        zRotation = applyDeadband(zRotation, 0.02)

        val angularPower: Double
        val overPower: Boolean

        if (isQuickTurn) {
            if (Math.abs(xSpeed) < m_quickStopThreshold) {
                m_quickStopAccumulator =
                    (1 - m_quickStopAlpha) * m_quickStopAccumulator + m_quickStopAlpha * limit(zRotation) * 2.0
            }
            overPower = true
            angularPower = zRotation
        } else {
            overPower = false
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1.0
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1.0
            } else {
                m_quickStopAccumulator = 0.0
            }
        }

        var leftMotorOutput = xSpeed + angularPower
        var rightMotorOutput = xSpeed - angularPower

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0
                leftMotorOutput = 1.0
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0
                rightMotorOutput = 1.0
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0
                leftMotorOutput = -1.0
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0
                rightMotorOutput = -1.0
            }
        }

        // Normalize the wheel speeds
        val maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput))
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude
            rightMotorOutput /= maxMagnitude
        }

        set(ControlMode.PercentOutput, leftMotorOutput, rightMotorOutput)
    }

    fun limit(value: Double): Double {
        if (value > 1.0) {
            return 1.0
        }
        return if (value < -1.0) {
            -1.0
        } else value
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    fun applyDeadband(value: Double, deadband: Double): Double {
        return if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                (value - deadband) / (1.0 - deadband)
            } else {
                (value + deadband) / (1.0 - deadband)
            }
        } else {
            0.0
        }
    }
}