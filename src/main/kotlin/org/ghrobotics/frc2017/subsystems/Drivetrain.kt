/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2017.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2017.Constants
import org.ghrobotics.frc2017.commands.TeleopDriveCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem

object Drivetrain : TankDriveSubsystem(), EmergencyHandleable {

  override val leftMotor = configureDriveGearbox(
    Constants.kDriveLeftMasterId, Constants.kDriveLeftSlaveId, true
  )
  override val rightMotor = configureDriveGearbox(
    Constants.kDriveRightMasterId, Constants.kDriveRightSlaveId, false
  )

  private val periodicIO = PeriodicIO()
  private var currentState = State.Nothing
  private var wantedState = State.Nothing

//    private val gyro = PigeonIMU(Constants.kDrivePigeonId)

  override val differentialDrive = Constants.kDriveModel
  override val trajectoryTracker =
    RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

  override val localization = TankEncoderLocalization(
    { angle },
    { lPosition },
    { rPosition }
  )

  private val lPosition: SIUnit<Meter>
    get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(
      periodicIO.leftRawSensorPosition
    )

  private val rPosition: SIUnit<Meter>
    get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(
      periodicIO.rightRawSensorPosition
    )

  private val angle: Rotation2d
    get() = Rotation2d.fromDegrees(periodicIO.gyroAngle)

  private fun configureDriveGearbox(
    masterId: Int,
    slaveId: Int,
    isLeft: Boolean
  ): FalconSRX<Meter> {
    val masterMotor = FalconSRX(masterId, Constants.kDriveNativeUnitModel)
    val slaveMotor = FalconSRX(slaveId, Constants.kDriveNativeUnitModel)

    slaveMotor.follow(masterMotor)

    masterMotor.outputInverted = !isLeft
    slaveMotor.outputInverted = !isLeft

    masterMotor.feedbackSensor = FeedbackDevice.QuadEncoder
    masterMotor.encoder.encoderPhase = true
    masterMotor.encoder.resetPosition(0.meter)

    fun configMotor(motor: FalconSRX<Meter>) {
      motor.talonSRX.configPeakOutputForward(1.0)
      motor.talonSRX.configPeakOutputReverse(-1.0)

      motor.talonSRX.configNominalOutputForward(0.0)
      motor.talonSRX.configNominalOutputReverse(0.0)

      motor.brakeMode = true

      motor.configCurrentLimit(
        true,
        FalconSRX.CurrentLimitConfig(
          Constants.kDrivePeakCurrentLimit,
          1.second,
          Constants.kDriveCurrentLimit
        )
      )
    }

    configMotor(masterMotor)
    configMotor(slaveMotor)

    @Suppress("MagicNumber")
    masterMotor.talonSRX.setStatusFramePeriod(
      StatusFrame.Status_2_Feedback0,
      10
    )

    masterMotor.talonSRX.config_kP(0, Constants.kDriveKp)
    masterMotor.talonSRX.config_kD(0, Constants.kDriveKd)

    return masterMotor
  }

  init {
    defaultCommand = TeleopDriveCommand()
  }

  override fun activateEmergency() {
    listOf(leftMotor, rightMotor).forEach { masterMotor ->
      masterMotor.talonSRX.config_kP(0, 0.0)
      masterMotor.talonSRX.config_kD(0, 0.0)
      masterMotor.talonSRX.configPeakOutputForward(Constants.kSlowModeFactor)
      masterMotor.talonSRX.configPeakOutputReverse(-Constants.kSlowModeFactor)
    }
    zeroOutputs()
  }

  override fun recoverFromEmergency() {
    listOf(leftMotor, rightMotor).forEach { masterMotor ->
      masterMotor.talonSRX.config_kP(0, Constants.kDriveKp)
      masterMotor.talonSRX.config_kD(0, Constants.kDriveKd)
      masterMotor.talonSRX.configPeakOutputForward(1.0)
      masterMotor.talonSRX.configPeakOutputReverse(-1.0)
    }
  }

  override fun setNeutral() {
    zeroOutputs()
  }

  override fun periodic() {
    periodicIO.leftVoltage = leftMotor.voltageOutput
    periodicIO.rightVoltage = rightMotor.voltageOutput

    periodicIO.leftCurrent = leftMotor.talonSRX.outputCurrent.amp
    periodicIO.rightCurrent = rightMotor.talonSRX.outputCurrent.amp

    periodicIO.leftRawSensorPosition = leftMotor.encoder.rawPosition
    periodicIO.rightRawSensorPosition = rightMotor.encoder.rawPosition

    periodicIO.leftRawSensorVelocity = leftMotor.encoder.rawVelocity
    periodicIO.rightRawSensorVelocity = rightMotor.encoder.rawVelocity

    periodicIO.gyroAngle = 0.0

    when (wantedState) {
      State.Nothing -> {
        leftMotor.setNeutral()
        rightMotor.setNeutral()
      }
      State.PathFollowing -> {
        leftMotor.setVelocity(
          periodicIO.leftDemand.meter.velocity, periodicIO.leftFeedforward
        )
        rightMotor.setVelocity(
          periodicIO.rightDemand.meter.velocity, periodicIO.rightFeedforward
        )
      }
      State.OpenLoop -> {
        leftMotor.setDutyCycle(periodicIO.leftDemand)
        rightMotor.setDutyCycle(periodicIO.rightDemand)
      }
    }
    if (currentState != wantedState) currentState = wantedState
  }

  override fun tankDrive(leftPercent: Double, rightPercent: Double) =
    setOpenLoop(leftPercent, rightPercent)

  private fun setOpenLoop(left: Double, right: Double) {
    wantedState = State.OpenLoop

    periodicIO.leftDemand = left
    periodicIO.rightDemand = right

    periodicIO.leftFeedforward = 0.volt
    periodicIO.rightFeedforward = 0.volt
  }

  override fun setOutput(
    wheelVelocities: DifferentialDrive.WheelState,
    wheelVoltages: DifferentialDrive.WheelState
  ) {
    wantedState = State.PathFollowing

    periodicIO.leftDemand = wheelVelocities.left * differentialDrive.wheelRadius
    periodicIO.rightDemand =
      wheelVelocities.right * differentialDrive.wheelRadius

    periodicIO.leftFeedforward = wheelVoltages.left.volt
    periodicIO.rightFeedforward = wheelVoltages.right.volt
  }

  override fun zeroOutputs() {
    wantedState = State.Nothing

    periodicIO.leftDemand = 0.0
    periodicIO.rightDemand = 0.0
  }

  private class PeriodicIO {
    // Inputs
    var leftVoltage: SIUnit<Volt> = 0.volt
    var rightVoltage: SIUnit<Volt> = 0.volt

    var leftCurrent: SIUnit<Ampere> = 0.amp
    var rightCurrent: SIUnit<Ampere> = 0.amp

    var leftRawSensorPosition: SIUnit<NativeUnit> = 0.nativeUnits
    var rightRawSensorPosition: SIUnit<NativeUnit> = 0.nativeUnits
    var gyroAngle = 0.0

    var leftRawSensorVelocity: SIUnit<NativeUnitVelocity> =
      0.nativeUnitsPer100ms
    var rightRawSensorVelocity: SIUnit<NativeUnitVelocity> =
      0.nativeUnitsPer100ms

    // Outputs
    var leftDemand: Double = 0.0
    var rightDemand: Double = 0.0

    var leftFeedforward: SIUnit<Volt> = 0.volt
    var rightFeedforward: SIUnit<Volt> = 0.volt
  }

  private enum class State {
    PathFollowing, OpenLoop, Nothing
  }
}
