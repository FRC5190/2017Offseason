package org.ghrobotics.frc2017.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrame
import com.ctre.phoenix.sensors.PigeonIMU
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2017.Constants
import org.ghrobotics.frc2017.commands.TeleopDriveCommand
import org.ghrobotics.lib.localization.TankEncoderLocalization
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Rotation2d
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.subsystems.drive.TankDriveSubsystem

object Drivetrain : TankDriveSubsystem(), EmergencyHandleable {

    override val leftMotor = configureDriveGearbox(Constants.kDriveLeftMasterId, Constants.kDriveLeftSlaveId, true)
    override val rightMotor = configureDriveGearbox(Constants.kDriveRightMasterId, Constants.kDriveRightSlaveId, false)

    private val periodicIO = PeriodicIO()
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    private val gyro = PigeonIMU(Constants.kDrivePigeonId)

    override val differentialDrive = Constants.kDriveModel
    override val trajectoryTracker = RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta)

    override val localization = TankEncoderLocalization(
        { angle },
        { lPosition_SI },
        { rPosition_SI }
    )

    private val lPosition_SI: Double
        get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(periodicIO.leftRawSensorPosition)

    private val rPosition_SI: Double
        get() = Constants.kDriveNativeUnitModel.fromNativeUnitPosition(periodicIO.rightRawSensorPosition)

    private val angle: Rotation2d
        get() = Rotation2d.fromDegrees(periodicIO.gyroAngle)


    private fun configureDriveGearbox(masterId: Int, slaveId: Int, isLeft: Boolean): FalconSRX<Length> {

        val masterMotor = FalconSRX(masterId, Constants.kDriveNativeUnitModel)
        val slaveMotor = FalconSRX(slaveId, Constants.kDriveNativeUnitModel)

        slaveMotor.follow(masterMotor)

        masterMotor.outputInverted = !isLeft
        slaveMotor.outputInverted = !isLeft

        masterMotor.feedbackSensor = FeedbackDevice.QuadEncoder
        masterMotor.encoder.encoderPhase = true
        masterMotor.encoder.resetPosition(0.0)

        fun configMotor(motor: FalconSRX<Length>) {
            motor.talonSRX.configPeakOutputForward(1.0)
            motor.talonSRX.configPeakOutputReverse(-1.0)

            motor.talonSRX.configNominalOutputForward(0.0)
            motor.talonSRX.configNominalOutputReverse(0.0)

            motor.brakeMode = true

            motor.configCurrentLimit(
                true, FalconSRX.CurrentLimitConfig(
                    80.amp,
                    1.second,
                    Constants.kDriveCurrentLimit
                )
            )
        }

        configMotor(masterMotor)
        configMotor(slaveMotor)

        masterMotor.talonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10)

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
        }
        zeroOutputs()
    }

    override fun recoverFromEmergency() {
        listOf(leftMotor, rightMotor).forEach { masterMotor ->
            masterMotor.talonSRX.config_kP(0, Constants.kDriveKp)
            masterMotor.talonSRX.config_kD(0, Constants.kDriveKd)
        }
    }

    override fun periodic() {
        periodicIO.leftVoltage = leftMotor.voltageOutput
        periodicIO.rightVoltage = rightMotor.voltageOutput

        periodicIO.leftCurrent = leftMotor.talonSRX.outputCurrent
        periodicIO.rightCurrent = rightMotor.talonSRX.outputCurrent

        periodicIO.leftRawSensorPosition = leftMotor.encoder.rawPosition
        periodicIO.rightRawSensorPosition = rightMotor.encoder.rawPosition

        periodicIO.leftRawSensorVelocity = leftMotor.encoder.rawVelocity
        periodicIO.rightRawSensorVelocity = rightMotor.encoder.rawVelocity

        periodicIO.gyroAngle = gyro.fusedHeading

        when (wantedState) {
            State.Nothing -> {
                leftMotor.setNeutral()
                rightMotor.setNeutral()
            }
            State.PathFollowing -> {
                leftMotor.setVelocity(periodicIO.leftDemand, periodicIO.leftFeedforward)
                rightMotor.setVelocity(periodicIO.rightDemand, periodicIO.rightFeedforward)
            }
            State.OpenLoop -> {
                leftMotor.setDutyCycle(periodicIO.leftDemand)
                rightMotor.setDutyCycle(periodicIO.rightDemand)
            }
        }
        if (currentState != wantedState) currentState = wantedState
    }


    override fun tankDrive(leftPercent: Double, rightPercent: Double) = setOpenLoop(leftPercent, rightPercent)

    fun setOpenLoop(left: Double, right: Double) {
        wantedState = State.OpenLoop

        periodicIO.leftDemand = left
        periodicIO.rightDemand = right

        periodicIO.leftFeedforward = 0.0
        periodicIO.rightFeedforward = 0.0
    }

    override fun setOutput(wheelVelocities: DifferentialDrive.WheelState, wheelVoltages: DifferentialDrive.WheelState) {
        wantedState = State.PathFollowing

        periodicIO.leftDemand = wheelVelocities.left * differentialDrive.wheelRadius
        periodicIO.rightDemand = wheelVelocities.right * differentialDrive.wheelRadius

        periodicIO.leftFeedforward = wheelVoltages.left
        periodicIO.rightFeedforward = wheelVoltages.right
    }

    override fun zeroOutputs() {
        wantedState = State.Nothing

        periodicIO.leftDemand = 0.0
        periodicIO.rightDemand = 0.0
    }

    private class PeriodicIO {
        // Inputs
        var leftVoltage: Double = 0.0
        var rightVoltage: Double = 0.0

        var leftCurrent: Double = 0.0
        var rightCurrent: Double = 0.0

        var leftRawSensorPosition: Double = 0.0
        var rightRawSensorPosition: Double = 0.0
        var gyroAngle = 0.0

        var leftRawSensorVelocity: Double = 0.0
        var rightRawSensorVelocity: Double = 0.0

        // Outputs
        var leftDemand: Double = 0.0
        var rightDemand: Double = 0.0

        var leftFeedforward: Double = 0.0
        var rightFeedforward: Double = 0.0
    }

    private enum class State {
        PathFollowing, OpenLoop, Nothing
    }
}