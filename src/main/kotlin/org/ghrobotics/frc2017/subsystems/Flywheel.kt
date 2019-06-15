package org.ghrobotics.frc2017.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import edu.wpi.first.wpilibj.Notifier
import org.ghrobotics.frc2017.Constants
import org.ghrobotics.frc2017.controllers.FlywheelController
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.motors.ctre.FalconSRX

object Flywheel : FalconSubsystem() {

    // Hardware Controller
    private val masterMotor = FalconSRX(Constants.kFlywheelId, Constants.kFlywheelNativeUnitModel)

    // Software Controller
    private val controller = FlywheelController()

    // Subsystem Updates
    private var currentState = State.Nothing
    private var wantedState = State.Nothing
    private val periodicIO = PeriodicIO()

    val speed_SI get() = Constants.kFlywheelNativeUnitModel.fromNativeUnitVelocity(periodicIO.rawSensorVelocity)
    val voltage get() = periodicIO.voltage

    init {
        // Setup Hardware Controller
        masterMotor.feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative
        masterMotor.brakeMode = false
        masterMotor.talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5)

        masterMotor.encoder.encoderPhase = true

        masterMotor.talonSRX.configOpenloopRamp(0.0)

        masterMotor.talonSRX.configPeakOutputReverse(0.0)
        masterMotor.talonSRX.configPeakOutputForward(1.0)
        masterMotor.outputInverted = true

        defaultCommand = object : FalconCommand(this@Flywheel) {

            override fun initialize() {
                setNeutral()
            }
        }
    }

    override fun lateInit() {
        Notifier(this::update).startPeriodic(1.0 / 200.0)
    }

    override fun setNeutral() {
        wantedState = State.Nothing
    }

    fun setOpenLoop(percent: Double) {
        wantedState = State.OpenLoop
        periodicIO.demand = percent
    }

    fun setVelocity(velocity: Double) {
        wantedState = State.Velocity
        periodicIO.demand = velocity
    }

    private fun update() {
        periodicIO.voltage = masterMotor.voltageOutput
        periodicIO.current = masterMotor.talonSRX.outputCurrent

        periodicIO.rawSensorVelocity = masterMotor.encoder.rawVelocity

        controller.setMeasuredVelocity(speed_SI)
        controller.update()

        if (wantedState == State.Velocity) controller.setReference(periodicIO.demand)

        when (wantedState) {
            State.Nothing -> masterMotor.setNeutral()
            State.OpenLoop -> masterMotor.setDutyCycle(periodicIO.demand)
            State.Velocity -> masterMotor.setVoltage(controller.voltage)
        }

        if (currentState != wantedState) currentState = wantedState
    }

    private class PeriodicIO {
        var voltage = 0.0
        var current = 0.0

        var rawSensorVelocity = 0.0

        var demand = 0.0
    }

    private enum class State { OpenLoop, Velocity, Nothing }
}
