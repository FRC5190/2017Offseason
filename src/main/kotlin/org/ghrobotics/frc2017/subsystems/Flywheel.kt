/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2017.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import edu.wpi.first.wpilibj.Notifier
import org.ghrobotics.frc2017.Constants
import org.ghrobotics.frc2017.controllers.FlywheelController
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitVelocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnitsPer100ms
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

    val speed get() =
        Constants.kFlywheelNativeUnitModel.fromNativeUnitVelocity(periodicIO.rawSensorVelocity)

    init {
        // Setup Hardware Controller
        masterMotor.feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative
        masterMotor.brakeMode = false

        @Suppress("MagicNumber")
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
        Notifier(this::update).startPeriodic(Constants.kFlywheelNotifierPeriod)
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
        periodicIO.current = masterMotor.talonSRX.outputCurrent.amp

        periodicIO.rawSensorVelocity = masterMotor.encoder.rawVelocity

        controller.setMeasuredVelocity(speed.value)
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
        var voltage: SIUnit<Volt> = 0.volt
        var current: SIUnit<Ampere> = 0.amp

        var rawSensorVelocity: SIUnit<NativeUnitVelocity> = 0.nativeUnitsPer100ms

        var demand = 0.0
    }

    private enum class State { OpenLoop, Velocity, Nothing }
}
