/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2017.subsystems

import org.ghrobotics.frc2017.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.ctre.FalconSRX

object Agitator : FalconSubsystem() {

    private val masterMotor = FalconSRX(Constants.kAgitatorId, DefaultNativeUnitModel)

    private val periodicIO = PeriodicIO()
    private var currentState = State.Nothing
    private var wantedState = State.Nothing

    init {
        defaultCommand = object : FalconCommand(this@Agitator) {
            override fun initialize() {
                setNeutral()
            }
        }
    }

    override fun periodic() {
        periodicIO.voltage = masterMotor.voltageOutput
        periodicIO.current = masterMotor.talonSRX.outputCurrent.amp

        when (wantedState) {
            State.Nothing -> masterMotor.setNeutral()
            State.OpenLoop -> masterMotor.setDutyCycle(periodicIO.demand)
        }
        if (currentState != wantedState) currentState = wantedState
    }

    fun setOpenLoop(percent: Double) {
        wantedState = State.OpenLoop
        periodicIO.demand = percent
    }

    override fun setNeutral() {
        wantedState = State.Nothing
    }

    private class PeriodicIO {
        var voltage: SIUnit<Volt> = 0.volt
        var current: SIUnit<Ampere> = 0.amp

        var demand = 0.0
    }

    private enum class State {
        Nothing, OpenLoop
    }
}
