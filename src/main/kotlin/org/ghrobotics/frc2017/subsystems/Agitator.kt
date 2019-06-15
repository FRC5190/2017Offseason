package org.ghrobotics.frc2017.subsystems

import org.ghrobotics.frc2017.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.nativeunits.DefaultNativeUnitModel
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
        periodicIO.current = masterMotor.talonSRX.outputCurrent

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
        var voltage = 0.0
        var current = 0.0

        var demand = 0.0
    }

    private enum class State {
        Nothing, OpenLoop
    }
}