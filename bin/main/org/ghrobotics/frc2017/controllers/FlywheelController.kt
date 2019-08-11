package org.ghrobotics.frc2017.controllers

import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.statespace.*

class FlywheelController {
    private val plant = StateSpacePlant(FlywheelCoeffs.plantCoeffs)
    private val controller = StateSpaceController(FlywheelCoeffs.controllerCoeffs, plant)
    private val observer = StateSpaceObserver(FlywheelCoeffs.observerCoeffs, plant)

    private var reference = vec(`2`).fill(0.0, 0.0)
    private var y = vec(`1`).fill(0.0)
    private var u = vec(`1`).fill(0.0)

    /**
     * Returns the input voltage in Volts
     */
    val voltage get() = u[0, 0]

    /**
     * Sets the controller reference
     * @param velocity The desired velocity in rad/s
     */
    fun setReference(velocity: Double) {
        reference[0, 0] = velocity
    }

    /**
     * Set the measured velocity
     * @param velocity The measured velocity in rad/s
     */
    fun setMeasuredVelocity(velocity: Double) {
        y[0, 0] = velocity
    }

    /**
     * Update the controller
     */
    fun update() {
        observer.correct(controller.u, y)
        observer.predict(controller.u)

        controller.update(reference, observer.xHat)

        this.u = controller.u
    }
}
