package org.ghrobotics.frc2017.controllers

import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.statespace.StateSpaceControllerCoeffs
import org.ghrobotics.lib.mathematics.statespace.StateSpaceObserverCoeffs
import org.ghrobotics.lib.mathematics.statespace.StateSpacePlantCoeffs

object FlywheelCoeffs {
    val plantCoeffs = StateSpacePlantCoeffs(
        `2`, `1`, `1`,
        A = mat(`2`, `2`).fill(
            0.970595658870265, 0.8344721004414811,
            0.000000000000000, 0.0000000000000000
        ),
        B = mat(`2`, `1`).fill(
            0.8344721004414811,
            0.0000000000000000
        ),
        C = mat(`1`, `2`).fill(
            1.0, 0.0
        ),
        D = mat(`1`, `1`).fill(
            0.0
        )
    )

    val controllerCoeffs = StateSpaceControllerCoeffs<`2`, `1`, `1`>(
        K = mat(`1`, `2`).fill(
            0.7365346559390376, 1.0
        ),
        Kff = mat(`1`, `2`).fill(
            0.635754996350343, 0.0
        ),
        UMin = vec(`1`).fill(-12.0),
        UMax = vec(`1`).fill(+12.0)
    )

    val observerCoeffs = StateSpaceObserverCoeffs<`2`, `1`, `1`>(
        K = mat(`2`, `1`).fill(
            0.999900019415846,
            0.000000000000000
        )
    )
}
