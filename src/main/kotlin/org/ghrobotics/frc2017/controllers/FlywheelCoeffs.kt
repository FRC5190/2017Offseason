/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

@file:Suppress("WildcardImport")

package org.ghrobotics.frc2017.controllers

import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.statespace.StateSpaceControllerCoeffs
import org.ghrobotics.lib.mathematics.statespace.StateSpaceObserverCoeffs
import org.ghrobotics.lib.mathematics.statespace.StateSpacePlantCoeffs

@Suppress("MagicNumber")
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
            0.24, 1.0
        ),
        Kff = mat(`1`, `2`).fill(
            0.0, 0.0
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
