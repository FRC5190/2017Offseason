/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2017

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.lib.mathematics.units.amp
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.lb
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import kotlin.math.pow

@Suppress("MemberVisibilityCanBePrivate", "MagicNumber")
object Constants {

    const val kCTRETimeout = 10

    // Robot
    val kRobotMass = 60.lb
    const val kRobotMomentOfInertia = 10.0
    const val kRobotAngularDrag = 12.0
    val kRobotTrackWidth = 27.inch

    // Drivetrain
    const val kDriveLeftMasterId = 1
    const val kDriveLeftSlaveId = 2
    const val kDriveRightMasterId = 3
    const val kDriveRightSlaveId = 4
    const val kDrivePigeonId = 5

    val kDriveWheelRadius = 2.inch
    val kDriveNativeUnitModel = NativeUnitLengthModel(1440.nativeUnits, kDriveWheelRadius)

    const val kDriveKp = 2.75
    const val kDriveKd = 2.0

    const val kDriveKv = .15
    const val kDriveKa = .00
    const val kDriveKs = 1.2

    const val kDriveBeta = 2.0
    const val kDriveZeta = 0.7

    val kDriveDCTransmission = DCMotorTransmission(
        1 / kDriveKv,
        kDriveWheelRadius.value.pow(2) * kRobotMass.value / (2.0 * kDriveKa),
        kDriveKs
    )

    val kDriveModel = DifferentialDrive(
        kRobotMass.value,
        kRobotMomentOfInertia,
        kRobotAngularDrag,
        kDriveWheelRadius.value,
        kRobotTrackWidth.value / 2.0,
        kDriveDCTransmission,
        kDriveDCTransmission
    )

    val kDrivePeakCurrentLimit = 80.amp
    val kDriveCurrentLimit = 38.amp

    const val kSlowModeFactor = 0.5

    // Flywheel
    const val kFlywheelId = 13
    val kFlywheelNativeUnitModel = NativeUnitRotationModel(9000.nativeUnits)

    val kFlywheelClosedLoopVelocityTolerance = 50.degree.velocity

    const val kFlywheelNotifierPeriod = 1 / 200.0

    // Agitator
    const val kAgitatorId = 11
}
