// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface GyroIO {
    fun updateInputs(inputs: GyroIOInputs) {}

    @AutoLog
    class GyroIOInputs {
        @JvmField
        var connected: Boolean = false
        @JvmField
        var yawPosition: Rotation2d = Rotation2d()
        @JvmField
        var yawVelocityRadPerSec: Double = 0.0
        @JvmField
        var accelX: Double = 0.0
        @JvmField
        var accelY: Double = 0.0
        @JvmField
        var accelZ: Double = 0.0
        @JvmField
        var quatW: Double = 0.0
        @JvmField
        var quatX: Double = 0.0
        @JvmField
        var quatY: Double = 0.0
        @JvmField
        var quatZ: Double = 0.0
    }
}
