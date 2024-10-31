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

interface ModuleIO {
    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs) {}

    /** Run the drive motor at the specified voltage.  */
    fun setDriveVoltage(volts: Double) {}

    /** Run the turn motor at the specified voltage.  */
    fun setTurnVoltage(volts: Double) {}

    /** Enable or disable brake mode on the drive motor.  */
    fun setDriveBrakeMode(enable: Boolean) {}

    /** Enable or disable brake mode on the turn motor.  */
    fun setTurnBrakeMode(enable: Boolean) {}

    @AutoLog
    class ModuleIOInputs {
        @JvmField
        var drivePositionRad: Double = 0.0
        @JvmField
        var driveVelocityRadPerSec: Double = 0.0
        @JvmField
        var driveAppliedVolts: Double = 0.0
        @JvmField
        var driveCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var driveTemperature: DoubleArray = doubleArrayOf()

        @JvmField
        var turnAbsolutePosition: Rotation2d = Rotation2d()
        @JvmField
        var turnPosition: Rotation2d = Rotation2d()
        @JvmField
        var turnVelocityRadPerSec: Double = 0.0
        @JvmField
        var turnAppliedVolts: Double = 0.0
        @JvmField
        var turnCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var turnTemperature: DoubleArray = doubleArrayOf()
    }
}
