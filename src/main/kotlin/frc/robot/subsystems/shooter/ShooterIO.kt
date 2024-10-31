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
package frc.robot.subsystems.shooter

import org.littletonrobotics.junction.AutoLog

interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        @JvmField
        var flywheelPositionRad: Double = 0.0
        @JvmField
        var flywheelVelocityRadPerSec: Double = 0.0
        @JvmField
        var flywheelAppliedVolts: Double = 0.0
        @JvmField
        var flywheelVoltages: DoubleArray = doubleArrayOf()
        @JvmField
        var flywheelCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var flywheelTemperature: DoubleArray = doubleArrayOf()
        @JvmField
        var flywheelAncillaryTemperature: DoubleArray = doubleArrayOf()
        @JvmField
        var flywheelProcessorTemperature: DoubleArray = doubleArrayOf()
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ShooterIOInputs) {}

    /** Run open loop at the specified voltage.  */
    fun setFlywheelVoltage(volts: Double) {}

    /** Stop in open loop.  */
    fun flywheelStop() {}
}
