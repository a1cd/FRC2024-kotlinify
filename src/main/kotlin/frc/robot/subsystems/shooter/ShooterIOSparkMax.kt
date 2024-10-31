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

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs

class ShooterIOSparkMax : ShooterIO {
    private val leader = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    private val follower = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)
    private val encoder: RelativeEncoder = leader.encoder

    init {
        leader.restoreFactoryDefaults()
        follower.restoreFactoryDefaults()

        leader.setCANTimeout(250)
        follower.setCANTimeout(250)

        leader.inverted = false
        follower.follow(leader, false)

        leader.enableVoltageCompensation(12.0)
        leader.setSmartCurrentLimit(30)

        leader.burnFlash()
        follower.burnFlash()
    }

    override fun updateInputs(inputs: ShooterIOInputs) {
        inputs.flywheelVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(encoder.velocity / GEAR_RATIO)
        inputs.flywheelAppliedVolts = leader.appliedOutput * leader.busVoltage
        inputs.flywheelCurrentAmps =
            doubleArrayOf(leader.outputCurrent, follower.outputCurrent)
        inputs.flywheelTemperature =
            doubleArrayOf(leader.motorTemperature, follower.motorTemperature)
    }

    override fun setFlywheelVoltage(volts: Double) {
        leader.setVoltage(volts)
    }

    override fun flywheelStop() {
        leader.stopMotor()
    }

    companion object {
        private const val GEAR_RATIO = 1.5
    }
}
