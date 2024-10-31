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
package frc.robot.subsystems.climb

import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
@Suppress("unused")
class ClimbIOSparkMax : ClimbIO {
    private val left: CANSparkMax = CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless)
    private val right: CANSparkMax = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    private val leftEncoder: RelativeEncoder = left.encoder
    private val rightEncoder: RelativeEncoder = right.encoder

    init {
        left.restoreFactoryDefaults()
        right.restoreFactoryDefaults()

        left.setCANTimeout(250)
        right.setCANTimeout(250)

        left.enableVoltageCompensation(12.0) // wait for electrical to do climb wiring
        left.setSmartCurrentLimit(30) // wait for electrical to find the right breaker for the climb

        right.enableVoltageCompensation(12.0) // wait for electrical to do climb wiring
        right.setSmartCurrentLimit(30) // wait for electrical to find the right breaker for the climb

        left.setIdleMode(IdleMode.kBrake)
        right.setIdleMode(IdleMode.kBrake)

        left.burnFlash()
        right.burnFlash()
    }

    override fun updateInputs(inputs: ClimbIOInputs) {
        inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.position / LEFT_GEAR_RATIO)
        inputs.leftVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.velocity / LEFT_GEAR_RATIO)
        inputs.leftAppliedVolts = left.appliedOutput * left.busVoltage
        inputs.leftCurrentAmps = doubleArrayOf(left.outputCurrent)
        inputs.leftTemperature = doubleArrayOf(left.motorTemperature)
        inputs.rightPositionRad =
            Units.rotationsToRadians(rightEncoder.position / RIGHT_GEAR_RATIO)
        inputs.rightVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.velocity / RIGHT_GEAR_RATIO)
        inputs.rightAppliedVolts = right.appliedOutput * right.busVoltage
        inputs.rightCurrentAmps = doubleArrayOf(right.outputCurrent)
        inputs.rightTemperature = doubleArrayOf(right.motorTemperature)
    }

    // sets the left motor voltage
    override fun setLeftVoltage(volts: Double) {
        left.setVoltage(volts)
    }

    // sets the right motor voltage
    override fun setRightVoltage(volts: Double) {
        right.setVoltage(volts)
    }

    companion object {
        private const val LEFT_GEAR_RATIO: Double = 16.0
        private const val RIGHT_GEAR_RATIO: Double = 16.0
    }
}
