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
package frc.robot.subsystems.intake

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs
import org.littletonrobotics.junction.Logger

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
class IntakeIOSparkMax : IntakeIO {
    private val arm: CANSparkMax = CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless)
    private val roller: CANSparkMax = CANSparkMax(31, CANSparkLowLevel.MotorType.kBrushless)
    private val encoder: AbsoluteEncoder = arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
    private val rollerEncoder: AbsoluteEncoder = arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    init {
        arm.restoreFactoryDefaults()
        roller.restoreFactoryDefaults()
        arm.setIdleMode(IdleMode.kBrake)

        arm.setCANTimeout(250)
        roller.setCANTimeout(250)

        arm.inverted = true
        arm.enableVoltageCompensation(12.0)
        arm.setSmartCurrentLimit(30)

        roller.inverted = false
        roller.enableVoltageCompensation(12.0)
        roller.setSmartCurrentLimit(30)

        encoder.setInverted(false)

        arm.burnFlash()
        roller.burnFlash()
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        inputs.armPositionRad = MathUtil.angleModulus(Units.rotationsToRadians(encoder.position))
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.velocity)
        inputs.armAppliedVolts = arm.appliedOutput * arm.busVoltage
        inputs.armCurrentAmps = doubleArrayOf(arm.outputCurrent)
        inputs.armTemperature = doubleArrayOf(arm.motorTemperature)

        inputs.rollerPositionRad = MathUtil.angleModulus(Units.rotationsToRadians(rollerEncoder.position))
        inputs.rollerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rollerEncoder.velocity)
        inputs.rollerAppliedVolts = roller.appliedOutput * roller.busVoltage
        inputs.rollerCurrentAmps = doubleArrayOf(roller.outputCurrent)
        inputs.rollerTemperature = doubleArrayOf(roller.motorTemperature)
    }

    override fun setArmVoltage(volts: Double) {
        Logger.recordOutput("ArmSetVoltage", volts)
        arm.setVoltage(MathUtil.clamp(volts, -10.0, 10.0))
    }

    override fun setRollerPercent(percent: Double) {
        roller.setVoltage(percent * roller.busVoltage)
    }

    override fun setRollerVoltage(volts: Double) {
        roller.setVoltage(volts)
    } //

    //    @Override
    //    public void setArmVelocity(double velocityRadPerSec, double ffVolts) {
    //        pid.setReference(
    //                Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * ARM_GEAR_RATIO,
    //                ControlType.kVelocity,
    //                0,
    //                ffVolts,
    //                ArbFFUnits.kVoltage);
    //    }
    //
    //    @Override
    //    public void stop() {
    //        arm.stopMotor();
    //    }
    //
    //    @Override
    //    public void configurePID(double kP, double kI, double kD) {
    //        pid.setP(kP, 0);
    //        pid.setI(kI, 0);
    //        pid.setD(kD, 0);
    //        pid.setFF(0, 0);
    //    }
    companion object {
        private const val ARM_GEAR_RATIO: Double = 100.0
        private const val ROLLER_GEAR_RATIO: Double = 3.0
    }
}
