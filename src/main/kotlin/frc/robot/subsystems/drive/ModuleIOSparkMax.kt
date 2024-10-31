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

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 *
 * NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 *
 * To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
open class ModuleIOSparkMax(index: Int) : ModuleIO {
    private var driveSparkMax: CANSparkMax? = null
    private var turnSparkMax: CANSparkMax? = null

    private val driveEncoder: RelativeEncoder
    private val turnRelativeEncoder: RelativeEncoder

    private val isTurnMotorInverted: Boolean = true

    private var cancoder: CANcoder? = null
    private val turnAbsolutePosition: StatusSignal<Double>
    private var absoluteEncoderOffset: Rotation2d? = null

    init {
        when (index) {
            0 -> {
                driveSparkMax = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)
                turnSparkMax = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless)
                cancoder = CANcoder(0)
                absoluteEncoderOffset =
                    Rotation2d(1.3698448436297932) // MUST BE CALIBRATED
            }

            1 -> {
                driveSparkMax = CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless)
                turnSparkMax = CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless)
                cancoder = CANcoder(1)
                absoluteEncoderOffset = Rotation2d(-1.9573594607102072) // MUST BE CALIBRATED
            }

            2 -> {
                driveSparkMax = CANSparkMax(5, CANSparkLowLevel.MotorType.kBrushless)
                turnSparkMax = CANSparkMax(6, CANSparkLowLevel.MotorType.kBrushless)
                cancoder = CANcoder(2)
                absoluteEncoderOffset =
                    Rotation2d(7.295612618859793) // MUST BE CALIBRATED
            }

            3 -> {
                driveSparkMax = CANSparkMax(7, CANSparkLowLevel.MotorType.kBrushless)
                turnSparkMax = CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushless)
                cancoder = CANcoder(3)
                absoluteEncoderOffset =
                    Rotation2d(3.471398522989793) // MUST BE CALIBRATED
            }

            else -> throw RuntimeException("Invalid module index")
        }

        driveSparkMax.restoreFactoryDefaults()
        turnSparkMax.restoreFactoryDefaults()

        driveSparkMax.setCANTimeout(250)
        turnSparkMax.setCANTimeout(250)

        driveEncoder = driveSparkMax.getEncoder()
        turnRelativeEncoder = turnSparkMax.getEncoder()

        turnSparkMax.setInverted(isTurnMotorInverted)
        driveSparkMax.setSmartCurrentLimit(60) //fixme: change this back to 60
        turnSparkMax.setSmartCurrentLimit(30)
        driveSparkMax.enableVoltageCompensation(12.0)
        turnSparkMax.enableVoltageCompensation(12.0)
        driveSparkMax.setInverted(true)

        driveEncoder.setPosition(0.0)
        driveEncoder.setMeasurementPeriod(10)
        driveEncoder.setAverageDepth(2)

        turnRelativeEncoder.setPosition(0.0)
        turnRelativeEncoder.setMeasurementPeriod(10)
        turnRelativeEncoder.setAverageDepth(2)

        driveSparkMax.setCANTimeout(0)
        turnSparkMax.setCANTimeout(0)

        driveSparkMax.burnFlash()
        turnSparkMax.burnFlash()

        cancoder.getConfigurator().apply(CANcoderConfiguration())

        turnAbsolutePosition = cancoder.getAbsolutePosition()
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        BaseStatusSignal.refreshAll(turnAbsolutePosition)
        inputs.drivePositionRad =
            Units.rotationsToRadians(driveEncoder.position) / DRIVE_GEAR_RATIO
        inputs.driveVelocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.velocity) / DRIVE_GEAR_RATIO
        inputs.driveAppliedVolts = driveSparkMax!!.appliedOutput * driveSparkMax.getBusVoltage()
        inputs.driveCurrentAmps = doubleArrayOf(driveSparkMax.getOutputCurrent())
        inputs.driveTemperature = doubleArrayOf(driveSparkMax.getMotorTemperature())

        inputs.turnAbsolutePosition =
            Rotation2d.fromRotations(turnAbsolutePosition.valueAsDouble)
                .minus(absoluteEncoderOffset)
        inputs.turnPosition =
            Rotation2d.fromRotations(turnRelativeEncoder.position / TURN_GEAR_RATIO)
        inputs.turnVelocityRadPerSec =
            (Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.velocity)
                    / TURN_GEAR_RATIO)
        inputs.turnAppliedVolts = turnSparkMax!!.appliedOutput * turnSparkMax.getBusVoltage()
        inputs.turnCurrentAmps = doubleArrayOf(turnSparkMax.getOutputCurrent())
        inputs.turnTemperature = doubleArrayOf(turnSparkMax.getMotorTemperature())
    }

    override fun setDriveVoltage(volts: Double) {
        //    Logger.recordOutput("Motor Voltage");
        driveSparkMax!!.setVoltage(volts)
    }

    override fun setTurnVoltage(volts: Double) {
        turnSparkMax!!.setVoltage(volts)
    }

    override fun setDriveBrakeMode(enable: Boolean) {
        driveSparkMax!!.setIdleMode(if (enable) IdleMode.kBrake else IdleMode.kCoast)
    }

    override fun setTurnBrakeMode(enable: Boolean) {
        turnSparkMax!!.setIdleMode(if (enable) IdleMode.kBrake else IdleMode.kCoast)
    }

    companion object {
        // Gear ratios for SDS MK4i L2, adjust as necessary
        const val DRIVE_GEAR_RATIO: Double = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
        const val TURN_GEAR_RATIO: Double = 150.0 / 7.0
    }
}
