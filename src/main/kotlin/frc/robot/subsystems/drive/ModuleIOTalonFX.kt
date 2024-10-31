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
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 *
 * NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 *
 * To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
class ModuleIOTalonFX(index: Int) : ModuleIO {
    private var driveTalon: TalonFX? = null
    private var turnTalon: TalonFX? = null
    private var cancoder: CANcoder? = null

    private val drivePosition: StatusSignal<Double>
    private val driveVelocity: StatusSignal<Double>
    private val driveAppliedVolts: StatusSignal<Double>
    private val driveCurrent: StatusSignal<Double>

    private val turnAbsolutePosition: StatusSignal<Double>
    private val turnPosition: StatusSignal<Double>
    private val turnVelocity: StatusSignal<Double>
    private val turnAppliedVolts: StatusSignal<Double>
    private val turnCurrent: StatusSignal<Double>

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private val DRIVE_GEAR_RATIO: Double = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)

    private val TURN_GEAR_RATIO: Double = 150.0 / 7.0

    private val isTurnMotorInverted: Boolean = true
    private var absoluteEncoderOffset: Rotation2d? = null

    init {
        when (index) {
            0 -> {
                driveTalon = TalonFX(0)
                turnTalon = TalonFX(1)
                cancoder = CANcoder(2)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }

            1 -> {
                driveTalon = TalonFX(3)
                turnTalon = TalonFX(4)
                cancoder = CANcoder(5)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }

            2 -> {
                driveTalon = TalonFX(6)
                turnTalon = TalonFX(7)
                cancoder = CANcoder(8)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }

            3 -> {
                driveTalon = TalonFX(9)
                turnTalon = TalonFX(10)
                cancoder = CANcoder(11)
                absoluteEncoderOffset = Rotation2d(0.0) // MUST BE CALIBRATED
            }

            else -> throw RuntimeException("Invalid module index")
        }

        val driveConfig: TalonFXConfiguration = TalonFXConfiguration()
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true
        driveTalon.getConfigurator().apply(driveConfig)
        setDriveBrakeMode(true)

        val turnConfig: TalonFXConfiguration = TalonFXConfiguration()
        turnConfig.CurrentLimits.StatorCurrentLimit = 30.0
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true
        turnTalon.getConfigurator().apply(turnConfig)
        setTurnBrakeMode(true)

        cancoder.getConfigurator().apply(CANcoderConfiguration())

        drivePosition = driveTalon.getPosition()
        driveVelocity = driveTalon.getVelocity()
        driveAppliedVolts = driveTalon.getMotorVoltage()
        driveCurrent = driveTalon.getStatorCurrent()

        turnAbsolutePosition = cancoder.getAbsolutePosition()
        turnPosition = turnTalon.getPosition()
        turnVelocity = turnTalon.getVelocity()
        turnAppliedVolts = turnTalon.getMotorVoltage()
        turnCurrent = turnTalon.getStatorCurrent()

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0, drivePosition, turnPosition
        ) // Required for odometry, use faster rate
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent
        )
        driveTalon.optimizeBusUtilization()
        turnTalon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent
        )

        inputs.drivePositionRad =
            Units.rotationsToRadians(drivePosition.valueAsDouble) / DRIVE_GEAR_RATIO
        inputs.driveVelocityRadPerSec =
            Units.rotationsToRadians(driveVelocity.valueAsDouble) / DRIVE_GEAR_RATIO
        inputs.driveAppliedVolts = driveAppliedVolts.valueAsDouble
        inputs.driveCurrentAmps = doubleArrayOf(driveCurrent.valueAsDouble)

        inputs.turnAbsolutePosition =
            Rotation2d.fromRotations(turnAbsolutePosition.valueAsDouble)
                .minus(absoluteEncoderOffset)
        inputs.turnPosition =
            Rotation2d.fromRotations(turnPosition.valueAsDouble / TURN_GEAR_RATIO)
        inputs.turnVelocityRadPerSec =
            Units.rotationsToRadians(turnVelocity.valueAsDouble) / TURN_GEAR_RATIO
        inputs.turnAppliedVolts = turnAppliedVolts.valueAsDouble
        inputs.turnCurrentAmps = doubleArrayOf(turnCurrent.valueAsDouble)
    }

    override fun setDriveVoltage(volts: Double) {
        driveTalon!!.setControl(VoltageOut(volts))
    }

    override fun setTurnVoltage(volts: Double) {
        turnTalon!!.setControl(VoltageOut(volts))
    }

    override fun setDriveBrakeMode(enable: Boolean) {
        val config: MotorOutputConfigs = MotorOutputConfigs()
        config.Inverted = InvertedValue.CounterClockwise_Positive
        config.NeutralMode = if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast
        driveTalon!!.configurator.apply(config)
    }

    override fun setTurnBrakeMode(enable: Boolean) {
        val config: MotorOutputConfigs = MotorOutputConfigs()
        config.Inverted =
            if (isTurnMotorInverted)
                InvertedValue.Clockwise_Positive
            else
                InvertedValue.CounterClockwise_Positive
        config.NeutralMode = if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast
        turnTalon!!.configurator.apply(config)
    }
}
