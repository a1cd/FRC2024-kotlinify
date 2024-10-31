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

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs
import kotlin.math.abs

/**
 * Physics sim implementation of module IO.
 *
 *
 * Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
class ModuleIOSim : ModuleIO {
    private val driveSim: DCMotorSim =
        DCMotorSim(LinearSystemId.createDCMotorSystem(0.13437 / 6.75, 0.13437 / 6.75), DCMotor.getNEO(1), 6.75)
    private val turnSim: DCMotorSim = DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004)

    private val turnAbsoluteInitPosition: Rotation2d = Rotation2d(Math.random() * 2.0 * Math.PI)
    private var driveAppliedVolts: Double = 0.0
    private var turnAppliedVolts: Double = 0.0

    constructor()

    constructor(ignored: Int)

    override fun updateInputs(inputs: ModuleIOInputs) {
        driveSim.update(LOOP_PERIOD_SECS)
        turnSim.update(LOOP_PERIOD_SECS)

        inputs.drivePositionRad = driveSim.angularPositionRad
        inputs.driveVelocityRadPerSec = driveSim.angularVelocityRadPerSec
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveCurrentAmps = doubleArrayOf(abs(driveSim.currentDrawAmps))

        inputs.turnAbsolutePosition =
            Rotation2d(turnSim.angularPositionRad).plus(turnAbsoluteInitPosition)
        inputs.turnPosition = Rotation2d(turnSim.angularPositionRad)
        inputs.turnVelocityRadPerSec = turnSim.angularVelocityRadPerSec
        inputs.turnAppliedVolts = turnAppliedVolts
        inputs.turnCurrentAmps = doubleArrayOf(abs(turnSim.currentDrawAmps))
    }

    override fun setDriveVoltage(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        driveSim.setInputVoltage(driveAppliedVolts)
    }

    override fun setTurnVoltage(volts: Double) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        turnSim.setInputVoltage(turnAppliedVolts)
    }

    companion object {
        private const val LOOP_PERIOD_SECS: Double = 0.02
    }
}
