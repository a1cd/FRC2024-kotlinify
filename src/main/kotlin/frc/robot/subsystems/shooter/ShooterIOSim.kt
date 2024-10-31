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

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs

class ShooterIOSim : ShooterIO {
    private val sim = FlywheelSim(DCMotor.getKrakenX60(2), 1.5, 0.004)
    private val armSim = SingleJointedArmSim(
        DCMotor.getNEO(1),
        9.0,
        .1,
        .2,
        Units.Radians.convertFrom(90.0, Units.Degrees),
        Units.Radians.convertFrom(-5.0, Units.Degrees),
        true,
        Units.Radians.convertFrom(45.0, Units.Degrees)
    )
    private val pid = PIDController(0.0, 0.0, 0.0)

    private var closedLoop = false
    private val ffVolts = 0.0
    private var appliedVolts = 0.0

    override fun updateInputs(inputs: ShooterIOInputs) {
        sim.update(0.02)

        inputs.flywheelVelocityRadPerSec = sim.angularVelocityRadPerSec
        inputs.flywheelAppliedVolts = appliedVolts
        inputs.flywheelCurrentAmps = doubleArrayOf(sim.currentDrawAmps)
    }

    override fun setFlywheelVoltage(volts: Double) {
        closedLoop = false
        appliedVolts = 0.0
        sim.setInputVoltage(volts)
    }

    override fun flywheelStop() {
        setFlywheelVoltage(0.0)
    }
}
