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
package frc.robot.subsystems.feeder

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs
import java.util.function.BooleanSupplier

class FeederIOSim : FeederIO {
    private val sim: FlywheelSim = FlywheelSim(
        DCMotor.getNEO(1), 1.5, 0.025
    )

    //  private ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0);
    //  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0);
    private var cycles: Int = 0

    private var intakeSupplier: BooleanSupplier = BooleanSupplier { cycles % 512 == 0 }
    private var shooterSupplier: BooleanSupplier = BooleanSupplier { cycles % 848 == 0 }

    constructor(intakeSupplier: BooleanSupplier, shooterSupplier: BooleanSupplier) {
        this.intakeSupplier = intakeSupplier
        this.shooterSupplier = shooterSupplier
    }

    constructor()

    private var appliedVolts: Double = 0.0

    override fun updateInputs(inputs: FeederIOInputs) {
        cycles++
        sim.setInputVoltage(appliedVolts)

        sim.update(0.02)

        inputs.positionRad = sim.angularVelocityRadPerSec
        inputs.velocityRadPerSec = sim.angularVelocityRadPerSec
        inputs.appliedVolts = appliedVolts
        inputs.currentAmps = doubleArrayOf(sim.currentDrawAmps)
        inputs.temperature = doubleArrayOf(0.0)
        inputs.beamUnobstructed = shooterSupplier.asBoolean
        inputs.intakebeamUnobstructed = intakeSupplier.asBoolean
    }

    override fun setVoltage(volts: Double) {
        appliedVolts = volts
        sim.setInputVoltage(volts)
    }

    override fun stop() {
        sim.setState(sim.angularVelocityRadPerSec)
        setVoltage(0.0)
    }
}
