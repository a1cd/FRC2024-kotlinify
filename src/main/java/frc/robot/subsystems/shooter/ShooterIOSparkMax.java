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

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ShooterIOSparkMax implements ShooterIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkMax leader = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkPIDController pid = leader.getPIDController();

  public ShooterIOSparkMax() {
    leader.restoreFactoryDefaults();
    follower.restoreFactoryDefaults();

    leader.setCANTimeout(250);
    follower.setCANTimeout(250);

    leader.setInverted(false);
    follower.follow(leader, false);

    leader.enableVoltageCompensation(12.0);
    leader.setSmartCurrentLimit(30);

    leader.burnFlash();
    follower.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelPositionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.flywheelVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.flywheelAppliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.flywheelCurrentAmps =
        new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void flywheelStop() {
    leader.stopMotor();
  }
}