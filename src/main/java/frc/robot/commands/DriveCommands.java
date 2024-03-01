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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.math.MathUtil.inputModulus;
import static edu.wpi.first.units.Units.*;

public class DriveCommands {

    private static final double DEADBAND = 0.1;
    private static final double DEADBANDX = 1.0;
    private static final double CANCEL_COMMAND_DEADBAND = 0.2;
    private static final double DRIVE_ROTATION_P_VALUE = 35.0;
    private static TrapezoidProfile.Constraints rotationConstraints =
            new TrapezoidProfile.Constraints(
                    RadiansPerSecond.of(5), RadiansPerSecond.per(Second).of(5.0));

    private DriveCommands() {
    }

  /*
  -------------------
  ---- UTILITIES ----
  -------------------
   */

    public static Translation2d getLinearVelocity(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        // Apply deadband
        double linearMagnitude =
                applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        // Square values
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Calcaulate new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

  /*
  ------------------
  ---- COMMANDS ----
  ------------------
   */

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(  //make shooter command class with an auto aim method
                                          Drive drive,
                                          DoubleSupplier xSupplier,
                                          DoubleSupplier ySupplier,
                                          DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calcaulate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to field relative speeds & send command
                    drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * drive.getMaxAngularSpeedRadPerSec(),
                                    drive.getRotation()));

                },
                drive);
    }

    public static class CommandAndReadySupplier {
        private Command command;
        private BooleanSupplier readySupplier;

        private CommandAndReadySupplier(Command command, BooleanSupplier readySupplier) {
            this.command = command;
            this.readySupplier = readySupplier;
        }

        public Command getCommand() {
            return command;
        }

        public BooleanSupplier getReadySupplier() {
            return readySupplier;
        }
    }

    public static CommandAndReadySupplier aimAtSpeakerCommand(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {

        final Pose2d[] previousPose = {null};
        ProfiledPIDController rotationController =
                new ProfiledPIDController(.5, 0, .0, new TrapezoidProfile.Constraints(1, 2));

        rotationController.enableContinuousInput(Rotations.toBaseUnits(0), Rotations.toBaseUnits(1));

        var command =
                new RunCommand(
                        () -> {
//                  rotationController.setP(SmartDashboard.getNumber("rotationPidP", 0.0));
//                    rotationController.setI(SmartDashboard.getNumber("rotationPidI", 0.0));
//                    rotationController.setD(SmartDashboard.getNumber("rotationPidD", 0.0));

                            // Calcaulate new linear velocity
                            Translation2d linearVelocity = getLinearVelocity(xSupplier, ySupplier);
                            // Get the angle to point at the goal
                            var goalAngle =
                                    ShooterCommands.getSpeakerPos().toPose2d()
                                            .getTranslation()
                                            .minus(drive.getPose().getTranslation())
                                            .getAngle();
                            Transform2d robotVelocity;
                            Pose2d movingWhileShootingTarget;
                            if (previousPose[0] != null) {
                                robotVelocity = previousPose[0].minus(drive.getPose());

                                double distance =
                                        ShooterCommands.getSpeakerPos().toPose2d()
                                                .getTranslation()
                                                .getDistance(previousPose[0].getTranslation());
                                if (distance != 0) {
                                    movingWhileShootingTarget =
                                            ShooterCommands.getSpeakerPos().toPose2d().plus(
                                                    robotVelocity.times(0.02).times(16.5 / distance));
                                } else movingWhileShootingTarget = ShooterCommands.getSpeakerPos().toPose2d();
                            } else movingWhileShootingTarget = ShooterCommands.getSpeakerPos().toPose2d();
                            Logger.recordOutput("speakerAimTargetPose", movingWhileShootingTarget);
                    /*

                    |--------|
                    |-------|
                    |------|
                    |----|
                    |---|
                    |--|
                    |-|*
                    |=>
                     */


                            Measure<Velocity<Angle>> goalAngleVelocity = null;
                            if (previousPose[0] != null) {
                                var previousAngle =
                                        movingWhileShootingTarget
                                                .getTranslation()
                                                .minus(previousPose[0].getTranslation())
                                                .getAngle();
                                var currentAngle = goalAngle;
                                goalAngleVelocity =
                                        Radians.of(currentAngle.minus(previousAngle).getRadians())
                                                .per(Seconds.of(0.02));
                            } else goalAngleVelocity = RadiansPerSecond.zero();
                            Logger.recordOutput("goalAngleVelocity", goalAngleVelocity);
                            // calculate how much speed is needed to get ther
//                  rotationController.reset(
//                      new TrapezoidProfile.State(
//                          Radians.of(drive.getRotation().getRadians()),
//                          drive.getAnglularVelocity()));
//                  rotationController.setGoal(
//                      new TrapezoidProfile.State(
//                          Radians.of(goalAngle.getRadians())));
                            var value = rotationController.calculate(
                                    inputModulus(drive.getPose().getRotation().getRotations(), 0, 1),
//                            new TrapezoidProfile.State(
                                    inputModulus(goalAngle.getRotations(), 0, 1)
//                                    goalAngleVelocity.in(RotationsPerSecond)*.0025
//                            )
                            );
                            Logger.recordOutput("angle value", (value));
                            drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                            (value) * -6.28,
                                            drive.getRotation()));
                            previousPose[0] = drive.getPose();
                        }, drive)
                        .beforeStarting(() -> {
                            rotationController.reset(MathUtil.inputModulus(drive.getRotation().getRotations(), 0, 1));
                        }, drive)
                        .until(
                                () -> {
                                    // if the controller is giving a turn input, end the command
                                    // because the driver is trying to take back control
                                    var isGTE = omegaSupplier.getAsDouble() >= CANCEL_COMMAND_DEADBAND;
                                    var isLTE = omegaSupplier.getAsDouble() <= -CANCEL_COMMAND_DEADBAND;
                                    return isLTE || isGTE;
                                });
        return new CommandAndReadySupplier(command, () -> rotationController.atGoal());
    }

    public static CommandAndReadySupplier aimAtSpeakerCommand(
            Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        return aimAtSpeakerCommand(drive, xSupplier, ySupplier, () -> 0.0);
    }
}
