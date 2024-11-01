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
package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotState
import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.drive.Drive
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import kotlin.math.hypot
import kotlin.math.withSign

object DriveCommands {
    private const val DEADBAND = 0.1
    private const val CANCEL_COMMAND_DEADBAND = 0.2
    private val rotationConstraints = TrapezoidProfile.Constraints(
        Units.RadiansPerSecond.of(9.315), Units.RadiansPerSecond.per(Units.Second).of(31.943)
    )

    /*
  -------------------
  ---- UTILITIES ----
  -------------------
   */
    private fun getLinearVelocity(
        xSupplier: DoubleSupplier, ySupplier: DoubleSupplier
    ): Translation2d {
        // Apply deadband
        var linearMagnitude =
            MathUtil.applyDeadband(hypot(xSupplier.asDouble, ySupplier.asDouble), DEADBAND)
        val linearDirection = Rotation2d(xSupplier.asDouble, ySupplier.asDouble)

        // Square values
        linearMagnitude *= linearMagnitude

        // Calcaulate new linear velocity
        return Pose2d(Translation2d(), linearDirection)
            .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
            .translation
    }


    //    public static Command ampAlign(Drive drive) {
    //        AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromChoreoTrajectory("AmpAlign"),);
    //    }
    /*
  ------------------
  ---- COMMANDS ----
  ------------------
   */
    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    fun joystickDrive(
        drive: Drive,
        xSupplier: DoubleSupplier,
        ySupplier: DoubleSupplier,
        omegaSupplier: DoubleSupplier
    ): Command {
        return Commands.run(
            {
                // Apply deadband
                var linearMagnitude =
                    MathUtil.applyDeadband(
                        hypot(xSupplier.asDouble, ySupplier.asDouble), DEADBAND
                    )
                val linearDirection =
                    Rotation2d(xSupplier.asDouble, ySupplier.asDouble)
                var omega = MathUtil.applyDeadband(omegaSupplier.asDouble, DEADBAND)

                // Square values
                linearMagnitude *= linearMagnitude
                omega = (omega * omega).withSign(omega)

                // Calcaulate new linear velocity
                val linearVelocity =
                    Pose2d(Translation2d(), linearDirection)
                        .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
                        .translation

                // Convert to field relative speeds & send command
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.x * Drive.maxLinearSpeedMetersPerSec,
                        linearVelocity.y * Drive.maxLinearSpeedMetersPerSec,
                        omega * Drive.maxAngularSpeedRadPerSec,
                        drive.rotation.rotateBy(
                            allianceRotation
                        )
                    )
                )
            },
            drive
        )
    }

    fun intakeAlign(
        drive: Drive,
        xSupplier: DoubleSupplier,
        ySupplier: DoubleSupplier,
        omegaSupplier: DoubleSupplier
    ): Command {
        val command =
            RunCommand(
                {
                    //                            // Calculate new linear velocity
                    val linearVelocity = getLinearVelocity(xSupplier, ySupplier)

                    var detectedNote = drive.detectedNote

                    if (detectedNote != null) {
                        // we have a target
                        detectedNote *= -0.1
                    } else {
                        // fall back to stick control
                        var omega = MathUtil.applyDeadband(omegaSupplier.asDouble, DEADBAND)
                        omega = (omega * omega).withSign(omega)
                        detectedNote = omega * Drive.maxAngularSpeedRadPerSec
                    }
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearVelocity.x * Drive.maxLinearSpeedMetersPerSec,
                            linearVelocity.y * Drive.maxLinearSpeedMetersPerSec,
                            detectedNote,
                            drive.rotation.rotateBy(allianceRotation)
                        )
                    )
                }, drive
            )

        return command
    }


    @JvmOverloads
    fun aimAtSpeakerCommand(
        drive: Drive,
        xSupplier: DoubleSupplier,
        ySupplier: DoubleSupplier,
        omegaSupplier: DoubleSupplier = DoubleSupplier { 0.0 }
    ): CommandAndReadySupplier {
        val previousPose = arrayOf<Pose2d?>(null)
        val rotationController =
            ProfiledPIDController(drive.rotationPID.kP, 0.0, drive.rotationPID.kD, rotationConstraints)

        rotationController.enableContinuousInput(-Math.PI, Math.PI)
        val filter = LinearFilter.singlePoleIIR(0.08, 0.02)

        val command =
            RunCommand(
                {
                    // Calculate new linear velocity
                    val linearVelocity = getLinearVelocity(xSupplier, ySupplier)
                    // Get the angle to point at the goal
                    val goalAngle =
                        ShooterCommands.speakerPos.toPose2d()
                            .translation
                            .minus(drive.pose.translation)
                            .angle
                    val robotVelocity = drive.twistPerDt
                    var targetPose = ShooterCommands.speakerPos.toPose2d()
                    targetPose = targetPose.plus(Transform2d(0.0, goalAngle.sin * 0.0, Rotation2d()))
                    val movingWhileShootingTarget: Pose2d = ShooterCommands.speakerPos.toPose2d()
                    Logger.recordOutput("speakerAimTargetPose", movingWhileShootingTarget)


                    var goalAngleVelocity: Double? = null
                    goalAngleVelocity = 0.0
                    Logger.recordOutput("Aim/goalAngleVelocity", goalAngleVelocity)
                    // calculate how much speed is needed to get there
//                  rotationController.reset(
//                      new TrapezoidProfile.State(
//                          Radians.of(drive.getRotation().getRadians()),
//                          drive.getAnglularVelocity()));
//                            rotationController.setGoal();
                    val value = rotationController.calculate(
                        MathUtil.angleModulus(drive.pose.rotation.radians),
                        TrapezoidProfile.State(goalAngle.radians, goalAngleVelocity)
                    )

                    Logger.recordOutput("Aim/Calculated Value", (value))
                    Logger.recordOutput("Aim/Goal Position", rotationController.goal.position)
                    Logger.recordOutput("Aim/Goal Velocity", rotationController.goal.velocity)
                    Logger.recordOutput("Aim/Setpoint Position Error", rotationController.positionError)
                    Logger.recordOutput("Aim/Setpoint Velocity Error", rotationController.velocityError)
                    Logger.recordOutput("Aim/Setpoint Velocity", rotationController.setpoint.velocity)
                    Logger.recordOutput("Aim/Setpoint Position", rotationController.setpoint.position)
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearVelocity.x * Drive.maxLinearSpeedMetersPerSec,
                            linearVelocity.y * Drive.maxLinearSpeedMetersPerSec,

                            (rotationController.setpoint.velocity + value),
                            drive.rotation.rotateBy(allianceRotation)
                        )
                    )
                    previousPose[0] = drive.pose
                }, drive
            )
                .beforeStarting(
                    {
                        rotationController.reset(
                            MathUtil.angleModulus(drive.rotation.radians),
                            drive.anglularVelocity.`in`(Units.RadiansPerSecond)
                        )
                    }, drive
                )
                .until {
                    // if the controller is giving a turn input, end the command
                    // because the driver is trying to take back control
                    val isGTE = omegaSupplier.asDouble >= CANCEL_COMMAND_DEADBAND
                    val isLTE = omegaSupplier.asDouble <= -CANCEL_COMMAND_DEADBAND
                    !RobotState.isAutonomous() && (isLTE || isGTE)
                }
        return CommandAndReadySupplier(command)
    }

    private val allianceRotation: Rotation2d
        //source angle is: 150 against side wall, 120 against speaker wall
        get() = Rotation2d.fromRotations(
            if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)) 0.5 else 0.0
        )

    class CommandAndReadySupplier(val command: Command)
}
