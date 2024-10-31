package frc.robot.commands

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.Debouncer.DebounceType
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.feeder.Feeder
import frc.robot.subsystems.shooter.Shooter
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber
import kotlin.math.abs
import kotlin.math.atan


object ShooterCommands {
    private var shooterAngleAdjustment: Double = 0.0

    private var distanceToAngle: InterpolatingTreeMap<Double, Double> = InterpolatingDoubleTreeMap()
    private var distanceToRPM: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()
    private var shooterOffset: Transform3d = Transform3d(Translation3d(0.0, 0.239, .669), Rotation3d())

    private var isOverridden: LoggedDashboardBoolean = LoggedDashboardBoolean("Aim/Override Hood Angle", false)
    private var overrideAngle: LoggedDashboardNumber = LoggedDashboardNumber("Aim/Overridden Hood Angle", 0.0)
    private var offsetAngle: LoggedDashboardNumber = LoggedDashboardNumber("Aim/Offsetted Hood Angle", 0.0)


    fun inverseInterpolate(startValue: Double, endValue: Double, q: Double): Double {
        val totalRange: Double = endValue - startValue
        if (totalRange <= 0) {
            return 0.0
        }
        val queryToStart: Double = q - startValue
        if (queryToStart <= 0) {
            return 0.0
        }
        return queryToStart / totalRange
    }

    private var height: LoggedDashboardNumber = LoggedDashboardNumber("Shooter Tweak Height", 2.13)
    private var fartherIn: LoggedDashboardNumber =
        LoggedDashboardNumber("Shoot Farther In", Units.Inches.of(0.0).`in`(Units.Meters))

    private val speakerPos: Pose3d
        get() = if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)) Pose3d(
            0.24 - fartherIn.get(),
            5.50,
            height.get(),
            Rotation3d()
        ) else Pose3d(16.27 + fartherIn.get(), 5.50, height.get(), Rotation3d())

    val sourcePos: Pose3d
        get() {
            return if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)) Pose3d(
                15.428,
                0.916,
                2.13,
                Rotation3d()
            ) else Pose3d(1.122, 0.916, 2.13, Rotation3d())
        }

    fun diagShot(shooter: Shooter): Command {
        return Commands.run({
            shooter.shooterRunVelocity(2000.0)
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.1))
        }, shooter)
            .withName("Set diag shot")
    }

    private fun getDistance(pose3d: Pose3d): Double {
        return pose3d.toPose2d().translation.norm
    }

    //0.35 offset
    private fun construct() {
        distanceToAngle.clear()
        distanceToAngle.put(0.0, 0.0)
        //        distanceToAngle.put(1.0, -0.38);
//        distanceToAngle.put(1.633, -0.2);
//        distanceToAngle.put(2.0, 0.2);
//        distanceToAngle.put(2.29, 0.0);
//        distanceToAngle.put(1.1, -0.2941);
//        distanceToAngle.put(1.7, -0.3472);
//        distanceToAngle.put(2.52, -0.1700); //GOOD VALUES
//        distanceToAngle.put(4.235, 0.0150);
        distanceToAngle.put(1000.0, 0.0)

        distanceToRPM.clear()
        distanceToRPM.put(0.0, 3500.0)
        distanceToRPM.put(0.894, 3500.0)
        distanceToRPM.put(2.52, 3750.0) //GOOD VALUES
        //        distanceToRPM.put(3.506, 4000.0);
        distanceToRPM.put(4.25, 4000.0)
        distanceToRPM.put(1000.0, 4000.0)
    }

    private var retractAfterShot: LoggedDashboardBoolean = LoggedDashboardBoolean("Aim/Retract After Shooting", true)
    private var flywheelSpeed: LoggedDashboardNumber = LoggedDashboardNumber("Aim/FlywheelSpeed", 3000.0)

    fun autoAim(shooter: Shooter, drive: Drive, feeder: Feeder?): Command {
        construct()
        flywheelSpeed.periodic()
        isOverridden.periodic()
        overrideAngle.periodic()
        offsetAngle.periodic()
        retractAfterShot.periodic()
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return Commands.run({
            if (isOverridden.get()) {
                shooter.setTargetShooterAngle(Rotation2d.fromRadians(overrideAngle.get()))
                shooter.overrideHoodAtSetpoint(true)
                shooter.shooterRunVelocity(3000.0)
            } else {
                // Calcaulate new linear velocity
                // Get the angle to point at the goal
                val goalAngle: Rotation2d =
                    speakerPos.toPose2d()
                        .translation
                        .minus(drive.pose.translation)
                        .angle
                var targetPose: Pose3d = speakerPos
                targetPose = targetPose.plus(Transform3d(0.0, goalAngle.sin * 0.5, 0.0, Rotation3d()))
                val shooterLocation: Pose3d = Pose3d(drive.pose).plus(shooterOffset)
                val targetRelativeToShooter: Pose3d = targetPose.relativeTo(shooterLocation)
                val distance: Double = getDistance(targetRelativeToShooter)
                val atan: Double = atan(targetRelativeToShooter.z / distance)
                Logger.recordOutput("distanceFromGoal", distance)
                Logger.recordOutput("Aim/getZ", targetRelativeToShooter.z)
                Logger.recordOutput("Aim/atan", atan)
                shooter.setTargetShooterAngle(
                    Rotation2d.fromRadians(atan + distanceToAngle.get(distance))
                        .plus(Rotation2d.fromRadians(offsetAngle.get()))
                )
                shooter.overrideHoodAtSetpoint(true)
                shooter.shooterRunVelocity(distanceToRPM.get(distance))
            }
        }, shooter)
            .raceWith(SpecializedCommands.timeoutDuringAutoSim(2.0))
            .withName("Auto Aim")
    }

    fun JustShoot(shooter: Shooter): Command {
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.0))
            shooter.shooterRunVelocity(3000.0)
        }, shooter)
            .raceWith(SpecializedCommands.timeoutDuringAutoSim(2.0))
            .withName("Just Shoot")
    }

    fun passNote(shooter: Shooter): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0.25))
            shooter.shooterRunVelocity(4000.0)
        }, shooter).withName("Pass note")
    }

    fun shooterIdle(shooter: Shooter): Command {
        return Commands.run({
            shooter.shooterRunVelocity(0.0)
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.6))
        }, shooter)
            .withName("Shooter Idle")
    }

    fun stopShooter(shooter: Shooter): Command {
        return Commands.runOnce({
            shooter.shooterRunVelocity(0.0)
        }, shooter).withName("Stop Shooter")
    }

    fun forceShoot(shooter: Shooter): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.0))
            shooter.shooterRunVelocity(3000.0)
        })
    }

    fun simpleHoodZero(shooter: Shooter): Command {
        val zeroStateDetection: Debouncer = Debouncer(.2, DebounceType.kRising)
        return Commands.race(
            Commands.run({
                shooter.zeroMode = true
                shooter.setHoodPIDEnabled(false)
                //fixme: potentially make this higher (must do with testing, could damage robot)
                shooter.hoodRunVolts(2.0)
            }, shooter),
            Commands.sequence(
                Commands.waitSeconds(0.25).raceWith(
                    Commands.run(
                        {
                            zeroStateDetection.calculate(
                                shooter.isStalled
                                        || (abs(
                                    shooter.hoodCharacterizationVelocity
                                        .`in`(Units.RadiansPerSecond)
                                ) > 1)
                            )
                        })
                ),
                Commands.waitUntil {
                    !(zeroStateDetection.calculate(
                        shooter.isStalled
                                || (abs(
                            shooter.hoodCharacterizationVelocity
                                .`in`(Units.RadiansPerSecond)
                        ) > 1)
                    ))
                },
                Commands.runOnce({ shooter.resetWhileZeroing() }),
                Commands.runOnce({ shooter.setHasZeroed(true) })
            )
        )
            .withTimeout(4.0)
            .finallyDo(Runnable {
                shooter.zeroMode = false
                shooter.setHoodPIDEnabled(true)
            })
            .handleInterrupt {
                shooter.zeroMode = false
                shooter.setHoodPIDEnabled(true)
            }
            .withName("Simple Hood Zero")
    }

    fun shooterSetZero(shooter: Shooter): Command {
        return Commands.run({
            shooter.shooterRunVelocity(0.0)
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0.0))
        }, shooter)
            .withName("shooterSetZero")
    }

    fun ampShoot(shooter: Shooter): Command {
        return Commands.run({
            shooter.shooterRunVelocity(1500.0)
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0.5166099131107331))
        }, shooter)
            .withName("Amp Shoot")
    }

    fun pushIntoAmp(shooter: Shooter): Command {
        return Commands.run({
            shooter.shooterRunVelocity(100.0)
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(-.76))
        }, shooter)
            .withName("Push note into amp")
    }

    fun addToOffset(): Command {
        shooterAngleAdjustment += 0.017
        return Commands.runOnce({ shooterAngleAdjustment += 0.017 })
            .withName("Add to Offset")
    }

    fun removeFromOffset(): Command {
        shooterAngleAdjustment -= 0.017
        return Commands.runOnce({ shooterAngleAdjustment += 0.017 })
            .withName("Subtract from Offset")
    }

    fun humanPlayerIntake(shooter: Shooter): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.15))
            shooter.shooterRunVelocity(-1500.0)
        })
    }

    fun newAmpShoot(shooter: Shooter): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.114))
            shooter.shooterRunVelocity(1000.0)
        })
    }

    fun ampSpin(shooter: Shooter): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.114))
            shooter.shooterRunVelocity(600.0)
        })
    }

    fun setAmpAngle(shooter: Shooter, angle: Double): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(angle))
        })
    }

    fun ampGo(shooter: Shooter, rpm: Int): Command {
        return Commands.run({
            shooter.shooterRunVelocity(rpm.toDouble())
        })
    }

    fun ampAng(shooter: Shooter): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.5))
        })
    }

    fun ampAngle(shooter: Shooter): Command {
        return Commands.run({
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(-0.3))
        })
    }

    private class Result
}
