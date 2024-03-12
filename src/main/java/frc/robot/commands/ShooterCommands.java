package frc.robot.commands;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static java.lang.Math.abs;
import static java.lang.Math.atan;


public class ShooterCommands {
    static double shooterAngleAdjustment = 0.0;

    static InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

    public static Pose3d getSpeakerPos() {
        return (DriverStation.getAlliance().orElse(Blue).equals(Blue)) ?
                new Pose3d(0.24, 5.50, 2.13, new Rotation3d()) :
                new Pose3d(16.27, 5.50, 2.13, new Rotation3d());
    }
    static Transform3d shooterOffset = new Transform3d(new Translation3d(0.0, 0.239, .669), new Rotation3d());

    private static double getDistance(Pose3d pose3d) {
        return pose3d.toPose2d().getTranslation().getNorm();
    }

    private static void populateITM() { //im making separate methods for this because I am not sure how much adjustments you would have to make
        distanceToAngle.put(0.0, 0.0);
        distanceToAngle.put(1.1, -0.2941);
        distanceToAngle.put(1.7, -0.4172);
        distanceToAngle.put(2.39, -0.2679);
        distanceToAngle.put(3.506, -0.1229);
        distanceToAngle.put(1000.0, 0.0);
        distanceToRPM.put(0.0, 3500.0);
        distanceToRPM.put(0.894, 3500.0);
        distanceToRPM.put(3.506, 5000.0);
        distanceToRPM.put(1000.0, 4000.0);
    }

    public static Command autoAim(Shooter shooter, Drive drive) {
        populateITM();
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return run(() -> {
            // Calcaulate new linear velocity
            // Get the angle to point at the goal
            var goalAngle =
                    ShooterCommands.getSpeakerPos().toPose2d()
                            .getTranslation()
                            .minus(drive.getPose().getTranslation())
                            .getAngle();
            Pose3d targetPose = ShooterCommands.getSpeakerPos();
            targetPose = targetPose.plus(new Transform3d(0.0, goalAngle.getSin() * 0.5, 0.0, new Rotation3d()));
            Pose3d shooter1 = new Pose3d(drive.getPose()).plus(shooterOffset);
            Pose3d pose3d = targetPose.relativeTo(shooter1);
            double distance = getDistance(pose3d);
            Logger.recordOutput("distanceFromGoal", distance);
            double atan = atan(pose3d.getZ() / distance);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(atan + distanceToAngle.get(distance)));
            shooter.shooterRunVelocity(distanceToRPM.get(distance));
        }, shooter)
                .withName("Auto Aim");
    }

    public static Command JustShoot(Shooter shooter) {
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return run(() -> {
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(.8));
            shooter.shooterRunVelocity(3500);
        }, shooter)
                .withName("Just Shoot");
    }

    public static Command shooterIdle(Shooter shooter) {
        return run(() -> {
            shooter.shooterRunVelocity(0.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(1.5));
        }, shooter)
                .withName("Shooter Idle");
    }

    public static Command simpleHoodZero(Shooter shooter) {
        Debouncer zeroStateDetection = new Debouncer(.25, Debouncer.DebounceType.kRising);
        return race(
                run(() -> {
                    shooter.setHoodPIDEnabled(false);
                    shooter.hoodRunVolts(2);
                }, shooter),
                sequence(
                        waitUntil(() -> !zeroStateDetection.calculate(
                                shooter.isStalled()
                                        || (abs(
                                        shooter.getHoodCharacterizationVelocity()
                                                .in(RadiansPerSecond)) > 1))),
                        runOnce(shooter::resetWhileZeroing),
                        runOnce(() -> shooter.setHasZeroed(true))
                )
        )
                .beforeStarting(print("Starting Hood Zero Sequence"))
                .withTimeout(1.5)
                .finallyDo(() -> {
                    System.out.println("Finished Hood Zero Sequence");
                    shooter.setHoodPIDEnabled(true);
                })
                .handleInterrupt(() -> shooter.setHoodPIDEnabled(true))
                .withName("Simple Hood Zero");
    }

    public static Command shooterSetZero(Shooter shooter) {
        return run(() -> {
            shooter.shooterRunVelocity(0.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0));
        }, shooter)
                .withName("shooterSetZero");
    }

    public static Command ampShoot(Shooter shooter) {
        return run(() -> {
            shooter.shooterRunVelocity(2000.0);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(0.5166099131107331));
        }, shooter)
                .withName("Amp Shoot");
    }

    public static Command addToOffset() {
        shooterAngleAdjustment += 0.017;
        return runOnce(() -> shooterAngleAdjustment += 0.017)
                .withName("Add to Offset");
    }

    public static Command removeFromOffset() {
        shooterAngleAdjustment -= 0.017;
        return runOnce(() -> shooterAngleAdjustment += 0.017)
                .withName("Subtract from Offset");
    }

    private static class Result {
        public final Pose3d pose3d;
        public final double distance;

        public Result(Pose3d pose3d, double distance) {
            this.pose3d = pose3d;
            this.distance = distance;
        }
    }
}
