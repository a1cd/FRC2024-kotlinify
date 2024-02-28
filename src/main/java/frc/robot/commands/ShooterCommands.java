package frc.robot.commands;


import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;


public class ShooterCommands {
    static InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();
    static InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
    static Pose3d speakerPos = new Pose3d(0.24, 5.50, 2.13, new Rotation3d());
    static Transform3d shooterOffset = new Transform3d(new Translation3d(0.0, 0.239, .669), new Rotation3d());

    private static double getDistance(Pose3d pose3d) {
        return pose3d.toPose2d().getTranslation().getNorm();
    }

    private static void populateITM() { //im making separate methods for this because I am not sure how much adjustments you would have to make
        distanceToAngle.put(0.0, 0.0);
        distanceToAngle.put(1000.0, 0.0);
        distanceToRPM.put(0.0, 3000.0);
        distanceToRPM.put(1000.0, 3000.0);
    }

    public static Command autoAim(Shooter shooter, Drive drive, DoubleSupplier supplier) {
        populateITM();
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return Commands.run(() -> {
            Pose3d shooter1 = new Pose3d(drive.getPose()).plus(shooterOffset);
            Pose3d pose3d = speakerPos.relativeTo(shooter1);
            double distance = getDistance(pose3d);
            double atan = Math.atan(pose3d.getZ() / distance);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(atan + supplier.getAsDouble()));
            shooter.shooterRunVelocity(3000);
        }, shooter).handleInterrupt(() -> {
            shooter.shooterRunVelocity(0.0);
        });
    }

    public static Command autoAimOnly(Shooter shooter, Drive drive, DoubleSupplier supplier) {
        populateITM();
        //the parameter is the robot, idk how to declare it, also this returns the angle
        return Commands.run(() -> {
            Pose3d shooter1 = new Pose3d(drive.getPose()).plus(shooterOffset);
            Pose3d pose3d = speakerPos.relativeTo(shooter1);
            double distance = getDistance(pose3d);
            Logger.recordOutput("distance", distance);
            double atan = Math.atan(pose3d.getZ() / distance);
            shooter.setTargetShooterAngle(Rotation2d.fromRadians(atan + supplier.getAsDouble()));
            shooter.shooterRunVelocity(0.0);
        }, shooter);
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
