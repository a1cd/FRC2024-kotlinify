package frc.robot.commands

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.subsystems.feeder.Feeder
import frc.robot.subsystems.intake.Intake

object IntakeCommands {
    fun intakeCommand(intake: Intake): Command {
        return RunCommand(
            {
                intake.setIntakePosition(Rotation2d.fromDegrees(-10.0))
                intake.setRollerVoltage(9.0)
            },
            intake
        )
    }

    private fun safeIntakeCommand(intake: Intake, feeder: Feeder): Command {
        return intakeCommand(intake)
            .onlyWhile { !feeder.beamBroken }
    }

    val sourcePos: Pose3d
        get() = if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)) Pose3d(
            15.424,
            0.909,
            2.13,
            Rotation3d()
        ) else Pose3d(16.27, 0.909, 2.13, Rotation3d())

    fun smartIntakeCommand(intake: Intake, feeder: Feeder): Command {
        return Commands.sequence(
            safeIntakeCommand(intake, feeder)
                .until { feeder.beamBroken },
            intakeCommand(intake)
                .withTimeout(1.0)
                .onlyWhile { feeder.intakeBeamBroken },
            flushIntakeWithoutTheArmExtendedOutward(intake, feeder)
                .onlyWhile { feeder.intakeBeamBroken }
        ).onlyIf { !(feeder.intakeBeamBroken || feeder.beamBroken) }
    }

    fun flushIntake(intake: Intake): Command {
        return RunCommand(
            {
                intake.setIntakePosition(Rotation2d.fromRadians(-2.05))
                intake.setRollerVoltage(-6.0)
            },
            intake
        )
    }

    fun flushIntakeWithoutTheArmExtendedOutward(intake: Intake, feeder: Feeder): Command {
        return RunCommand(
            {
                intake.setIntakePosition(Rotation2d.fromRadians(-2.05))
                intake.setRollerVoltage(6.0)
            }, intake
        )
            .raceWith(SpecializedCommands.timeoutDuringAutoSim(5.0))
            .until { !feeder.intakeBeamBroken }
    }

    fun idleCommand(intake: Intake): Command {
        return RunCommand(
            {
                intake.setIntakePosition(Rotation2d.fromRadians(-1.90))
                intake.setRollerVoltage(0.0)
            },
            intake
        )
    }

    fun setAngle(intake: Intake, angle: Double): Command {
        return RunCommand(
            {
                intake.setIntakePosition(Rotation2d.fromRadians(angle))
                intake.setRollerVoltage(0.0)
            },
            intake
        )
    }
}
