package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.feeder.Feeder
import java.util.function.BooleanSupplier

object FeederCommands {
    fun idleFeeder(feeder: Feeder): Command {
        return Commands.repeatingSequence(
            Commands.either(
                Commands.run({ feeder.runVolts(0.0) })
                    .onlyWhile(BooleanSupplier { feeder.beamBroken }),
                Commands.sequence(
                    Commands.run({ feeder.runVolts(0.0) })
                        .onlyWhile { !feeder.beamBroken },
                    zeroToBeamBreak(feeder)
                ),
                BooleanSupplier { feeder.beamBroken }
            ).beforeStarting({ feeder.state = (Feeder.State.none) })
        )
    }

    fun feedToShooter(feeder: Feeder): Command {
        return Commands.sequence(
            Commands.run({ feeder.runVolts(9.0) }, feeder)
                .onlyIf(BooleanSupplier { feeder.beamBroken })
                .raceWith(SpecializedCommands.timeoutDuringAutoSim(2.0))
                .until { !feeder.beamBroken },
            Commands.run({ feeder.runVolts(9.0) }, feeder)
                .withTimeout(0.5),
            Commands.runOnce({ feeder.runVolts(0.0) })
        )
            .beforeStarting({ feeder.state = Feeder.State.feedingShooter })
            .finallyDo { interrupted: Boolean -> feeder.state = Feeder.State.none }
    }

    fun flushFeeder(feeder: Feeder): Command {
        return Commands.startEnd(
            { feeder.runVolts(-4.0) },
            { feeder.runVolts(0.0) },
            feeder
        )
    }

    fun feedToBeamBreak(feeder: Feeder): Command {
        return Commands.either(
            zeroToBeamBreak(feeder),
            Commands.sequence(
                Commands.sequence(
                    Commands.run({ feeder.runVolts(6.0) }, feeder)
                        .onlyWhile { !feeder.intakeBeamBroken }
                        .raceWith(SpecializedCommands.timeoutDuringAutoSim(2.0)),
                    Commands.run({ feeder.runVolts(6.0) }, feeder)
                        .onlyWhile(BooleanSupplier { feeder.intakeBeamBroken })
                        .raceWith(SpecializedCommands.timeoutDuringAutoSim(3.0))
                )
                    .onlyWhile { !feeder.beamBroken },
                Commands.either(
                    zeroToBeamBreak(feeder),
                    slowToBeam(feeder),
                    BooleanSupplier { feeder.beamBroken }
                )
            ),
            BooleanSupplier { feeder.beamBroken })
            .beforeStarting({ feeder.state = Feeder.State.zeroingNote })
            .finallyDo { interrupted: Boolean -> feeder.state = (Feeder.State.none) }
    }

    private fun slowToBeam(feeder: Feeder): Command {
        return Commands.run({ feeder.runVolts(2.0) }, feeder)
            .onlyWhile { !feeder.beamBroken }
            .raceWith(SpecializedCommands.timeoutDuringAutoSim(2.0))
    }

    private fun zeroToBeamBreak(feeder: Feeder): Command {
        return Commands.sequence(
            Commands.run({ feeder.runVolts(-6.0) }, feeder)
                .onlyWhile(BooleanSupplier { feeder.beamBroken }),
            slowToBeam(feeder)
        )
            .raceWith(SpecializedCommands.timeoutDuringAutoSim(1.0))
    }

    fun humanPlayerIntake(feeder: Feeder): Command {
        return Commands.startEnd(
            { feeder.runVolts(-8.0) },
            { feeder.runVolts(0.0) }
        )
    } //    public static Command feedToBeamBreak(Feeder feeder) {
    //        return sequence(
    //                run(() -> {
    //                    feeder.runVolts(6);
    //                }, feeder)
    //                        .onlyWhile(() -> !feeder.getBeamBroken()),
    //                run(() -> {
    //                    feeder.runVolts(-4);
    //                }, feeder)
    //                        .onlyWhile(feeder::getBeamBroken),
    //                run(() -> {
    //                    feeder.runVolts(3);
    //                }, feeder)
    //                        .onlyWhile(() -> !feeder.getBeamBroken())
    //        );
    //    }
}
