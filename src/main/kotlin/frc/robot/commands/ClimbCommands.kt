package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.Debouncer.DebounceType
import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.climb.Climb
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.abs

object ClimbCommands {
    // run left until cannot move, if cannot move
    fun zero(climb: Climb, currentThreshhold: Double): Command {
        val debouncers = object {
            var leftDebounce: Debouncer = Debouncer(1.0, DebounceType.kFalling)
            var cannotMoveAL: Boolean = false
            var cannotMoveAR: Boolean = false
            var cannotMoveBL: Boolean = false
            var cannotMoveBR: Boolean = false
            var rightDebounce: Debouncer = Debouncer(1.0, DebounceType.kFalling)

            fun reset() {
                leftDebounce = Debouncer(1.0, DebounceType.kFalling)
                cannotMoveAL = false
                cannotMoveAR = false
                cannotMoveBL = false
                cannotMoveBR = false
                rightDebounce = Debouncer(1.0, DebounceType.kFalling)
                rightDebounce.calculate(true)
                leftDebounce.calculate(true)
            }
        }
        val sequentialCommandGroup: SequentialCommandGroup = InstantCommand({ debouncers.reset() }).andThen(
            Commands.parallel(
                runLeft(climb, true)
                    .onlyWhile(
                        getDebounce(debouncers.leftDebounce,
                            DoubleSupplier { climb.leftVelocityRadPerSec },
                            Runnable { debouncers.cannotMoveAL = true })
                    )
                    .withTimeout(8.0),
                runRight(climb, true)
                    .onlyWhile(
                        getDebounce(debouncers.rightDebounce,
                            DoubleSupplier { climb.rightVelocityRadPerSec },
                            Runnable { debouncers.cannotMoveAR = true })
                    )
                    .withTimeout(8.0)
            )
        )
        sequentialCommandGroup.addRequirements(climb)
        return sequentialCommandGroup
    }

    private fun getDebounce(debouncers: Debouncer, climb: DoubleSupplier, onTrue: Runnable): BooleanSupplier {
        return BooleanSupplier {
            val calculate: Boolean = debouncers.calculate(abs(climb.asDouble) > 0.5)
            if (calculate) onTrue.run()
            calculate
        }
    }

    fun zeroClimb(climb: Climb): Command {
        return Commands.run({
            climb.runLeftVolts(4.0)
            climb.runRightVolts(4.0)
        })
    }

    fun runClimb(climb: Climb, leftSupplier: DoubleSupplier, rightSupplier: DoubleSupplier): Command {
        return Commands.run(
            {
                climb.runLeftVolts(
                    MathUtil.applyDeadband(leftSupplier.asDouble, 0.075) * 12
                )
                climb.runRightVolts(
                    MathUtil.applyDeadband(rightSupplier.asDouble, 0.075) * 12
                )
            },
            climb
        )
    }

    private fun runLeft(climb: Climb, direction: Boolean): RunCommand {
        return RunCommand({ climb.runLeftVolts((if (direction) 10 else -10).toDouble()) })
    }

    private fun runRight(climb: Climb, direction: Boolean): RunCommand {
        return RunCommand({ climb.runRightVolts((if (direction) 10 else -10).toDouble()) })
    }
}
