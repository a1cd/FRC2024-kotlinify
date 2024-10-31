package frc.robot.util

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.RobotContainer

class ModeHelper(private val container: RobotContainer) {
    private val mode: Mode = Mode.NEUTRAL

    private var transition: Command? = null

    private fun switchTo(target: Mode) {
        if (mode != Mode.NEUTRAL && target == mode) {
            switchTo(Mode.NEUTRAL)
            return
        }

        val exit: Command = container.getExitCommand(mode)
        val enter: Command = container.getEnterCommand(target)

        transition =
            Commands.sequence(
                exit.withInterruptBehavior(
                    Command.InterruptionBehavior.kCancelIncoming
                ),  /* command group to exit mode */
                enter
            )
        transition.schedule()
    }

    fun cancelTransition() {
        if (transition != null) transition!!.cancel()
    }
}
