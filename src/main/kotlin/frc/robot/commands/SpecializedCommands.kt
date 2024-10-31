package frc.robot.commands

import edu.wpi.first.wpilibj.RobotState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants

object SpecializedCommands {
    fun timeoutDuringAuto(timeout: Double): Command {
        return Commands.either(
            Commands.waitSeconds(timeout),
            Commands.none()
        ) { RobotState.isAutonomous() }
    }

    fun timeoutDuringAutoSim(timeout: Double): Command {
        return Commands.either(
            Commands.waitSeconds(timeout),
            Commands.waitUntil { false }
        ) { RobotState.isAutonomous() && Constants.currentMode == Constants.Mode.SIM }
    }
}
