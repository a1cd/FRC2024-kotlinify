package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.ControllerRumble
import kotlin.math.max
import kotlin.math.pow

object RumbleCommands {
    fun rumbleLight(controller: ControllerRumble): Command {
        return Commands.run({ controller.setRumbleLight(0.5) }, controller)
    }

    fun rumbleLightWithFalloff(controller: ControllerRumble): Command {
        return rumbleLightWithFalloff(controller, 40.0)
    }

    private fun rumbleLightWithFalloff(controller: ControllerRumble, falloffStrength: Double): Command {
        val c: Double = 0.025
        val n: Double = max(falloffStrength, 2.0)
        val offset: Double = (c.pow((-1) / (-n - 1)) * (1 / n).pow((-1) / (-n - 1)))
        val timer: Timer = Timer()
        return Commands.run({
            val elapsed: Double = timer.get()
            val strength: Double = 1 / ((elapsed + offset).pow(n) + 1)
            controller.setRumbleLight(strength)
        }, controller).beforeStarting({ timer.restart() })
    }

    fun noRumble(controller: ControllerRumble): Command {
        return Commands.run({ controller.setRumbleLight(0.0) }, controller)
    }
}
