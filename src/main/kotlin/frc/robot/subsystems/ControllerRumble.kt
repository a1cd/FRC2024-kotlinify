package frc.robot.subsystems

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ControllerRumble(port: Int) : SubsystemBase() {
    private val controller: XboxController = XboxController(port)

    fun setRumbleLight(rumble: Double) {
        controller.setRumble(GenericHID.RumbleType.kRightRumble, rumble)
    }

    fun setRumbleHeavy(rumble: Double) {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, rumble)
    }
}
