package frc.robot.subsystems.intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs

class IntakeIOSim : IntakeIO {
    private val rollerSim: FlywheelSim = FlywheelSim(DCMotor.getNEO(1), 3.0, .01)
    private val armSim: SingleJointedArmSim = SingleJointedArmSim(
        DCMotor.getNEO(1),
        100.0,
        .1819,
        Units.inchesToMeters(7.063364),
        Math.PI * -.2,
        Math.PI * .8,
        true,
        0.0
    ) // mass is 8.495 lbs
    private var armVoltage: Double = 0.0
    private var rollerVoltage: Double = 0.0
    private var timestamp: Double? = null

    init {
        setArmVoltage(0.0)
        setRollerPercent(0.0)
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        val ct: Double = Timer.getFPGATimestamp()
        val dt: Double = if ((timestamp == null)) .02 else ct - timestamp!!
        inputs.armCurrentAmps = doubleArrayOf(armSim.currentDrawAmps)
        inputs.armAppliedVolts = -armVoltage
        inputs.armPositionRad = -armSim.angleRads
        inputs.armVelocityRadPerSec = -armSim.velocityRadPerSec
        armSim.update(0.02)

        inputs.rollerCurrentAmps = doubleArrayOf(rollerSim.currentDrawAmps)
        inputs.rollerAppliedVolts = rollerVoltage
        inputs.rollerVelocityRadPerSec = rollerSim.angularVelocityRadPerSec
        rollerSim.update(dt)
        timestamp = ct
    }

    override fun setArmVoltage(volts: Double) {
        armVoltage = MathUtil.clamp(-volts, -12.0, 12.0)
        armSim.setInputVoltage(armVoltage)
    }

    override fun setRollerPercent(percent: Double) {
        rollerVoltage = percent * 12.0
        rollerSim.setInputVoltage(rollerVoltage)
    }
}
