package frc.robot.subsystems.climb

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs
import kotlin.math.abs

@Suppress("unused")
class ClimbIOSim : ClimbIO {
    private val leftSim = DCMotorSim(DCMotor.getNEO(1), 16.0, 0.025)
    private val rightSim = DCMotorSim(DCMotor.getNEO(1), 16.0, 0.025)

    private var leftAppliedVolts = 0.0
    private var rightAppliedVolts = 0.0

    //  private ProfiledPIDController pid = new ProfiledPIDController(0.0, 0.0, 0.0);
    //  private SimpleMotorFeedforward ffModel = new SimpleMotorFeedforward(0.0, 0.0);
    private var closedLoop = false

    override fun updateInputs(inputs: ClimbIOInputs) {
        if (closedLoop) {
            //      appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0,
            // 12.0);
            leftSim.setInputVoltage(leftAppliedVolts)
            rightSim.setInputVoltage(rightAppliedVolts)
        }

        leftSim.update(0.02)
        rightSim.update(0.02)

        inputs.leftPositionRad = leftSim.angularPositionRad
        //    inputs.leftPosition = (Rotation2d) new Rotation2d(leftSim.getAngularPositionRad());
        inputs.leftVelocityRadPerSec = leftSim.angularVelocityRadPerSec
        inputs.leftAppliedVolts = leftAppliedVolts
        inputs.leftCurrentAmps = doubleArrayOf(abs(leftSim.currentDrawAmps))
        inputs.rightPositionRad = rightSim.angularPositionRad
        //    inputs.rightPosition = (Rotation2d) new Rotation2d(rightSim.getAngularPositionRad());
        inputs.rightVelocityRadPerSec = rightSim.angularVelocityRadPerSec
        inputs.rightAppliedVolts = rightAppliedVolts
        inputs.rightCurrentAmps = doubleArrayOf(abs(rightSim.currentDrawAmps))
    }

    override fun setLeftVoltage(volts: Double) {
        closedLoop = false
        leftAppliedVolts = 0.0
        leftSim.setInputVoltage(volts)
    }

    override fun setRightVoltage(volts: Double) {
        closedLoop = false
        rightAppliedVolts = 0.0
        rightSim.setInputVoltage(volts)
    }
}
