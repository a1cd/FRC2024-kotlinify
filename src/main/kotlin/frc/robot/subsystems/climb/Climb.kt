package frc.robot.subsystems.climb

import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Climb(private val io: ClimbIO) : SubsystemBase() {
    private val inputs: ClimbIOInputsAutoLogged = ClimbIOInputsAutoLogged()
    private val leftRelativeOffset: Double? = null // Relative + Offset = Absolute
    private val rightRelativeOffset: Double? = null // Relative + Offset = Absolute

    private var leftOffset: Double = 0.0
    private var rightOffset: Double = 0.0
    var leftVelocity: Double = 0.0
    var rightVelocity: Double = 0.0
    var leftMeasuredPosition: Double = 0.0
    var rightMeasuredPosition: Double = 0.0
    var leftGoalPosition: Double = 0.0
    var rightGoalPosition: Double = 0.0

    override fun periodic() {
        io.updateInputs(inputs)
        // sets voltages
        //    io.setLeftVoltage(
        //        pidController.calculate(inputs.leftPositionRad, leftGoalPosition)
        //            + ffModel.calculate(pidController.getSetpoint().velocity));
        //    io.setRightVoltage(
        //        pidController.calculate(inputs.rightPositionRad, rightGoalPosition)
        //            + ffModel.calculate(pidController.getSetpoint().velocity));
        Logger.processInputs("Climb", inputs)
    }

    /** Runs left motor at the specified voltage.  */
    fun runLeftVolts(volts: Double) {
        io.setLeftVoltage(volts)
    }

    /** Runs right motor at the specified voltage.  */
    fun runRightVolts(volts: Double) {
        io.setRightVoltage(volts)
    }

    /** Run closed loop to the specified position. (for left motor)  */
    fun runLeftPosition(leftPosition: Double) {
        val leftGoalPosition: Double = leftPosition
        // Log climb setpoint
        Logger.recordOutput("Climb left motor/SetpointRPM", leftGoalPosition)
    }

    /** Run closed loop to the specified position. (for right motor)  */
    fun runRightPosition(rightPosition: Double) {
        val rightGoalPosition: Double = rightPosition
        // Log climb setpoint
        Logger.recordOutput("Climb right motor/SetpointRPM", rightGoalPosition)
    }

    // idk what this does (for the left)
    fun resetLeftPosition() {
        leftOffset = inputs.leftPositionRad
        //    pidController.reset(inputs.leftPositionRad, leftVelocity = 0);
    }

    // idk what this does (for the right)
    fun resetRightPosition() {
        rightOffset = inputs.rightPositionRad
        //    pidController.reset(inputs.rightPositionRad, rightVelocity = 0);
    }

    val leftPositionMeters: Double
        /** Returns the current left position of the climb in meters.  */
        get() {
            return inputs.leftPositionRad * LEFT_RADIUS
        }

    val rightPositionMeters: Double
        /** Returns the current right position of the climb in meters.  */
        get() {
            return inputs.rightPositionRad * RIGHT_RADIUS
        }

    val leftVelocityRadPerSec: Double
        /** Returns the left motor velocity in radians/sec.  */
        get() {
            return inputs.leftVelocityRadPerSec
        }

    val rightVelocityRadPerSec: Double
        /** Returns the current velocity in radians per second.  */
        get() {
            return inputs.rightVelocityRadPerSec
        }

    companion object {
        private val LEFT_RADIUS: Double = Units.inchesToMeters(0.8) // FIGURE OUT RADIUS
        private val RIGHT_RADIUS: Double = Units.inchesToMeters(0.8) // FIGURE OUT RADIUS
    }
}
