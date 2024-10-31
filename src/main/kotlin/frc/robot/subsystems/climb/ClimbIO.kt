package frc.robot.subsystems.climb

import org.littletonrobotics.junction.AutoLog

interface ClimbIO {
    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ClimbIOInputs) {}

    /** Run open loop at the specified voltage.  */
    fun setLeftVoltage(volts: Double) {}

    fun setRightVoltage(volts: Double) {}

    /** Stop in open loop.  */
    fun leftStop() {}

    /** Stop in open loop.  */
    fun rightStop() {}

    @AutoLog
    class ClimbIOInputs {
        @JvmField
        var leftPositionRad: Double = 0.0
        @JvmField
        var leftVelocityRadPerSec: Double = 0.0
        @JvmField
        var leftAppliedVolts: Double = 0.0
        @JvmField
        var leftCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var leftTemperature: DoubleArray = doubleArrayOf()

        @JvmField
        var rightPositionRad: Double = 0.0
        @JvmField
        var rightVelocityRadPerSec: Double = 0.0
        @JvmField
        var rightAppliedVolts: Double = 0.0
        @JvmField
        var rightCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var rightTemperature: DoubleArray = doubleArrayOf()
    }
}
