package frc.robot.subsystems.intake

import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        @JvmField
        var armPositionRad: Double = 0.0
        @JvmField
        var armVelocityRadPerSec: Double = 0.0
        @JvmField
        var armAppliedVolts: Double = 0.0
        @JvmField
        var armCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var armTemperature: DoubleArray = doubleArrayOf()

        @JvmField
        var rollerVelocityRadPerSec: Double = 0.0
        @JvmField
        var rollerAppliedVolts: Double = 0.0
        @JvmField
        var rollerCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var rollerPositionRad: Double = 0.0
        @JvmField
        var rollerTemperature: DoubleArray = doubleArrayOf()
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: IntakeIOInputs) {}

    /** Run open loop at the specified voltage.  */
    fun setArmVoltage(volts: Double) {}

    /** Set intake wheel voltage.  */
    fun setRollerPercent(percent: Double) {}

    fun setRollerVoltage(volts: Double) {
    }
}
