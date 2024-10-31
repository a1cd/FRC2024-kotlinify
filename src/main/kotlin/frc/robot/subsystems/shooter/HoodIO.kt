package frc.robot.subsystems.shooter

import org.littletonrobotics.junction.AutoLog

interface HoodIO {
    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: HoodIOInputs) {}

    @AutoLog
    class HoodIOInputs {
        @JvmField
        var motorPositionRad: Double = 0.0
        @JvmField
        var hoodPositionRad: Double = 0.0
        @JvmField
        var hoodVelocityRadPerSec: Double = 0.0
        @JvmField
        var hoodAppliedVolts: Double = 0.0
        @JvmField
        var hoodCurrentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var hoodTemperature: DoubleArray = doubleArrayOf()
        @JvmField
        var isStalled: Boolean = false
        @JvmField
        var islimitSwitchPressed: Boolean = false
    }

    /** Run open loop at the specified voltage.  */
    fun setVoltage(volts: Double) {}

    /** Stop in open loop.  */
    fun wristStop() {}

    fun setBrakeMode(enableBrakeMode: Boolean) {}
}
