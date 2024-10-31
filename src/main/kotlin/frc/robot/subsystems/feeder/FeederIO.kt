package frc.robot.subsystems.feeder

import org.littletonrobotics.junction.AutoLog

interface FeederIO {
    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: FeederIOInputs) {
    }

    /** Run open loop at the specified voltage.  */
    fun setVoltage(volts: Double) {
    }

    /** Stop in open loop.  */
    fun stop() {
    }

    @AutoLog
    class FeederIOInputs {
        @JvmField
        var beamUnobstructed: Boolean = false
        @JvmField
        var positionRad: Double = 0.0
        @JvmField
        var velocityRadPerSec: Double = 0.0
        @JvmField
        var appliedVolts: Double = 0.0
        @JvmField
        var currentAmps: DoubleArray = doubleArrayOf()
        @JvmField
        var temperature: DoubleArray = doubleArrayOf()
        @JvmField
        var intakebeamUnobstructed: Boolean = false
    }
}
