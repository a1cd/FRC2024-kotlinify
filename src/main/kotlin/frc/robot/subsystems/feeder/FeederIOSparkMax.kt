package frc.robot.subsystems.feeder

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs

class FeederIOSparkMax : FeederIO {
    var feeder: CANSparkMax = CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless)
    private val conveyorSensor: DigitalInput? = null

    init {
        feeder.restoreFactoryDefaults()

        feeder.setCANTimeout(250)

        feeder.inverted = false

        feeder.enableVoltageCompensation(12.0)
        feeder.setSmartCurrentLimit(30)

        feeder.burnFlash()

        val conveyorSensor: DigitalInput = DigitalInput(conveyorSensorNum)
    }

    override fun updateInputs(inputs: FeederIOInputs) {
        inputs.positionRad = Units.rotationsToRadians(feeder.encoder.position / GEAR_RATIO)
        inputs.velocityRadPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(feeder.encoder.velocity / GEAR_RATIO)
        inputs.appliedVolts = feeder.appliedOutput * feeder.busVoltage
        inputs.currentAmps = doubleArrayOf(feeder.outputCurrent)
        inputs.temperature = doubleArrayOf(feeder.motorTemperature)

        inputs.beamUnobstructed = conveyorSensor!!.get()
    }

    override fun setVoltage(volts: Double) {
        feeder.setVoltage(volts)
    }

    override fun stop() {
        feeder.setVoltage(0.0)
    }

    companion object {
        private const val GEAR_RATIO: Double = 1.0
        private const val conveyorSensorNum: Int = 9
    }
}
