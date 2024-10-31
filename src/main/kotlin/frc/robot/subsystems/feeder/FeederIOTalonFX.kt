package frc.robot.subsystems.feeder

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs

class FeederIOTalonFX : FeederIO {
    private val conveyorSensor: DigitalInput = DigitalInput(conveyorSensorNum)
    private val intakeSensor: DigitalInput = DigitalInput(intakeSensorNum)
    private val feedMotor: TalonFX = TalonFX(43)
    private val feedMotorPosition: StatusSignal<Double> = feedMotor.position
    private val feedMotorVelocity: StatusSignal<Double> = feedMotor.velocity
    private val feedMotorAppliedVolts: StatusSignal<Double> = feedMotor.motorVoltage
    private val feedMotorCurrent: StatusSignal<Double> = feedMotor.statorCurrent
    private val feedMotorTemperature: StatusSignal<Double> = feedMotor.deviceTemp

    init {
        val config: TalonFXConfiguration = TalonFXConfiguration()
        config.CurrentLimits.StatorCurrentLimit = 30.0
        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast
        feedMotor.configurator.apply(config)

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            feedMotorPosition,
            feedMotorVelocity,
            feedMotorAppliedVolts,
            feedMotorCurrent,
            feedMotorTemperature
        )
        feedMotor.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: FeederIOInputs) {
        BaseStatusSignal.refreshAll(
            feedMotorPosition,
            feedMotorVelocity,
            feedMotorAppliedVolts,
            feedMotorCurrent,
            feedMotorTemperature
        )
        inputs.positionRad = Units.rotationsToRadians(feedMotorPosition.valueAsDouble)
        inputs.velocityRadPerSec =
            Units.rotationsToRadians(feedMotorVelocity.valueAsDouble)
        inputs.appliedVolts = feedMotorAppliedVolts.valueAsDouble
        inputs.currentAmps = doubleArrayOf(feedMotorCurrent.valueAsDouble)
        inputs.temperature = doubleArrayOf(feedMotorTemperature.valueAsDouble)
        inputs.beamUnobstructed = conveyorSensor.get()
        inputs.intakebeamUnobstructed = intakeSensor.get()
    }

    override fun setVoltage(volts: Double) {
        feedMotor.setVoltage(volts)
    }

    override fun stop() {
        feedMotor.stopMotor()
    }

    companion object {
        private const val conveyorSensorNum: Int = 9
        private const val intakeSensorNum: Int = 7
    }
}
