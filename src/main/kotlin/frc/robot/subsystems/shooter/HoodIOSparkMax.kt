package frc.robot.subsystems.shooter

import com.revrobotics.*
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.robot.subsystems.shooter.HoodIO.HoodIOInputs
import org.littletonrobotics.junction.Logger

class HoodIOSparkMax : HoodIO {
    private val leader: CANSparkMax = CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless)

    private val encoder: SparkAbsoluteEncoder = leader.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(0)

    private val motorEncoder: RelativeEncoder = leader.encoder
    var hasReset: Boolean = false
    private var hoodLimitSwitch: DigitalInput = DigitalInput(5)

    init {
        val codes: Array<REVLibError?> = arrayOfNulls(11)
        for (tries in 0..4) {
            var a: Int = 0
            codes[a++] = leader.restoreFactoryDefaults()
            codes[a++] = leader.setCANTimeout(250)
            leader.inverted = false
            codes[a++] = leader.enableVoltageCompensation(12.0)
            codes[a++] = leader.setSmartCurrentLimit(50)

            codes[a++] = motorEncoder.setPositionConversionFactor(MOTOR_TO_ROBOT)
            codes[a++] = motorEncoder.setVelocityConversionFactor(MOTOR_TO_ROBOT)
            codes[a++] = motorEncoder.setPosition(encoder.position)
            codes[a++] = encoder.setInverted(true)

            codes[a++] = leader.burnFlash()

            codes[a++] = motorEncoder.setPositionConversionFactor(MOTOR_TO_ROBOT)
            codes[a++] = motorEncoder.setVelocityConversionFactor(MOTOR_TO_ROBOT)
            //            codes[a] = motorEncoder.setPosition(MathUtil.angleModulus(encoder.getPosition() * Math.PI * 2) / GEAR_RATIO);
            var failed: Boolean = false
            for (code: REVLibError? in codes) {
                if (code != REVLibError.kOk) {
                    failed = true
                    println("an error occured while starting the motor")
                    println(codes.contentDeepToString())
                    println("waiting 1 second")
                    try {
                        Thread.sleep(1000)
                    } catch (e: InterruptedException) {
                        println("an error occured while sleeping:")
                        println(e)
                    }
                }
            }
            if (!failed) break
        }
        println("printing codes:")
        println(codes.contentDeepToString())
        println("finished printing codes.")
    }

    override fun updateInputs(inputs: HoodIOInputs) {
        inputs.isStalled = leader.getFault(CANSparkBase.FaultID.kStall)
        inputs.hoodPositionRad = (absoluteEncoder.absolutePosition * Math.PI * 2) / GEAR_RATIO
        inputs.motorPositionRad = motorEncoder.position
        inputs.hoodAppliedVolts = leader.busVoltage * leader.appliedOutput
        inputs.hoodCurrentAmps = doubleArrayOf(leader.outputCurrent)
        inputs.hoodVelocityRadPerSec = (motorEncoder.velocity * Math.PI * 2) / GEAR_RATIO
        inputs.hoodTemperature = doubleArrayOf(leader.motorTemperature)
        inputs.islimitSwitchPressed = hoodLimitSwitch.get()
    }

    override fun setBrakeMode(enable: Boolean) {
        leader.setIdleMode(if (enable) IdleMode.kBrake else IdleMode.kCoast)
    }

    override fun setVoltage(volts: Double) {
        Logger.recordOutput("HoodVoltage", volts)
        leader.setVoltage(MathUtil.clamp(volts, -5.0, 5.0))
    }

    companion object {
        private const val GEAR_RATIO: Double = 1.5
        private const val MOTOR_TO_ROBOT = (1 / (36 * GEAR_RATIO)) * Math.PI * 2
    }
}
