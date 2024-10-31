package frc.robot.subsystems.lights

import com.ctre.phoenix.led.CANdle
import com.ctre.phoenix.led.CANdleConfiguration
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDs : SubsystemBase() {
    var candle: CANdle? = null

    init {
        if (RobotBase.isReal()) {
            candle = CANdle(0)
            candle!!.configFactoryDefault()
            val config = CANdleConfiguration()
            config.disableWhenLOS = true
            config.statusLedOffWhenActive = true
            config.stripType = CANdle.LEDStripType.GRB
            config.brightnessScalar = 1.0
            config.v5Enabled = false
            config.enableOptimizations = true
            candle!!.configAllSettings(config)
            for (i in 0 until candle!!.maxSimultaneousAnimationCount) {
                candle!!.clearAnimation(i)
            }
        }
    }

    override fun periodic() {
    }

    companion object {
        const val stripLength: Int = 16
        const val stripCount: Int = 4
        const val candleLength: Int = 8 // 0-7
    }
}
