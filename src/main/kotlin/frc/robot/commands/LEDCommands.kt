package frc.robot.commands

import com.ctre.phoenix.led.*
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.*
import frc.robot.RobotContainer
import frc.robot.subsystems.lights.LEDs

object LEDCommands {
    private var wantsHPI: Boolean = false
    private fun flameCommand(leds: LEDs?, brightness: Double): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle
        return Commands.startEnd(
            {
                candle!!.animate(RgbFadeAnimation(1.0, 0.5, LEDs.candleLength, 0), 0)
                candle.animate(
                    FireAnimation(
                        brightness,
                        0.0,
                        LEDs.stripLength,
                        1.0,
                        .5,
                        false,
                        LEDs.candleLength
                    ), 1
                )
                candle.animate(
                    FireAnimation(
                        brightness,
                        0.0,
                        LEDs.stripLength,
                        1.0,
                        .5,
                        true,
                        (LEDs.candleLength) + LEDs.stripLength
                    ), 2
                )
                candle.animate(
                    FireAnimation(
                        brightness,
                        0.0,
                        LEDs.stripLength,
                        1.0,
                        .5,
                        false,
                        (LEDs.candleLength) + LEDs.stripLength * 2
                    ), 3
                )
                candle.animate(
                    FireAnimation(
                        brightness,
                        0.0,
                        LEDs.stripLength,
                        1.0,
                        .5,
                        true,
                        (LEDs.candleLength) + LEDs.stripLength * 3
                    ), 4
                )
            },
            {
                for (i in 0..4) candle!!.clearAnimation(i)
            },
            leds
        )
    }

    fun setIntakeBoolean(): Command {
        return Commands.runOnce({ wantsHPI = !wantsHPI })
            .withName("Change Intake Mode")
    }

    fun setIntakeType(leds: LEDs?): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle

        return Commands.runEnd(
            {
                if (wantsHPI) {
                    candle!!.animate(
                        SingleFadeAnimation(
                            255,
                            255,
                            50,
                            0,
                            .1,
                            LEDs.stripLength * 4,
                            LEDs.candleLength
                        ), 0
                    )
                } else {
                    candle!!.animate(
                        SingleFadeAnimation(
                            0,
                            0,
                            80,
                            0,
                            .1,
                            LEDs.stripLength * 4,
                            LEDs.candleLength
                        ), 0
                    )
                }
            },
            {
                for (i in 0 until candle!!.maxSimultaneousAnimationCount) candle.clearAnimation(i)
            },
            leds
        )
    }

    fun ledsUp(leds: LEDs?): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle

        return Commands.runEnd(
            {
                candle!!.setLEDs(0, 0, 0, 0, LEDs.candleLength, LEDs.stripLength / 2) // up half 1
                candle.setLEDs(
                    100,
                    100,
                    100,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 1 - LEDs.stripLength / 2,
                    LEDs.stripLength
                ) // up half 2 down half 2
                candle.setLEDs(
                    0,
                    0,
                    0,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 2 - LEDs.stripLength / 2,
                    LEDs.stripLength
                ) // down half 1 up half 1
                candle.setLEDs(
                    100,
                    100,
                    100,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 3 - LEDs.stripLength / 2,
                    LEDs.stripLength
                ) // up half 2 down half 2
                candle.setLEDs(
                    0,
                    0,
                    0,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 4 - LEDs.stripLength / 2,
                    LEDs.stripLength / 2
                ) // down half 1
            },
            {
                for (i in 0 until candle!!.maxSimultaneousAnimationCount) candle.clearAnimation(i)
            },
            leds
        )
    }

    fun ledsDown(leds: LEDs?): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle

        return Commands.runEnd(
            {
                candle!!.setLEDs(
                    100,
                    100,
                    100,
                    0,
                    LEDs.candleLength,
                    LEDs.stripLength / 2
                ) // up half 1
                candle.setLEDs(
                    0,
                    0,
                    0,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 1 - LEDs.stripLength / 2,
                    LEDs.stripLength
                ) // up half 2 down half 2
                candle.setLEDs(
                    100,
                    100,
                    100,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 2 - LEDs.stripLength / 2,
                    LEDs.stripLength
                ) // down half 1 up half 1
                candle.setLEDs(
                    0,
                    0,
                    0,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 3 - LEDs.stripLength / 2,
                    LEDs.stripLength
                ) // up half 2 down half 2
                candle.setLEDs(
                    100,
                    100,
                    100,
                    0,
                    LEDs.candleLength + LEDs.stripLength * 4 - LEDs.stripLength / 2,
                    LEDs.stripLength / 2
                ) // down half 1
            },
            {
                for (i in 0 until candle!!.maxSimultaneousAnimationCount) candle.clearAnimation(i)
            },
            leds
        )
    }

    fun dropNoteEmily(leds: LEDs?): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle

        return Commands.startEnd(
            {
                candle!!.animate(
                    LarsonAnimation(
                        255,
                        255,
                        50,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Back,
                        LEDs.stripLength,
                        LEDs.candleLength
                    ), 0
                )
                candle.animate(
                    LarsonAnimation(
                        255,
                        255,
                        50,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Front,
                        LEDs.stripLength,
                        LEDs.candleLength + LEDs.stripLength
                    ), 1
                )
                candle.animate(
                    LarsonAnimation(
                        255,
                        255,
                        50,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Back,
                        LEDs.stripLength,
                        LEDs.candleLength + LEDs.stripLength * 2
                    ), 2
                )
                candle.animate(
                    LarsonAnimation(
                        255,
                        255,
                        50,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Front,
                        LEDs.stripLength,
                        LEDs.candleLength + LEDs.stripLength * 3
                    ), 3
                )
            },
            {
                for (i in 0 until candle!!.maxSimultaneousAnimationCount) candle.clearAnimation(i)
            },
            leds
        )
    }

    fun hasNote(leds: LEDs?): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle

        return Commands.startEnd(
            {
                candle!!.animate(
                    LarsonAnimation(
                        255,
                        165,
                        0,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Back,
                        LEDs.stripLength,
                        LEDs.candleLength
                    ), 0
                )
                candle.animate(
                    LarsonAnimation(
                        255,
                        165,
                        0,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Front,
                        LEDs.stripLength,
                        LEDs.candleLength + LEDs.stripLength
                    ), 1
                )
                candle.animate(
                    LarsonAnimation(
                        255,
                        165,
                        0,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Back,
                        LEDs.stripLength,
                        LEDs.candleLength + LEDs.stripLength * 2
                    ), 2
                )
                candle.animate(
                    LarsonAnimation(
                        255,
                        165,
                        0,
                        0,
                        0.0,
                        LEDs.stripLength,
                        LarsonAnimation.BounceMode.Front,
                        LEDs.stripLength,
                        LEDs.candleLength + LEDs.stripLength * 3
                    ), 3
                )
            },
            {
                for (i in 0 until candle!!.maxSimultaneousAnimationCount) candle.clearAnimation(i)
            },
            leds
        )
    }

    fun flameCommand(leds: LEDs?): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        return flameCommand(leds, 0.25)
    }

    fun disabled(leds: LEDs?, robotContainer: RobotContainer): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle

        return Commands.startEnd(
            {
                candle!!.animate(
                    RainbowAnimation(
                        1.0,
                        0.0,
                        LEDs.stripLength * 4,
                        false,
                        LEDs.candleLength
                    ), 1
                )
            },
            {
                for (i in 0 until candle!!.maxSimultaneousAnimationCount) candle.clearAnimation(i)
            },
            leds
        ).deadlineWith(Commands.run({
            val dsAttached = DriverStation.isDSAttached()
            val cameraConnected = robotContainer.drive!!.cameraCount() > 1
            val gyroConnected = robotContainer.drive!!.isGyroConnected
            if (cameraConnected) {
                candle!!.clearAnimation(2)
                candle.setLEDs(0, 100, 0, 0, 0, 1)
            } else candle!!.animate(StrobeAnimation(255, 0, 0, 0, 0.0, 1, 0), 2)

            val side = DriverStation.getAlliance()
            if (side.isEmpty) {
                candle.animate(StrobeAnimation(50, 50, 50, 0, 0.0, 1, 1), 3) //blinkred
            } else if (side.get() == Alliance.Blue) {
                candle.animate(SingleFadeAnimation(0, 0, 100, 0, 0.0, 1, 1), 3)
            } else if (side.get() == Alliance.Red) {
                candle.animate(SingleFadeAnimation(100, 0, 0, 0, 0.0, 1, 1), 3)
            } else {
                candle.animate(StrobeAnimation(100, 0, 0, 0, 0.0, 1, 1), 3) //blinkred
            }

            if (dsAttached) {
                candle.clearAnimation(4)
                candle.setLEDs(0, 100, 0, 0, 2, 1)
            } else candle.animate(StrobeAnimation(100, 0, 0, 0, 0.0, 1, 2), 4)

            if (gyroConnected) {
                candle.clearAnimation(5)
                candle.setLEDs(0, 100, 0, 0, 3, 1)
            } else candle.animate(StrobeAnimation(100, 0, 0, 0, 0.0, 1, 3), 5)

            if (RobotController.getBatteryVoltage() > 13.0) {
                candle.clearAnimation(5)
                candle.setLEDs(0, 100, 0, 0, 4, 1)
            } else candle.animate(StrobeAnimation(100, 0, 0, 0, 0.0, 1, 4), 5)
            if (DriverStation.isFMSAttached()) {
                candle.clearAnimation(6)
                candle.setLEDs(0, 100, 0, 0, 5, 1)
            } else candle.animate(StrobeAnimation(100, 0, 0, 0, 0.0, 1, 5), 6)
        })).handleInterrupt {
            for (i in 0 until candle!!.maxSimultaneousAnimationCount) {
                candle.clearAnimation(i)
            }
        }
    }

    fun enabled(leds: LEDs?): Command {
        if (leds == null) return Commands.none()
        if (leds.candle == null) return Commands.idle(leds)
        val candle = leds.candle
        return Commands.startEnd(
            {
                candle!!.animate(RgbFadeAnimation(1.0, 0.5, LEDs.candleLength, 0), 0)
                candle.animate(
                    TwinkleAnimation(
                        200,
                        200,
                        200,
                        1,
                        0.0,
                        LEDs.stripLength * 4,
                        TwinklePercent.Percent30,
                        LEDs.candleLength
                    ), 1
                )
            },
            {
                for (i in 0 until candle!!.maxSimultaneousAnimationCount) candle.clearAnimation(i)
            },
            leds
        )
    }
}
