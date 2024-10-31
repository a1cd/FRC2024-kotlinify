// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.util.PolynomialRegression
import java.util.*
import java.util.function.Consumer
import java.util.function.Supplier
import kotlin.math.abs

class FeedForwardCharacterization(
    subsystem: Subsystem,
    voltageConsumer: Consumer<Double>,
    velocitySupplier: Supplier<Double>
) :
    Command() {
    private var data: FeedForwardCharacterizationData? = null
    private val voltageConsumer: Consumer<Double>
    private val velocitySupplier: Supplier<Double>

    private val timer = Timer()

    /** Creates a new FeedForwardCharacterization command.  */
    init {
        addRequirements(subsystem)
        this.voltageConsumer = voltageConsumer
        this.velocitySupplier = velocitySupplier
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        data = FeedForwardCharacterizationData()
        timer.reset()
        timer.start()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (timer.get() < START_DELAY_SECS) {
            voltageConsumer.accept(0.0)
        } else {
            val voltage = (timer.get() - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC
            voltageConsumer.accept(voltage)
            data!!.add(velocitySupplier.get(), voltage)
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        voltageConsumer.accept(0.0)
        timer.stop()
        data!!.print()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }

    class FeedForwardCharacterizationData {
        private val velocityData: MutableList<Double> = LinkedList()
        private val voltageData: MutableList<Double> = LinkedList()

        fun add(velocity: Double, voltage: Double) {
            if (abs(velocity) > 1E-4) {
                velocityData.add(abs(velocity))
                voltageData.add(abs(voltage))
            }
        }

        fun print() {
            if (velocityData.size == 0 || voltageData.size == 0) {
                return
            }

            val regression =
                PolynomialRegression(
                    velocityData.stream().mapToDouble { obj: Double -> obj }.toArray(),
                    voltageData.stream().mapToDouble { obj: Double -> obj }.toArray(),
                    1
                )

            println("FF Characterization Results:")
            println("\tCount=" + velocityData.size.toString() + "")
            println(String.format("\tR2=%.5f", regression.R2()))
            println(String.format("\tkS=%.5f", regression.beta(0)))
            println(String.format("\tkV=%.5f", regression.beta(1)))
        }
    }

    companion object {
        private const val START_DELAY_SECS = 2.0
        private const val RAMP_VOLTS_PER_SEC = 0.1
    }
}
