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
package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.*
import frc.robot.Constants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber
import kotlin.math.cos
import kotlin.math.min

class Module(private val io: ModuleIO, val index: Int) {
    private val pPidRot: LoggedDashboardNumber = LoggedDashboardNumber("Drive/Module/Rot P")
    private val dPidRot: LoggedDashboardNumber = LoggedDashboardNumber("Drive/Module/Rot D")

    private val inputs: ModuleIOInputsAutoLogged = ModuleIOInputsAutoLogged()

    private var driveFeedforward: SimpleMotorFeedforward? = null
    private var driveFeedback: PIDController? = null
    private var turnFeedback: PIDController? = null
    private var angleSetpoint: Rotation2d? = null // Setpoint for closed loop control, null for open loop
    private var speedSetpoint: Double? = null // Setpoint for closed loop control, null for open loop
    private var turnRelativeOffset: Rotation2d? = null // Relative + Offset = Absolute
    private var lastPositionMeters: Double = 0.0 // Used for delta calculation

    private var lastDriveVelocity: Measure<Velocity<Angle>> = Units.RadiansPerSecond.zero()
    private var lastTime: Measure<Time> = Units.Microsecond.of(Logger.getTimestamp().toDouble()).minus(Units.Seconds.of(0.02))

    init {
        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                when (index) {
                    0 -> {
                        driveFeedforward = SimpleMotorFeedforward(0.039527, 0.13437, 0.12428)
                        driveFeedback = PIDController(0.15254, 0.0, 0.0)
                    }

                    1 -> {
                        driveFeedforward = SimpleMotorFeedforward(0.024784, 0.13088, 0.1558 / 6.2831)
                        driveFeedback = PIDController(0.1295, 0.0, 0.0)
                    }

                    2 -> {
                        driveFeedforward = SimpleMotorFeedforward(0.077976, 0.13341, 0.13853 / 6.2831)
                        driveFeedback = PIDController(0.13535, 0.0, 0.0)
                    }

                    3 -> {
                        driveFeedforward = SimpleMotorFeedforward(0.077976, 0.12927, 0.1631 / 6.2831)
                        driveFeedback = PIDController(0.10222, 0.0, 0.0)
                    }

                    else -> {
                        driveFeedforward = SimpleMotorFeedforward(.175, 0.127, .13)
                        driveFeedback = PIDController(0.15254, 0.0, 0.0)
                    }
                }
                //        driveFeedback = new PIDController(0.0097924, 0.0, 0.0);//fixme: try commenting/uncommenting this line: it overrides the previous ones
                turnFeedback = PIDController(5.0, 0.0, .02)
            }

            Constants.Mode.REPLAY -> {
                driveFeedforward = SimpleMotorFeedforward(0.1, 0.13)
                driveFeedback = PIDController(0.05, 0.0, 0.0)
                turnFeedback = PIDController(7.0, 0.0, 0.0)
            }

            Constants.Mode.SIM -> {
                driveFeedforward = SimpleMotorFeedforward(0.0, 0.13)
                driveFeedback = PIDController(0.1, 0.0, 0.0)
                turnFeedback = PIDController(10.0, 0.0, 0.0)
            }

            else -> {
                driveFeedforward = SimpleMotorFeedforward(0.0, 0.0)
                driveFeedback = PIDController(0.0, 0.0, 0.0)
                turnFeedback = PIDController(0.0, 0.0, 0.0)
            }
        }

        dPidRot.setDefault(turnFeedback.getD())
        pPidRot.setDefault(turnFeedback.getP())

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI)
        setBrakeMode(true)
    }

    fun periodic() {
        io.updateInputs(inputs)
        val time: Measure<Time> = Units.Microsecond.of(Logger.getTimestamp().toDouble())
        val currentVelocity: Measure<Velocity<Angle>> = Units.RadiansPerSecond.of(inputs.driveVelocityRadPerSec)
        val acceleration: Measure<Velocity<Velocity<Angle>>> =
            (lastDriveVelocity.minus(currentVelocity)).per(lastTime.minus(time))
        val maxAcheivableAcceleration: Measure<Velocity<Velocity<Angle>>> =
            Units.RadiansPerSecond.per(Units.Seconds).of(
                driveFeedforward!!.maxAchievableAcceleration(
                    inputs.driveAppliedVolts,
                    lastDriveVelocity.`in`(Units.RadiansPerSecond)
                )
            )
        val freeSpinningAmount: Double =
            min(acceleration.baseUnitMagnitude() / (maxAcheivableAcceleration.baseUnitMagnitude()), 1.0)
        Logger.recordOutput("Drive/Module$index/Max Achievable Acceleration", maxAcheivableAcceleration)
        Logger.recordOutput("Drive/Module$index/Acceleration", acceleration)
        Logger.recordOutput("Drive/Module$index/Free Spinning Amount", freeSpinningAmount)
        lastDriveVelocity = currentVelocity
        lastTime = time
        //    turnFeedback.setP(pPidRot.get());
//    turnFeedback.setD(dPidRot.get());
        Logger.processInputs("Drive/Module$index", inputs)

        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition)
        }

        // Run closed loop turn control
        if (angleSetpoint != null) {
            io.setTurnVoltage(
                turnFeedback!!.calculate(inputs.turnAbsolutePosition.getRadians(), angleSetpoint!!.radians)
            )

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
                // Scale velocity based on turn error
                //
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                val adjustSpeedSetpoint: Double = speedSetpoint!! * cos(turnFeedback.getPositionError())

                // Run drive controller
                val velocityRadPerSec: Double = adjustSpeedSetpoint / WHEEL_RADIUS
                val feedForwardValue: Double = driveFeedforward.calculate(velocityRadPerSec)
                val feedBackValue: Double = driveFeedback!!.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)
                val sum: Double = feedBackValue + feedForwardValue
                io.setDriveVoltage(sum)
                Logger.recordOutput("Drive/Module $index/Drive/FF Value", feedForwardValue)
                Logger.recordOutput("Drive/Module $index/Drive/FB Value", feedBackValue)
                Logger.recordOutput("Drive/Module $index/Drive/Voltage Sum", sum)
                Logger.recordOutput("Drive/Module $index/Drive/feedback Setpoint", driveFeedback.getSetpoint())
                Logger.recordOutput(
                    "Drive/Module $index/Drive/feedback Position Error",
                    driveFeedback.getPositionError()
                )
                Logger.recordOutput(
                    "Drive/Module $index/Drive/feedback Velocity Error",
                    driveFeedback.getVelocityError()
                )
            }
        }
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state.  */
    fun runSetpoint(state: SwerveModuleState): SwerveModuleState {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        val optimizedState: SwerveModuleState = SwerveModuleState.optimize(state, angle)

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedState.angle
        speedSetpoint = optimizedState.speedMetersPerSecond

        return optimizedState
    }

    /** Runs the module with the specified voltage while controlling to zero degrees.  */
    fun runCharacterization(volts: Double) {
        // Closed loop turn control
        angleSetpoint = Rotation2d()

        // Open loop drive control
        io.setDriveVoltage(volts)
        Logger.recordOutput("Voltage for motor" + this.index, volts)
        speedSetpoint = null
    }

    /** Disables all outputs to motors.  */
    fun stop() {
        io.setTurnVoltage(0.0)
        io.setDriveVoltage(0.0)

        // Disable closed loop control for turn and drive
        angleSetpoint = null
        speedSetpoint = null
    }

    /** Sets whether brake mode is enabled.  */
    private fun setBrakeMode(enabled: Boolean) {
        io.setDriveBrakeMode(enabled)
        io.setTurnBrakeMode(enabled)
    }

    val angle: Rotation2d
        /** Returns the current turn angle of the module.  */
        get() {
            return if (turnRelativeOffset == null) {
                Rotation2d()
            } else {
                inputs.turnPosition.plus(turnRelativeOffset)
            }
        }

    private val positionMeters: Double
        /** Returns the current drive position of the module in meters.  */
        get() {
            return inputs.drivePositionRad * WHEEL_RADIUS
        }

    private val velocityMetersPerSec: Double
        /** Returns the current drive velocity of the module in meters per second.  */
        get() {
            return inputs.driveVelocityRadPerSec * WHEEL_RADIUS
        }

    val position: SwerveModulePosition
        /** Returns the module position (turn angle and drive position).  */
        get() {
            return SwerveModulePosition(positionMeters, angle)
        }

    val positionDelta: SwerveModulePosition
        /** Returns the module position delta since the last call to this method.  */
        get() {
            val delta: SwerveModulePosition = SwerveModulePosition(positionMeters - lastPositionMeters, angle)
            lastPositionMeters = positionMeters
            return delta
        }

    val state: SwerveModuleState
        /** Returns the module state (turn angle and drive velocity).  */
        get() {
            return SwerveModuleState(velocityMetersPerSec, angle)
        }

    val characterizationVelocityRadPerSec: Double
        /** Returns the drive velocity in radians/sec.  */
        get() {
            return inputs.driveVelocityRadPerSec
        }

    val characterizationDrivePosition: Measure<Angle>
        /** Returns the drive velocity unitless.  */
        get() {
            return Units.Radians.of(inputs.drivePositionRad)
        }
    val characterizationTurnPosition: Measure<Angle>
        /** Returns the turn velocity unitless.  */
        get() {
            return Units.Radians.of(inputs.turnPosition.getRadians())
        }
    val characterizationDriveVelocity: Measure<Velocity<Angle>>
        /** Returns the drive velocity unitless.  */
        get() {
            return Units.RadiansPerSecond.of(inputs.driveVelocityRadPerSec)
        }
    val characterizationTurnVelocity: Measure<Velocity<Angle>>
        /** Returns the turn velocity unitless.  */
        get() {
            return Units.RadiansPerSecond.of(inputs.turnVelocityRadPerSec)
        }

    companion object {
        private const val WHEEL_RADIUS: Double = 0.057
    }
}
