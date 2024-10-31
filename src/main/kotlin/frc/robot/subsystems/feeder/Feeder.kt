package frc.robot.subsystems.feeder

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.filter.Debouncer.DebounceType
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Feeder(private val io: FeederIO) : SubsystemBase() {
    private val inputs: FeederIOInputsAutoLogged = FeederIOInputsAutoLogged()
    private var ffModel: SimpleMotorFeedforward? = null
    private var pidController: ProfiledPIDController? = null
    private var offset: Double = 0.0

    var state: State = State.none

    enum class State {
        feedingShooter,
        intaking,
        zeroingNote,
        humanPlayerIntake,
        none
    }

    private var debouncer: Debouncer = Debouncer(.05, DebounceType.kBoth)
    private var debouncer2: Debouncer = Debouncer(.05, DebounceType.kBoth)

    @get:AutoLogOutput
    val beamBroken: Boolean
        get() {
            return !debouncer.calculate(inputs.beamUnobstructed)
        }

    @get:AutoLogOutput
    val intakeBeamBroken: Boolean
        get() {
            return !debouncer2.calculate(inputs.intakebeamUnobstructed)
        }

    init {
        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        when (Constants.currentMode) {
            Constants.Mode.REAL, Constants.Mode.REPLAY -> {
                ffModel = SimpleMotorFeedforward(0.1, 0.05)
                pidController =
                    ProfiledPIDController(
                        1.0,
                        0.0,
                        0.0,
                        TrapezoidProfile.Constraints(10.0, 10.0)
                    ) // fixme: tune velocity and acceleration
            }

            Constants.Mode.SIM -> {
                ffModel = SimpleMotorFeedforward(0.0, 0.03)
                pidController =
                    ProfiledPIDController(
                        1.0,
                        0.0,
                        0.0,
                        TrapezoidProfile.Constraints(10.0, 10.0)
                    ) // fixme: tune velocity and acceleration
            }

            else -> {
                ffModel = SimpleMotorFeedforward(0.0, 0.0)
                pidController =
                    ProfiledPIDController(0.0, 0.0, .0, TrapezoidProfile.Constraints(0.0, 0.0))
            }
        }
    }

    override fun periodic() {
        if (this.currentCommand != null) {
            Logger.recordOutput("Commands/Feeder", currentCommand.name)
        } else {
            Logger.recordOutput("Commands/Feeder", "")
        }
        io.updateInputs(inputs)
        //    io.setVoltage(
//        pidController.calculate(inputs.positionRad)
//            + ffModel.calculate(pidController.getSetpoint().velocity));
        Logger.processInputs("Flywheel", inputs)
    }

    /** Run open loop at the specified voltage.  */
    fun runVolts(volts: Double) {
        io.setVoltage(volts)
    }

    /**
     * Run open loop at the specified voltage.
     */
    fun runVolts(volts: Measure<Voltage?>) {
        runVolts(volts.`in`(edu.wpi.first.units.Units.Volts))
    }

    val characterizationVoltage: Measure<Voltage>
        /**
         * Run open loop at the specified voltage.
         */
        get() {
            return edu.wpi.first.units.Units.Volts.of(inputs.appliedVolts)
        }

    val characterizationCurrent: Measure<Current>
        /**
         * Run open loop at the specified voltage.
         */
        get() {
            var sum = 0.0
            for (i in inputs.currentAmps.indices.reversed()) {
                sum += inputs.currentAmps.get(i)
            }
            return if (inputs.currentAmps.size != 0) {
                edu.wpi.first.units.Units.Amps.of(sum / inputs.currentAmps.size)
            } else edu.wpi.first.units.Units.Amps.zero()
        }

    val characterizationPosition: Measure<Angle>
        get() {
            return edu.wpi.first.units.Units.Radians.of(inputs.positionRad)
        }

    val characterizationVelocity: Measure<Velocity<Angle>>
        get() {
            return edu.wpi.first.units.Units.RadiansPerSecond.of(inputs.velocityRadPerSec)
        }

    /** Run closed loop to the specified position.  */
    fun runPosition(position: Double) {
        pidController!!.setGoal(position + offset)
        // Log flywheel setpoint
        Logger.recordOutput("Flywheel/SetpointRPM", position)
    }

    fun resetPosition() {
        val oldGoal: TrapezoidProfile.State =
            TrapezoidProfile.State(
                pidController!!.goal.position - offset, pidController.getGoal().velocity
            )
        offset = inputs.positionRad
        pidController.setGoal(TrapezoidProfile.State(oldGoal.position + offset, oldGoal.velocity))
    }

    /** Stops the flywheel.  */
    fun stop() {
        io.stop()
    }

    @get:AutoLogOutput
    val velocityRPM: Double
        /** Returns the current velocity in RPM.  */
        get() {
            return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec)
        }
}
