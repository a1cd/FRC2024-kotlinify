package frc.robot.subsystems.intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Intake(var io: IntakeIO) : SubsystemBase() {
    private val ligament1: MechanismLigament2d
    private val ligament1A: MechanismLigament2d
    private val ligament2: MechanismLigament2d
    private val ligament2A: MechanismLigament2d
    private val inputs = IntakeIOInputsAutoLogged()
    private var armFB: ProfiledPIDController = ProfiledPIDController(
        7.0,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(
            Units.RadiansPerSecond.of(18.0),
            Units.RadiansPerSecond.per(Units.Second).of(240.0)
        )
    )
    private var armFF: ArmFeedforward = ArmFeedforward(0.0, 0.0, 0.0, 0.0)
    private var mechanism2d: Mechanism2d = Mechanism2d(0.0, 0.0)

    // double position = 0.0;
    private var armTarget: Rotation2d = Rotation2d.fromDegrees(-90.0)

    private var off1: Rotation2d = Rotation2d(1.2466477)
    private var off1A: Rotation2d = Rotation2d(0.313352305)
    private var off2: Rotation2d = Rotation2d(1.36216528)
    private var off2A: Rotation2d = Rotation2d(0.154247715)
    private var quarterTurn: Rotation2d = Rotation2d.fromRadians(Math.PI / 2)

    private var mustReset: Boolean = true
    private var rollerVoltageSetpoint = 0.0

    private fun resetArmFB() {
        armFB.reset(
            TrapezoidProfile.State(
                Units.Radians.of(inputs.armPositionRad),
                Units.RadiansPerSecond.of(inputs.armVelocityRadPerSec)
            )
        )
    }

    fun setIntakePosition(position: Rotation2d) {
        armTarget = position
    }

    @AutoLogOutput
    fun feedControlDisabled(): Boolean {
        return disableFeedControl
    }

    fun setFeedControl(disabled: Boolean) {
        this.disableFeedControl = disabled
    }

    private var disableFeedControl: Boolean = false

    init {
        val root = mechanism2d.getRoot("Root", .305, .220)
        armFB.enableContinuousInput(-Math.PI, Math.PI)
        armFB.setTolerance(0.025)
        ligament1 = MechanismLigament2d("Intake", .135, off1.degrees, .1, Color8Bit(1, 1, 1))
        ligament1A = MechanismLigament2d("Intake", 0.232427, off1A.minus(quarterTurn).degrees, .5, Color8Bit(1, 1, 1))
        ligament2 = MechanismLigament2d("Intake2", .227, off2.degrees, .1, Color8Bit(1, 1, 1))
        ligament2A = MechanismLigament2d("Intake2", 0.232983, off2A.minus(quarterTurn).degrees, .5, Color8Bit(1, 1, 1))
        root.append(ligament1).append(ligament1A)
        root.append(ligament2).append(ligament2A)

        SmartDashboard.putNumber("IntakePIDD", armFB.d)
        SmartDashboard.putNumber("IntakePIDP", armFB.p)
    }


    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
        if (this.currentCommand != null) {
            Logger.recordOutput("Commands/Intake", this.currentCommand.name)
        } else {
            Logger.recordOutput("Commands/Intake", "")
        }
        if (inputs.armPositionRad < 1.715 && inputs.armPositionRad > -0.97) {
            io.setRollerPercent(0.0)
        }
        if (mustReset) {
            resetArmFB()
            mustReset = false
        }
        if (!disableFeedControl) {
            if (RobotController.isSysActive()) io.setArmVoltage(
                armFB.calculate(inputs.armPositionRad, MathUtil.angleModulus(armTarget.radians))
                        + armFF.calculate(armFB.setpoint.position, armFB.setpoint.velocity)
            )
            else io.setArmVoltage(0.0)
        }

        val rotation2d = Rotation2d(inputs.armPositionRad)
        ligament1.setAngle(
            rotation2d
                .plus(off1)
                .minus(quarterTurn)
                .times(-1.0)
        )
        ligament2.setAngle(
            rotation2d
                .plus(off2)
                .minus(quarterTurn)
                .times(-1.0)
        )
        Logger.recordOutput("Intake", mechanism2d)

        io.setRollerVoltage(rollerVoltageSetpoint /* * ((inputs.armPositionRad > -4.5) ? -1 : 1)*/)
    }

    fun setRollerVoltage(voltage: Double) {
        rollerVoltageSetpoint = voltage
    }

    val armCharacterizationVoltage: Measure<Voltage>
        get() = Units.Volts.of(inputs.armAppliedVolts)

    val armCharacterizationPosition: Measure<Angle>
        get() = Units.Radians.of(inputs.armPositionRad)

    val armCharacterizationVelocity: Measure<Velocity<Angle>>
        get() = Units.RadiansPerSecond.of(inputs.armVelocityRadPerSec)

    val armCharacterizationCurrent: Measure<Current>
        /**
         * Run open loop at the specified voltage.
         */
        get() {
            var sum = 0.0
            for (i in inputs.rollerCurrentAmps.indices.reversed()) {
                sum += inputs.rollerCurrentAmps[i]
            }
            return if (inputs.rollerCurrentAmps.size != 0) {
                Units.Amps.of(sum / inputs.rollerCurrentAmps.size)
            } else Units.Amps.zero()
        }

    fun runArmVolts(voltageMeasure: Measure<Voltage?>) {
        io.setArmVoltage(voltageMeasure.`in`(Units.Volts))
    }
}
