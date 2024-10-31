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
package frc.robot.subsystems.shooter

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.RobotState
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class Shooter(private val shooterIO: ShooterIO, private val hoodIO: HoodIO) : SubsystemBase() {
    private val shooterInputs = ShooterIOInputsAutoLogged()
    private val hoodInputs = HoodIOInputsAutoLogged()
    private var shooterVelocityFB: PIDController? = null
    private var setpointRadPS: Double = 0.0
    private var hoodFB: ProfiledPIDController? = null
    private var shooterVelocityFF: SimpleMotorFeedforward? = null
    private var characterizeMode = false
    private var mech1: Mechanism2d = Mechanism2d(3.0, 3.0)
    var root: MechanismRoot2d = mech1.getRoot("shooter", 0.0, 0.0)
    private var sState: MechanismLigament2d = root.append(MechanismLigament2d("shooter", 0.14, -90.0))

    private var hoodOffsetAngle = Rotation2d()

    private var hasZeroed = false

    fun hasZeroed(): Boolean {
        return hasZeroed
    }

    fun setHasZeroed(hasZeroed: Boolean) {
        this.hasZeroed = hasZeroed
    }

    val limitSwitch: Boolean
        get() = hoodInputs.islimitSwitchPressed

    private var hasReset = false
    private var hoodPIDEnabled = true

    fun setHoodPIDEnabled(hoodPIDEnabled: Boolean) {
        this.hoodPIDEnabled = hoodPIDEnabled
    }

    private var previousAnglularVelocity: Double = 0.0

    private var lastLimitSwitch = true

    var zeroMode: Boolean = false

    override fun periodic() {
        if (this.currentCommand != null) {
            Logger.recordOutput("Commands/Shooter", this.currentCommand.name)
        } else {
            Logger.recordOutput("Commands/Shooter", "")
        }
        shooterIO.updateInputs(shooterInputs)
        Logger.processInputs("Shooter/Flywheel", shooterInputs)

        hoodIO.updateInputs(hoodInputs)
        Logger.processInputs("Shooter/Hood", hoodInputs)


        if (!hasReset) {
            resetToStartingAngle()
            hoodFB!!.reset(hoodInputs.motorPositionRad - hoodOffsetAngle.radians)
            hasReset = true
        }

        if (!characterizeMode || setpointRadPS != 0.0) {
            if (setpointRadPS == 0.0) shooterIO.setFlywheelVoltage(0.0)
            else shooterIO.setFlywheelVoltage(
                shooterVelocityFB!!.calculate(shooterInputs.flywheelVelocityRadPerSec, setpointRadPS)
                        + shooterVelocityFF!!.calculate(shooterVelocityFB.setpoint)
            )
        } else if (setpointRadPS == 0.0) shooterIO.setFlywheelVoltage(0.0)
        val currentCommand = currentCommand
        if (currentCommand != null) Logger.recordOutput("Shooter/Current Command", currentCommand.name)
        else Logger.recordOutput("Shooter/Current Command", "null")
        Logger.recordOutput("Shooter/Shooter Speed", setpointRadPS)
        Logger.recordOutput("Shooter/Hood/Target Hood Angle", targetHoodAngleRad)
        Logger.recordOutput(
            "Shooter/Hood/Inputs/Offset Motor Position Radians",
            hoodInputs.motorPositionRad - hoodOffsetAngle.radians
        )
        Logger.recordOutput("Shooter/Hood/Offset Radians", hoodOffsetAngle.radians)
        Logger.recordOutput("Shooter/Hood/Beam Break Status", hoodInputs.islimitSwitchPressed)

        previousAnglularVelocity = hoodInputs.hoodVelocityRadPerSec
        if (previousAnglularVelocity != 0.0) Logger.recordOutput(
            "Shooter/Hood Acceleration",
            edu.wpi.first.units.Units.RadiansPerSecond.of(previousAnglularVelocity - hoodInputs.hoodVelocityRadPerSec)
                .per(edu.wpi.first.units.Units.Seconds.of(0.02))
        )
        if (hoodPIDEnabled) {
            hoodIO.setVoltage(
                hoodFB!!.calculate(hoodInputs.motorPositionRad - hoodOffsetAngle.radians, targetHoodAngleRad)
            )
        }
        sState.angle = hoodInputs.hoodPositionRad
        Logger.recordOutput("Shooter/Mechanism", mech1)

        // Let the hood move more easily while disabled so that we don't skip gears as much
        hoodIO.setBrakeMode(RobotState.isDisabled() && (abs(shooterInputs.flywheelVelocityRadPerSec) < 1000))

        if (hoodInputs.islimitSwitchPressed != lastLimitSwitch) {
            lastLimitSwitch = hoodInputs.islimitSwitchPressed
            // only do it during these times to prevent arbitrary zero events
            if (zeroMode || RobotState.isDisabled()) resetToLimitAngle()
        }
    }

    private fun resetToLimitAngle() {
        hoodOffsetAngle = Rotation2d(hoodInputs.motorPositionRad - (1.98875))
        hoodFB!!.reset(hoodInputs.motorPositionRad - (1.98875 + .16))
    }

    private fun resetToStartingAngle() {
        hoodOffsetAngle = Rotation2d(hoodInputs.motorPositionRad - 1.98542)
        hoodFB!!.reset(hoodInputs.motorPositionRad - hoodOffsetAngle.radians)
    }

    fun resetWhileZeroing() {
        hoodOffsetAngle = Rotation2d(hoodInputs.motorPositionRad - (2.225))
        hoodFB!!.reset(hoodInputs.motorPositionRad - hoodOffsetAngle.radians)
    }

    private var hoodOverride: Boolean = true

    /**
     * Creates a new Shooter.
     */
    init {
        sState.length = 0.14 //im not sure if this should be here

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                hoodFB = ProfiledPIDController(6.0, 0.0, .25, TrapezoidProfile.Constraints(1000.0 / 2.0, 7600.0 / 32.0))
                hoodFB.setTolerance(0.025)
                shooterVelocityFB =
                    PIDController(0.0079065 * 5, 0.0015, 0.0)
                shooterVelocityFB.setIZone(2.0)
                shooterVelocityFB.setTolerance(218.69 * .25) // this is the pid max velocity error (rad/sec)
                shooterVelocityFF = SimpleMotorFeedforward(.58287, .013052, .0038592)
            }

            Constants.Mode.REPLAY -> {
                hoodFB = ProfiledPIDController(4.0, 0.0, 0.0, TrapezoidProfile.Constraints(1.0, 2.0))

                shooterVelocityFB =
                    PIDController(0.0050812, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/)
                shooterVelocityFB.setTolerance(25.0)
                shooterVelocityFF = SimpleMotorFeedforward(0.10548, 0.11959, 0.066251)
            }

            Constants.Mode.SIM -> {
                shooterVelocityFB =
                    PIDController(0.5, 0.0, 0.0 /*, new TrapezoidProfile.Constraints(0.5, 99)*/)
                hoodFB = ProfiledPIDController(4.0, 0.0, 0.0, TrapezoidProfile.Constraints(1.0, 2.0))
                shooterVelocityFF = SimpleMotorFeedforward(0.0, 0.0)
            }

            else -> {}
        }
    }

    fun overrideHoodAtSetpoint(isAtSetpoint: Boolean) {
        hoodOverride = isAtSetpoint
    }

    val isStalled: Boolean
        get() = hoodInputs.isStalled

    @AutoLogOutput
    fun flywheelAtSetpoint(): Boolean {
        return shooterVelocityFB!!.atSetpoint()
    }

    @AutoLogOutput
    fun hoodAtSetpoint(): Boolean {
        return hoodFB!!.atGoal() && hoodOverride
    }

    @AutoLogOutput
    fun allAtSetpoint(): Boolean {
        return flywheelAtSetpoint() && hoodAtSetpoint()
    }

    fun setCharacterizeMode(on: Boolean) {
        characterizeMode = on
    }

    /**
     * Run open loop at the specified voltage.
     */
    fun shooterRunVolts(voltage: Measure<Voltage?>) {
        shooterIO.setFlywheelVoltage(voltage.`in`(edu.wpi.first.units.Units.Volts))
    }

    val characterizationAppliedVolts: Measure<Voltage>
        get() = edu.wpi.first.units.Units.Volts.of(shooterInputs.flywheelAppliedVolts)

    fun hoodRunVolts(volts: Double) {
        hoodIO.setVoltage(volts)
    }

    /**
     * Run closed loop at the specified velocity.
     */
    fun shooterRunVelocity(velocityRPM: Double) {
        val velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM)

        setpointRadPS = velocityRadPerSec

        // Log flywheel setpoint
        Logger.recordOutput("Shooter/SetpointRPM", velocityRPM)
    }

    /**
     * Stops the flywheel.
     */
    fun stopShooter() {
        shooterIO.flywheelStop()
    }

    fun stopHood() {
        hoodIO.wristStop()
    }

    @get:AutoLogOutput
    val shooterVelocityRPM: Double
        /**
         * Returns the current velocity in RPM.
         */
        get() = Units.radiansPerSecondToRotationsPerMinute(shooterInputs.flywheelVelocityRadPerSec)

    val characterizationVelocity: Measure<Velocity<Angle>>
        /**
         * Returns the current velocity in radians per second.
         */
        get() = edu.wpi.first.units.Units.RadiansPerSecond.of(shooterInputs.flywheelVelocityRadPerSec)

    val characterizationPosition: Measure<Angle>
        /**
         * Returns the current velocity in radians per second.
         */
        get() = edu.wpi.first.units.Units.Radians.of(shooterInputs.flywheelPositionRad)

    val characterizationCurrent: Measure<Current>
        /**
         * Returns the current velocity in radians per second.
         */
        get() {
            var sum = 0.0
            for (flywheelCurrentAmp in shooterInputs.flywheelCurrentAmps) sum += flywheelCurrentAmp

            sum =
                if ((shooterInputs.flywheelCurrentAmps.size > 0)) sum / shooterInputs.flywheelCurrentAmps.size else 0.0
            return edu.wpi.first.units.Units.Amps.of(sum)
        }

    fun setTargetShooterAngle(angle: Rotation2d) {
        targetHoodAngleRad = MathUtil.clamp(angle.radians, -2.0, 2.0)
    }

    val hoodCharacterizationVoltage: Measure<Voltage>
        get() = edu.wpi.first.units.Units.Volts.of(hoodInputs.hoodAppliedVolts)

    val hoodCharacterizationPosition: Measure<Angle>
        get() = edu.wpi.first.units.Units.Radians.of(hoodInputs.hoodPositionRad)

    val hoodCharacterizationVelocity: Measure<Velocity<Angle>>
        get() = edu.wpi.first.units.Units.RadiansPerSecond.of(hoodInputs.hoodVelocityRadPerSec)

    fun runHoodVoltage(voltageMeasure: Measure<Voltage?>) {
        hoodRunVolts(voltageMeasure.`in`(edu.wpi.first.units.Units.Volts))
    }

    companion object {
        // the ratio for turning the shooter
        //    private static final double TURN_SHOOTER_RATIO = 5.4;
        private var targetHoodAngleRad = 0.0
    }
}
