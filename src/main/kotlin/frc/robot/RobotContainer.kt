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
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.BaseUnits
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.RobotState
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.commands.*
import frc.robot.subsystems.ControllerRumble
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIO
import frc.robot.subsystems.climb.ClimbIOSim
import frc.robot.subsystems.climb.ClimbIOSparkMax
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.feeder.Feeder
import frc.robot.subsystems.feeder.FeederIO
import frc.robot.subsystems.feeder.FeederIOSim
import frc.robot.subsystems.feeder.FeederIOTalonFX
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIO
import frc.robot.subsystems.intake.IntakeIOSim
import frc.robot.subsystems.intake.IntakeIOSparkMax
import frc.robot.subsystems.lights.LEDs
import frc.robot.subsystems.shooter.*
import frc.robot.util.Mode
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    var shooter: Shooter? = null
    var feeder: Feeder? = null
    var intake: Intake? = null
    private var climb: Climb? = null

    //    private final Dashboard dashboard;
    private val leds: LEDs = LEDs()
    private val driverRumble = ControllerRumble(0)
    private val operatorRumble = ControllerRumble(1)

    // Controller
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)

    // Dashboard inputs
    private val autoChooser: LoggedDashboardChooser<Command>
    private val smartCommandsMode = LoggedDashboardChooser("Use Smart Commands", SendableChooser<SmartCommandsMode>())
    private val flywheelSpeedInput = LoggedDashboardNumber("Flywheel Speed", 1500.0)
    private val reactions: ReactionObject
    var angleOffsetInput: LoggedDashboardNumber = LoggedDashboardNumber("Angle Offset", 0.0)

    // Subsystems
    @JvmField
    var drive: Drive? = null
    private var invertX: LoggedDashboardBoolean = LoggedDashboardBoolean("Invert X Axis", false)
    private var invertY: LoggedDashboardBoolean = LoggedDashboardBoolean("Invert Y Axis", false)
    private var invertOmega: LoggedDashboardBoolean = LoggedDashboardBoolean("Invert Omega Axis", false)


    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    init {
        when (Constants.currentMode) {
            Constants.Mode.REAL -> {
                // Real robot, instantiate hardware IO implementations
                drive =
                    Drive(
                        GyroIOPigeon2(),
                        object : ModuleIOSparkMax(0) {},
                        object : ModuleIOSparkMax(1) {},
                        object : ModuleIOSparkMax(2) {},
                        object : ModuleIOSparkMax(3) {},
                        arrayOf(
                            VisionIOReal("ShootSideCamera"),
                            VisionIOReal("RightCamera")
                        ),
                        LimelightNoteDetection()
                    )
                shooter = Shooter(ShooterIOTalonFX(), HoodIOSparkMax())
                feeder = Feeder(FeederIOTalonFX())
                intake = Intake(IntakeIOSparkMax())
                climb = Climb(ClimbIOSparkMax())
            }

            Constants.Mode.SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                drive =
                    Drive(
                        object : GyroIO {},
                        VisionIOSim(
                            "ShootSideCamera"
                        ) { if ((drive == null)) (drive!!.pose) else Pose2d() },
                        ModuleIOSim(),
                        ModuleIOSim(),
                        ModuleIOSim(),
                        ModuleIOSim(),
                        object : LimelightNoteDetection() {})
                shooter = Shooter(ShooterIOSim(), object : HoodIO {})
                feeder = Feeder(FeederIOSim())
                intake = Intake(IntakeIOSim())
                climb = Climb(ClimbIOSim())
            }

            else -> {
                // Replayed robot, disable IO implementations
                drive =
                    Drive(
                        object : GyroIO {},
                        object : ModuleIO {},
                        object : ModuleIO {},
                        object : ModuleIO {},
                        object : ModuleIO {},
                        arrayOf(object : VisionIO {
                            override fun getCameraName(): String {
                                return "ShootSideCamera"
                            }
                        }, object : VisionIO {
                            override fun getCameraName(): String {
                                return "RightCamera"
                            }
                        }),
                        object : LimelightNoteDetection() {})
                shooter = Shooter(object : ShooterIO {
                }, object : HoodIO {
                })
                feeder = Feeder(object : FeederIO {
                })
                intake = Intake(object : IntakeIO {
                })
                climb = Climb(object : ClimbIO {
                })
            }
        }

        val command =
            DriveCommands.aimAtSpeakerCommand(
                drive,
                { driverController.leftY },
                { driverController.leftX },
                { driverController.rightX })

        NamedCommands.registerCommand(
            "Aim Drivetrain",
            command.command
        )
        NamedCommands.registerCommand(
            "Ready Shooter",
            ShooterCommands.autoAim(shooter, drive, feeder).asProxy()
        )
        NamedCommands.registerCommand(
            "Zero Feeder",
            FeederCommands.feedToBeamBreak(feeder).asProxy()
        )
        NamedCommands.registerCommand(
            "Zero Hood",
            ShooterCommands.simpleHoodZero(shooter).asProxy()
        )
        NamedCommands.registerCommand(
            "Auto Point",
            ShooterCommands.autoAim(shooter, drive, feeder).asProxy()
        )
        NamedCommands.registerCommand(
            "Shoot When Ready",
            Commands.sequence(
                Commands.waitUntil { feeder.beamBroken },
                Commands.sequence(
                    Commands.waitUntil { (shooter.allAtSetpoint() && (shooter.shooterVelocityRPM > 1000)) },
                    FeederCommands.feedToShooter(feeder)
                )
                    .onlyWhile { !feeder.beamBroken }
                    .withTimeout(3.0)
            ).raceWith(SpecializedCommands.timeoutDuringAutoSim(3.0))
        )
        NamedCommands.registerCommand(
            "Shoot",
            Commands.sequence(
                FeederCommands.feedToBeamBreak(feeder),
                Commands.waitUntil { (shooter.allAtSetpoint() && (shooter.shooterVelocityRPM > 1000)) },
                FeederCommands.feedToShooter(feeder)
            )
                .deadlineWith(ShooterCommands.JustShoot(shooter))
                .withTimeout(4.0).asProxy()
        )
        NamedCommands.registerCommand(
            "Intake Note",
            IntakeCommands.smartIntakeCommand(intake, feeder).raceWith(FeederCommands.feedToBeamBreak(feeder))
                .andThen(
                    Commands.either(
                        Commands.none(),
                        Commands.race(
                            FeederCommands.feedToBeamBreak(feeder).withTimeout(5.0),
                            IntakeCommands.flushIntakeWithoutTheArmExtendedOutward(intake, feeder)
                        )
                    ) { feeder.beamBroken }).withTimeout(3.0).asProxy()
        )
        NamedCommands.registerCommand(
            "Intake",
            IntakeCommands.intakeCommand(intake)
                .withTimeout(1.0)
        )
        NamedCommands.registerCommand(
            "Force Shoot",
            Commands.sequence(
                ShooterCommands.forceShoot(shooter),
                Commands.waitUntil { (shooter.allAtSetpoint() && (shooter.shooterVelocityRPM > 1000)) },
                Commands.run({ feeder.runVolts(8.0) }, feeder)
            ).asProxy()
        )
        autoChooser = LoggedDashboardChooser("Auto Choices", AutoBuilder.buildAutoChooser())
        //        dashboard = new Dashboard(autoChooser, drive, shooter, feeder, intake, vision, this.smartCommandsMode);
        this.reactions = ReactionObject(
            Trigger { feeder.intakeBeamBroken },
            Trigger { feeder.beamBroken },
            Trigger { RobotState.isTeleop() },
            Trigger { RobotState.isAutonomous() },
            Trigger { RobotState.isEnabled() }
        )

        configureButtonBindings()
        configureReactions()
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a [GenericHID] or one of its subclasses ([ ] or [XboxController]), and then passing it to a [ ].
     */
    private fun configureButtonBindings() {
        when (sysIDMode) {
            SysIDMode.Disabled -> {
                // ---- DEFAULT COMMANDS ----
                drive!!.defaultCommand = DriveCommands.joystickDrive(
                    drive,
                    { (-driverController.leftY * (if (invertX.get()) -1 else 1)) },
                    { (-driverController.leftX * (if (invertY.get()) -1 else 1)) },
                    { (-driverController.rightX) * (if (invertOmega.get()) -1 else 1) })
                intake!!.defaultCommand = IntakeCommands.idleCommand(intake)
                feeder!!.defaultCommand = FeederCommands.idleFeeder(feeder)
                shooter!!.defaultCommand = Commands.either(
                    ShooterCommands.shooterIdle(shooter),
                    Commands.sequence(
                        ShooterCommands.shooterIdle(shooter).until { shooter.hoodAtSetpoint() }.withTimeout(.5),
                        ShooterCommands.simpleHoodZero(shooter),
                        ShooterCommands.shooterIdle(shooter)
                    ).withName("Default Command")
                ) { shooter.hasZeroed() }
                // CLIMB DEFAULT COMMAND
                climb!!.defaultCommand = Commands.sequence(
                    ClimbCommands.zero(climb, 10.0).withTimeout(5.0),
                    ClimbCommands.runClimb(climb, { operatorController.leftY }, { operatorController.rightY })
                )

                leds.defaultCommand = Commands.either(
                    LEDCommands.enabled(leds), LEDCommands.disabled(
                        leds,
                        this
                    )
                ) { RobotState.isEnabled() }
                    .ignoringDisable(true)

                // ---- DRIVETRAIN COMMANDS ----
                driverController.x().whileTrue(Commands.runOnce({ drive!!.stopWithX() }, drive))

                val command =
                    DriveCommands.aimAtSpeakerCommand(
                        drive,
                        { (-driverController.leftY * (if (invertX.get()) -1 else 1)) },
                        { (-driverController.leftX * (if (invertY.get()) -1 else 1)) },
                        { (-driverController.rightX) * (if (invertOmega.get()) -1 else 1) })

                driverController.leftBumper().whileTrue(command.command)

                // ---- INTAKE COMMANDS ----
                driverController
                    .leftTrigger()
                    .whileTrue(
                        Commands.parallel(
                            IntakeCommands.smartIntakeCommand(intake, feeder),
                            FeederCommands.feedToBeamBreak(feeder)
                        )
                    )
                    .onFalse(
                        Commands.either(
                            Commands.none(),
                            Commands.race(
                                FeederCommands.feedToBeamBreak(feeder).withTimeout(5.0),
                                IntakeCommands.flushIntakeWithoutTheArmExtendedOutward(intake, feeder)
                            )
                        ) { feeder.beamBroken }
                    )

                operatorController
                    .povLeft()
                    .whileTrue(
                        IntakeCommands.flushIntake(intake)
                            .alongWith(FeederCommands.flushFeeder(feeder))
                    )
                operatorController
                    .povDown()
                    .and(operatorController.a().negate())
                    .whileTrue(
                        Commands.sequence(
                            Commands.parallel(
                                ShooterCommands.humanPlayerIntake(shooter),
                                FeederCommands.humanPlayerIntake(feeder)
                            ).until { feeder.beamBroken },
                            Commands.parallel(
                                ShooterCommands.humanPlayerIntake(shooter),
                                FeederCommands.humanPlayerIntake(feeder)
                            ).until { !feeder.beamBroken }
                        ).andThen(FeederCommands.feedToBeamBreak(feeder))
                    )
                    .onFalse(
                        FeederCommands.humanPlayerIntake(feeder)
                            .withTimeout(5.0)
                            .until { !feeder.beamBroken }
                    )
                operatorController
                    .povUp()
                    .and(operatorController.a())
                    .whileTrue(
                        LEDCommands.ledsUp(leds)
                    )
                operatorController
                    .povDown()
                    .and(operatorController.a())
                    .whileTrue(
                        LEDCommands.ledsDown(leds)
                    )

                // ---- SHOOTER COMMANDS ----
                operatorController
                    .y()
                    .whileTrue(ShooterCommands.autoAim(shooter, drive, feeder))
                operatorController
                    .x()
                    .whileTrue(
                        ShooterCommands.JustShoot(shooter)
                    )
                operatorController
                    .leftBumper()
                    .onTrue(
                        FeederCommands.feedToBeamBreak(feeder)
                            .withTimeout(5.0)
                    )
                operatorController.rightTrigger()
                    .whileTrue(ShooterCommands.passNote(shooter))
                driverController
                    .rightTrigger()
                    .whileTrue(
                        Commands.sequence(
                            Commands.waitUntil { (shooter.allAtSetpoint() && (shooter.shooterVelocityRPM > 1000) /*&& command.getReadySupplier().getAsBoolean()*/) },
                            FeederCommands.feedToShooter(feeder)
                                .until { !feeder.beamBroken },
                            FeederCommands.feedToShooter(feeder)
                                .withTimeout(.25)
                        )
                    )
                operatorController.rightBumper().whileTrue(Commands.run({ feeder.runVolts(8.0) }, feeder))
                operatorController
                    .start()
                    .onTrue(
                        ShooterCommands.simpleHoodZero(shooter)
                            .withTimeout(4.0)
                    )
                driverController
                    .rightBumper()
                    .whileTrue(
                        Commands.sequence(
                            FeederCommands.feedToShooter(feeder)
                                .alongWith(ShooterCommands.ampSpin(shooter)).withTimeout(0.2),
                            ShooterCommands.ampAng(shooter)
                                .alongWith(ShooterCommands.ampGo(shooter, 600))
                                .withTimeout(0.25)
                                .andThen(ShooterCommands.setAmpAngle(shooter, -0.4))
                        )
                    )
            }

            SysIDMode.DriveMotors -> {
                drive!!.defaultCommand = DriveCommands.joystickDrive(
                    drive,
                    { driverController.leftY },
                    { driverController.leftX },
                    { driverController.rightX })
                val drivetrainDriveSysID =
                    SysIdRoutine(
                        SysIdRoutine.Config(
                            BaseUnits.Voltage.per(Units.Second).of(2.0),
                            BaseUnits.Voltage.of(8.0),
                            Units.Seconds.of(12.0)
                        ),
                        Mechanism(
                            { voltageMeasure: Measure<Voltage?>? -> drive!!.runCharacterizationVolts(voltageMeasure) },
                            { routineLog: SysIdRoutineLog? -> drive!!.populateDriveCharacterizationData(routineLog) },
                            drive,
                            "DrivetrainDriveMotors"
                        )
                    )
                driverController
                    .x()
                    .whileTrue(drivetrainDriveSysID.dynamic(SysIdRoutine.Direction.kForward))
                    .onFalse(Commands.runOnce({ drive!!.stopWithX() }, drive))
                driverController
                    .y()
                    .whileTrue(drivetrainDriveSysID.dynamic(SysIdRoutine.Direction.kReverse))
                    .onFalse(Commands.runOnce({ drive!!.stopWithX() }, drive))
                driverController
                    .a()
                    .whileTrue(drivetrainDriveSysID.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(20.0))
                    .onFalse(Commands.runOnce({ drive!!.stopWithX() }, drive))
                driverController
                    .b()
                    .whileTrue(drivetrainDriveSysID.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(20.0))
                    .onFalse(Commands.runOnce({ drive!!.stopWithX() }, drive))
                driverController
                    .rightTrigger()
                    .whileTrue(
                        RunCommand({ shooter!!.setTargetShooterAngle(Rotation2d(-0.61)) })
                            .andThen(
                                (RunCommand(
                                    { shooter!!.shooterRunVelocity(5000.0) },  //THIS NUMBER NEEDS TO BE CALIBRATED

                                    intake
                                ))
                            )
                    )
            }

            SysIDMode.TurnMotors -> {}
            SysIDMode.Shooter -> {
                val shooterSysId =
                    SysIdRoutine(
                        SysIdRoutine.Config(
                            BaseUnits.Voltage.per(Units.Second).of(.25),
                            BaseUnits.Voltage.of(9.0),
                            Units.Seconds.of(36.0)
                        ),
                        Mechanism(
                            { voltage: Measure<Voltage?>? -> shooter!!.shooterRunVolts(voltage) },
                            { log: SysIdRoutineLog ->
                                val motor = log.motor("Shooter")
                                motor.voltage(shooter!!.characterizationAppliedVolts)
                                motor.angularPosition(shooter.characterizationPosition)
                                motor.angularVelocity(shooter.characterizationVelocity)
                                motor.current(shooter.characterizationCurrent)
                            },
                            climb,
                            "FlywheelMotors"
                        )
                    )
                driverController
                    .a()
                    .onTrue(
                        shooterSysId
                            .dynamic(SysIdRoutine.Direction.kForward)
                            .withTimeout(3.0)
                            .andThen(
                                WaitCommand(5.0),
                                shooterSysId.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(3.0),
                                WaitCommand(5.0),
                                shooterSysId.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(36.0),
                                WaitCommand(5.0),
                                shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(36.0)
                            )
                            .alongWith(RunCommand({
                                shooter!!.setTargetShooterAngle(Rotation2d.fromRadians(0.0))
                                shooter.setCharacterizeMode(true)
                            }))
                    )
            }

            SysIDMode.AllElse -> {
                val shooterArmSysID =
                    SysIdRoutine(
                        SysIdRoutine.Config(
                            BaseUnits.Voltage.per(Units.Second).of(1.0),
                            BaseUnits.Voltage.of(9.0),
                            Units.Seconds.of(5.0)
                        ),
                        Mechanism(
                            { voltageMeasure: Measure<Voltage?>? -> shooter!!.runHoodVoltage(voltageMeasure) },
                            { log: SysIdRoutineLog ->
                                val motor = log.motor("FeederKraken")
                                motor
                                    .voltage(shooter!!.hoodCharacterizationVoltage)
                                    .angularPosition(shooter.hoodCharacterizationPosition)
                                    .angularVelocity(shooter.hoodCharacterizationVelocity)
                            },
                            climb,
                            "FeederMotors"
                        )
                    )
                val intakeWheelsSysID =
                    SysIdRoutine(
                        SysIdRoutine.Config(
                            BaseUnits.Voltage.per(Units.Second).of(1.0),
                            BaseUnits.Voltage.of(9.0),
                            Units.Seconds.of(9.0)
                        ),
                        Mechanism(
                            { voltageMeasure: Measure<Voltage?>? -> intake!!.runArmVolts(voltageMeasure) },
                            { log: SysIdRoutineLog ->
                                val motor = log.motor("IntakeWheels")
                                motor.voltage(intake!!.armCharacterizationVoltage)
                                motor.angularPosition(intake.armCharacterizationPosition)
                                motor.angularVelocity(intake.armCharacterizationVelocity)
                                motor.current(intake.armCharacterizationCurrent)
                            },
                            intake,
                            "IntakeWheels"
                        )
                    )
                driverController.a().onTrue(
                    Commands.sequence(
                        shooterArmSysID.quasistatic(SysIdRoutine.Direction.kForward).until {
                            shooter!!.hoodCharacterizationPosition.gte(
                                Units.Radians.of(1.8)
                            ) && shooter.hoodCharacterizationPosition.lte(Units.Radians.of(-1.6))
                        },  //fixme shooterArmSysID has incorrect name (intake arm sysid)
                        shooterArmSysID.quasistatic(SysIdRoutine.Direction.kReverse).until {
                            shooter!!.hoodCharacterizationPosition.gte(
                                Units.Radians.of(1.8)
                            ) && shooter.hoodCharacterizationPosition.lte(Units.Radians.of(-1.6))
                        },
                        shooterArmSysID.dynamic(SysIdRoutine.Direction.kForward).until {
                            shooter!!.hoodCharacterizationPosition.gte(
                                Units.Radians.of(1.8)
                            ) && shooter.hoodCharacterizationPosition.lte(Units.Radians.of(-1.6))
                        },
                        shooterArmSysID.dynamic(SysIdRoutine.Direction.kReverse).until {
                            shooter!!.hoodCharacterizationPosition.gte(
                                Units.Radians.of(1.8)
                            ) && shooter.hoodCharacterizationPosition.lte(Units.Radians.of(-1.6))
                        }
                    )
                )
            }
        }
    }

    // TODO: populate switch statements here
    fun getEnterCommand(m: Mode?): Command {
        return Commands.none()
    }

    fun getExitCommand(m: Mode?): Command {
        return Commands.none()
    }

    private fun configureReactions() {
        driverRumble.defaultCommand = RumbleCommands.noRumble(driverRumble).ignoringDisable(true)
        operatorRumble.defaultCommand = RumbleCommands.noRumble(operatorRumble).ignoringDisable(true)
        reactions.intakeBeamBroken
            .and(reactions.shooterBeamBroken.negate())
            .whileTrue(
                Commands.parallel(
                    Commands.parallel(
                        RumbleCommands.rumbleLight(driverRumble)
                            .withTimeout(0.1),
                        Commands.waitSeconds(0.2)
                            .andThen(RumbleCommands.rumbleLightWithFalloff(operatorRumble).withTimeout(10.0))
                    ),
                    LEDCommands.hasNote(leds)
                        .withTimeout(1.0)
                        .andThen(
                            LEDCommands.setIntakeType(leds)
                        )
                ).ignoringDisable(true)
            )
        reactions
            .isAutonomous
            .and(reactions.isEnabled)
            .whileTrue(LEDCommands.flameCommand(leds).ignoringDisable(true))
        reactions
            .isTeleop
            .and(reactions.isEnabled)
            .whileTrue(LEDCommands.enabled(leds).ignoringDisable(true))
        reactions
            .isTeleop
            .and(reactions.isEnabled)
            .whileTrue(ClimbCommands.zeroClimb(climb).ignoringDisable(true).withTimeout(6.5))
    }


    internal enum class SmartCommandsMode {
        Smart,
        Safe,
        Disabled
    }

    val autonomousCommand: Command
        /**
         * Use this to pass the autonomous command to the main [Robot] class.
         *
         * @return the command to run in autonomous
         */
        get() = autoChooser.get()

    enum class SysIDMode {
        Disabled,
        DriveMotors,
        TurnMotors,
        Shooter,
        AllElse
    }

    private class ReactionObject(
        var intakeBeamBroken: Trigger,
        var shooterBeamBroken: Trigger,
        var isTeleop: Trigger,
        var isAutonomous: Trigger,
        var isEnabled: Trigger
    )

    companion object {
        var sysIDMode: SysIDMode = SysIDMode.Disabled
    }
}
