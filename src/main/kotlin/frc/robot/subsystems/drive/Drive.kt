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

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.PathPlannerLogging
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.subsystems.drive.VisionIO.VisionIOInputs
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.photonvision.PhotonPoseEstimator
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.pow

class Drive(
    gyroIO: GyroIO,
    flModuleIO: ModuleIO,
    frModuleIO: ModuleIO,
    blModuleIO: ModuleIO,
    brModuleIO: ModuleIO,
    private val visionIO: Array<VisionIO>,
    private val noteIO: NoteDetectionIO
) : SubsystemBase() {
    private val gyroIO: GyroIO

    private val noteInputs = NoteDetectionIOInputsAutoLogged()
    private val gyroInputs = GyroIOInputsAutoLogged()
    private val modules = arrayOfNulls<Module>(4) // FL, FR, BL, BR
    private var swerveModulePositions: Array<SwerveModulePosition?>
    private val visionInputs: Array<VisionIOInputs>

    //  private final
    private val kinematics = SwerveDriveKinematics(*moduleTranslations)
    private val poseEstimator: SwerveDrivePoseEstimator
    private var pose = Pose2d()
    private var positionPID: PIDConstants = PIDConstants(5.0, 0.0) //64 //works at 8
    var rotationPID: PIDConstants = PIDConstants(3.0, 0.0) //32+16
    private var angularVelocity: Measure<Velocity<Angle>> = edu.wpi.first.units.Units.RadiansPerSecond.zero()
    private val lastGyroRotation = Rotation2d()

    private var aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

    // from center.
    private val photonPoseEstimator: Array<PhotonPoseEstimator?>
    private var noGyroPoseEstimation: SwerveDrivePoseEstimator?
    private var noGyroRotation: Rotation2d?
    private var previousPose = Pose2d()
    private var timestampSeconds: DoubleArray

    private val lastModulePositions =  // For delta tracking
        arrayOf<SwerveModulePosition?>(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition()
        )

    constructor(
        gyroIO: GyroIO,
        visionIO: VisionIO,
        flModuleIO: ModuleIO,
        frModuleIO: ModuleIO,
        blModuleIO: ModuleIO,
        brModuleIO: ModuleIO,
        noteIO: NoteDetectionIO
    ) : this(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO, arrayOf<VisionIO>(visionIO), noteIO)

    private var transform: Transform2d = Transform2d(0.0, 0.0, Rotation2d.fromRadians(0.0))

    init {
        visionInputs = arrayOfNulls(visionIO.size)
        run {
            var i = 0
            val visionIOLength = visionIO.size
            while (i < visionIOLength) {
                visionInputs[i] = object : VisionIOInputs(visionIO[i].cameraName) {
                }
                i++
            }
        }
        timestampSeconds = DoubleArray(visionIO.size)
        run {
            var i = 0
            val visionIOLength = visionIO.size
            while (i < visionIOLength) {
                timestampSeconds[i] = 0.0
                i++
            }
        }

        this.gyroIO = gyroIO

        modules[3] = Module(flModuleIO, 0)
        modules[2] = Module(frModuleIO, 1)
        modules[1] = Module(blModuleIO, 2)
        modules[0] = Module(brModuleIO, 3)

        photonPoseEstimator = arrayOfNulls(visionInputs.size)
        for (i in visionInputs.indices) {
            photonPoseEstimator[i] = PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam.get(i)
            )
        }

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
            { this.getPose() },
            { pose: Pose2d -> this.setPose(pose) },
            { kinematics.toChassisSpeeds(*moduleStates) },
            { speeds: ChassisSpeeds -> this.runVelocity(speeds) },
            HolonomicPathFollowerConfig(
                positionPID, rotationPID,
                maxLinearSpeedMetersPerSec, DRIVE_BASE_RADIUS, ReplanningConfig(true, true), Robot.defaultPeriodSecs
            ),
            {
                DriverStation.getAlliance().isPresent
                        && DriverStation.getAlliance().get() == Alliance.Red
            },
            this
        )
        //    Pathfinding.setPathfinder(new LocalADStar());
        PathPlannerLogging.setLogActivePathCallback { activePath: List<Pose2d> ->
            Logger.recordOutput(
                "Odometry/Trajectory", *activePath.toTypedArray<Pose2d>()
            )
        }
        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d ->
            Logger.recordOutput(
                "Odometry/TrajectorySetpoint",
                targetPose
            )
        }

        poseEstimator =
            SwerveDrivePoseEstimator(
                kinematics,
                gyroInputs.yawPosition,
                arrayOf(
                    modules[0].getPosition(),
                    modules[1].getPosition(),
                    modules[2].getPosition(),
                    modules[3].getPosition()
                ),
                Pose2d(3.0, 5.0, Rotation2d(3.0))
            )
        poseEstimator.setVisionMeasurementStdDevs(Matrix(Nat.N3(), Nat.N1(), doubleArrayOf(4.0, 4.0, 8.0)))

        swerveModulePositions = arrayOfNulls(modules.size)
        noGyroPoseEstimation = null
        noGyroRotation = null
    }

    //    Transform2d accelerometerModifiedVelocity = new Transform2d(0,0, Rotation2d.fromRadians(0));
    //    LinearSystem system = new LinearSystem<N3, N5, N3>(
    //            new Matrix<>()
    //    ).;
    //    KalmanFilter<N2, N2, N2> xVelocityKF = new KalmanFilter<N4, N6, N4>(Nat.N4(), Nat.N4())
    override fun periodic() {
        previousPose = pose

        if (this.currentCommand != null) {
            Logger.recordOutput("Commands/Drive", this.currentCommand.name)
        } else {
            Logger.recordOutput("Commands/Drive", "")
        }

        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)

        noteIO.updateInputs(noteInputs)
        Logger.processInputs("Drive/Note", noteInputs)

        var vision: VisionIO? = null
        run {
            var i = 0
            val visionIOLength = visionIO.size
            while (i < visionIOLength) {
                vision = visionIO[i]
                vision!!.updateInputs(visionInputs[i])
                i++
            }
        }
        for (i in visionInputs.indices) {
            val visionInput = visionInputs[i]
            Logger.processInputs("Drive/Vision/" + visionIO[i].cameraName, visionInput)
        }

        for (module in modules) {
            module.periodic()
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) for (module in modules) module.stop()
        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleState>())
            Logger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState>())
        }

        updateSwerveModulePositions()

        // Update odometry

        // Read wheel positions and deltas from each module
        swerveModulePositions = modulePositions
        val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)
        for (moduleIndex in 0..3) {
            moduleDeltas[moduleIndex] =
                SwerveModulePosition(
                    swerveModulePositions[moduleIndex]!!.distanceMeters
                            - lastModulePositions[moduleIndex]!!.distanceMeters,
                    swerveModulePositions[moduleIndex]!!.angle
                )
            lastModulePositions[moduleIndex] = swerveModulePositions[moduleIndex]
        }


        // The twist represents the motion of the robot since the last
        // loop cycle in x, y, and theta based only on the modules,
        // without the gyro. The gyro is always disconnected in simulation.
        val twist = kinematics.toTwist2d(*moduleDeltas)
        this.transform = pose.exp(twist).minus(pose)
        twist.dtheta *= -1.0
        if (isGyroConnected) {
            var isAnyCameraConnected = false
            for (visionInput in visionInputs) {
                if (visionInput.connected) {
                    isAnyCameraConnected = true
                    break
                }
            }
            if (noGyroPoseEstimation != null && isAnyCameraConnected) {
                // todo: make the next line conditional, only update pose if cameras are
                //  online, otherwise don't do it.
                poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose)
                noGyroPoseEstimation = null
            }

            // If the gyro is connected, replace the theta component of the twist
            // with the change in angle since the last loop cycle.

            // update the pose estimator
            pose = poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyroInputs.yawPosition, swerveModulePositions)
            angularVelocity = edu.wpi.first.units.Units.RadiansPerSecond.of(gyroInputs.yawVelocityRadPerSec)
        } else {
            if (noGyroPoseEstimation == null) {
                noGyroRotation = pose.rotation
                noGyroPoseEstimation =
                    SwerveDrivePoseEstimator(
                        kinematics, pose.rotation, swerveModulePositions, pose
                    )
            }

            // Apply the twist (change since last loop cycle) to the current pose
            noGyroRotation = noGyroRotation!!.plus(Rotation2d.fromRadians(twist.dtheta))
            pose =
                noGyroPoseEstimation!!.updateWithTime(Timer.getFPGATimestamp(), noGyroRotation, swerveModulePositions)
            angularVelocity = edu.wpi.first.units.Units.RadiansPerSecond.of(twist.dtheta / .02)
        }

        Logger.recordOutput("pose", pose)
        /*
    _____ R: <-x  y^
    | ^ | G: \|/x ->y
    |___|
    */
//    new UnscentedKalmanFilter<>()
//    Transform2d twistPerDt = getTwistPerDt();
//    poseEstimator.addVisionMeasurement(pose.minus(new Transform2d(gyroInputs.accelY * 0.02 * 0.02 - twistPerDt.getX() * .02, gyroInputs.accelX * 0.02 * 0.02 - twistPerDt.getY() * .02, Rotation2d.fromDegrees(0.0))), new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{0.075, .075, 100.0}));// fixme: add acceleration from gyro
//    var visionInput = visionInputs[0];
//
        run {
            var i = 0
            val visionInputsLength = visionInputs.size
            while (i < visionInputsLength) {
                val visionInput = visionInputs[i]
                val estPose = photonPoseEstimator[i]!!.update(visionInput.cameraResult)
                if (estPose.isPresent && (timestampSeconds[i] != estPose.get().timestampSeconds)) {
                    val estimatedRobotPose = estPose.get()
                    Logger.recordOutput(
                        "Vision/" + visionIO[i].cameraName + "/Estimated Robot Pose",
                        estimatedRobotPose.estimatedPose
                    )
                    val pose2d = estimatedRobotPose.estimatedPose.toPose2d()
                    if (((!DriverStation.isFMSAttached()) ||
                                ((estimatedRobotPose.estimatedPose.x <= 16.5) &&
                                        (estimatedRobotPose.estimatedPose.x > 0) &&
                                        (estimatedRobotPose.estimatedPose.z <= 1) &&
                                        (estimatedRobotPose.estimatedPose.z > -1) &&  //fixme: remove Z restriction
                                        (estimatedRobotPose.estimatedPose.y <= 8.2) &&
                                        (estimatedRobotPose.estimatedPose.y > 0)))
                    )  // only add it if it's less than 1 meter and in the field
                    {
                        var visionMatrix: Matrix<N3?, N1?>
                        when (estimatedRobotPose.targetsUsed.size) {
                            0 -> visionMatrix = Matrix(Nat.N3(), Nat.N1(), doubleArrayOf(16.0, 16.0, 32.0))
                            1 -> {
                                val target = estimatedRobotPose.targetsUsed[0]
                                var mult = target.poseAmbiguity * 500
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Single Tag Matrix Multiplier/Step 0",
                                    mult
                                )
                                mult *= mult
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Single Tag Matrix Multiplier/Step 1",
                                    mult
                                )
                                if ((pose.x <= 16.5) &&
                                    (pose.x > 0) &&
                                    (pose.y <= 8.2) &&
                                    (pose.y > 0)
                                ) mult /= 4
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Single Tag Matrix Multiplier/Step 2",
                                    mult
                                )
                                if (target.poseAmbiguity < 0.4) mult *= abs(estimatedRobotPose.estimatedPose.z).pow(2.0) + 0.01
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Single Tag Matrix Multiplier/Step 3",
                                    mult
                                )
                                if (target.area > 0.05) mult /= (target.area * target.area) * 8 * 8
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Single Tag Matrix Multiplier/Step 4",
                                    mult
                                )
                                var j = 0
                                while (j < visionInputs.size) {
                                    if (i == j) break
                                    if (visionInputs[j].cameraResult.getTargets().size > visionInputs[i].cameraResult.getTargets().size && visionInputs[j].timestampSeconds != timestampSeconds[j]) mult += 0.5
                                    j++
                                }
                                mult += 2.0
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Single Tag Matrix Multiplier/Step 5",
                                    mult
                                )
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Single Tag Matrix Multiplier",
                                    mult
                                )
                                visionMatrix = Matrix(Nat.N3(), Nat.N1(), doubleArrayOf(8 * mult, 8 * mult, 12 * mult))
                            }

                            else -> {
                                val multiTagResult = visionInput.cameraResult.multiTagResult
                                val meterErrorEstimation =
                                    (multiTagResult.estimatedPose.bestReprojErr / visionInput.cameraResult.bestTarget.area) * 0.045
                                //                            meterErrorEstimation = 1;
                                Logger.recordOutput(
                                    "Vision/" + visionIO[i].cameraName + "/Multi Tag meterErrorEstimation",
                                    meterErrorEstimation
                                )
                                visionMatrix = Matrix(
                                    Nat.N3(),
                                    Nat.N1(),
                                    doubleArrayOf(
                                        1.75 * meterErrorEstimation,
                                        5 * meterErrorEstimation,
                                        3 * meterErrorEstimation
                                    )
                                )
                            }
                        }
                        if (isGyroConnected) poseEstimator.addVisionMeasurement(
                            pose2d,
                            estimatedRobotPose.timestampSeconds,
                            visionMatrix
                        )
                        else noGyroPoseEstimation!!.addVisionMeasurement(
                            pose2d,
                            estimatedRobotPose.timestampSeconds,
                            visionMatrix
                        )
                        poseEstTransform()
                    }
                }
                i++
            }
        }
        var i = 0
        val visionInputsLength = visionInputs.size
        while (i < visionInputsLength) {
            val visionInput = visionInputs[i]
            timestampSeconds[i] = visionInput.timestampSeconds
            i++
        }
        twistPerDt
        logCameraData()
    }

    private fun updateSwerveModulePositions() {
        // populate the list
        for (i in modules.indices) swerveModulePositions[i] = modules[i].getPosition()
    }

    val isGyroConnected: Boolean
        get() = gyroInputs.connected

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    fun runVelocity(speeds: ChassisSpeeds) {
        val flipped = ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond)
        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(flipped, 0.02)
        val setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeedMetersPerSec)

        // Send setpoints to modules
        val optimizedSetpointStates = arrayOfNulls<SwerveModuleState>(4)
        for (i in 0..3) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i]!!.runSetpoint(setpointStates[i])
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *optimizedSetpointStates)
    }

    /**
     * Stops the drive.
     */
    fun stop() {
        runVelocity(ChassisSpeeds())
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    fun stopWithX() {
        val headings = arrayOfNulls<Rotation2d>(4)
        for (i in 0..3) {
            headings[i] = moduleTranslations[i].angle
        }
        kinematics.resetHeadings(*headings)
        stop()
    }

    /**
     * Runs forwards at the commanded voltage.
     */
    fun runCharacterizationVolts(voltageMeasure: Measure<Voltage?>) {
        for (i in 0..3) {
            modules[i]!!.runCharacterization(voltageMeasure.`in`(edu.wpi.first.units.Units.Volts))
        }
    }

    val characterizationVelocity: Double
        /**
         * Returns the average drive velocity in radians/sec.
         */
        get() {
            var driveVelocityAverage = 0.0
            for (module in modules) {
                driveVelocityAverage += module.characterizationVelocityRadPerSec
            }
            return driveVelocityAverage / 4.0
        }

    /**
     * Returns the average drive velocity in radians/sec.
     */
    fun populateDriveCharacterizationData(routineLog: SysIdRoutineLog) {
        var driveVelocityAverage = edu.wpi.first.units.Units.RadiansPerSecond.zero()
        var drivePositionAverage = edu.wpi.first.units.Units.Radians.zero()

        for (module in modules) {
            val motor = routineLog.motor("DriveMotor #" + module.index)
            val angularPosition = module.characterizationDrivePosition
            val angularVelocity = module.characterizationDriveVelocity
            motor.angularPosition(angularPosition)
            motor.angularVelocity(angularVelocity)

            drivePositionAverage = drivePositionAverage.plus(angularPosition)
            driveVelocityAverage = driveVelocityAverage.plus(angularVelocity)
        }
        val averageDriveMotor = routineLog.motor("Average DriveMotor")
        averageDriveMotor.angularVelocity(driveVelocityAverage.divide(4.0))
        averageDriveMotor.angularPosition(drivePositionAverage.divide(4.0))
    }

    @AutoLogOutput
    fun poseEstTransform(): Transform3d? {
        return if (visionInputs.size >= 2) visionInputs[1].cameraResult.multiTagResult.estimatedPose.best
            .plus(visionInputs[0].cameraResult.multiTagResult.estimatedPose.best.inverse())
        else null
    }

    fun populateTurnCharacterizationData(routineLog: SysIdRoutineLog) {
        var driveVelocityAverage = edu.wpi.first.units.Units.RadiansPerSecond.zero()
        var drivePositionAverage = edu.wpi.first.units.Units.Radians.zero()

        for (module in modules) {
            val motor = routineLog.motor("TurnMotor #" + module.index)
            val angularPosition = module.characterizationTurnPosition
            val angularVelocity = module.characterizationTurnVelocity
            motor.angularPosition(angularPosition)
            motor.angularVelocity(angularVelocity)

            driveVelocityAverage = driveVelocityAverage.plus(angularVelocity)
            drivePositionAverage = drivePositionAverage.plus(angularPosition)
        }
        val averageDriveMotor = routineLog.motor("Average TurnMotor")
        averageDriveMotor.angularVelocity(driveVelocityAverage.divide(4.0))
        averageDriveMotor.angularPosition(drivePositionAverage.divide(4.0))
    }

    private fun logCameraData() {
        for (k in visionInputs.indices) {
            val visionInput = visionInputs[k]
            // Corners
            val targets = visionInput.cameraResult.targets
            val outCorners = arrayOfNulls<Translation2d>(targets.size * 4)
            for (i in targets.indices) {
                val corners = targets[i].detectedCorners
                for (j in corners.indices) {
                    val corner = corners[j]
                    outCorners[i + j] = Translation2d(
                        corner.x,
                        corner.y
                    )
                }
            }
            for (i in outCorners.indices) {
                val translation2d = outCorners[i]
                if (translation2d == null) outCorners[i] = Translation2d()
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Corners", *outCorners)

            // PNP Result
            val a = visionInput.cameraResult.multiTagResult.estimatedPose
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/best", a.best)
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/bestReprojErr", a.bestReprojErr)
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/isPresent", a.isPresent)
            Logger.recordOutput("Vision/" + visionInput.name + "/MultiTag PNP Result/ambiguity", a.ambiguity)

            // Tag Count
            Logger.recordOutput("Vision/" + visionInput.name + "/Tag Count", visionInput.cameraResult.getTargets().size)

            // Camera Offsets
            Logger.recordOutput<Pose3d>(
                "Vision/" + visionInput.name + "/Vision Override",
                Pose3d(pose).plus(robotToCam.get(k))
            )

            val outAmbiguities = DoubleArray(targets.size)
            run {
                var i = 0
                val targetsSize = targets.size
                while (i < targetsSize) {
                    val target = targets[i]
                    outAmbiguities[i] = target.poseAmbiguity
                    i++
                }
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Tag Ambiguities", outAmbiguities)

            val outAreas = DoubleArray(targets.size)
            run {
                var i = 0
                val targetsSize = targets.size
                while (i < targetsSize) {
                    val target = targets[i]
                    outAreas[i] = target.area
                    i++
                }
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Tag Areas", outAreas)

            // Tags 2D
            val outTags2D = arrayOfNulls<Pose2d>(targets.size)
            for (i in targets.indices) {
                val b: Transform3d = robotToCam.get(k).plus(targets[i].bestCameraToTarget)
                outTags2D[i] = pose.plus(Transform2d(b.translation.toTranslation2d(), b.rotation.toRotation2d()))
            }
            Logger.recordOutput("Vision/" + visionInput.name + "/Tags 2d", *outTags2D)

            val outTags3d = arrayOfNulls<Pose3d>(targets.size)
            for (i in targets.indices) outTags3d[i] =
                Pose3d(pose).plus(robotToCam.get(k).plus(targets[i].bestCameraToTarget))
            Logger.recordOutput("Vision/" + visionInput.name + "/Tags 3d", *outTags3d)
        }
    }

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState?>
        /**
         * Returns the module states (turn angles and drive velocities) for all of the modules.
         */
        get() {
            val states = arrayOfNulls<SwerveModuleState>(4)
            for (i in 0..3) states[i] = modules[i].getState()
            return states
        }

    private val modulePositions: Array<SwerveModulePosition?>
        /** Returns the module positions (turn angles and drive positions) for all of the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModulePosition>(4)
            for (i in 0..3) {
                states[i] = modules[i].getPosition()
            }
            return states
        }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    fun getPose(): Pose2d {
        return pose
    }

    @AutoLogOutput(key = "Vision/Camera Count")
    fun cameraCount(): Int {
        var cameraCount = 0
        for (visionInput in visionInputs) {
            if (visionInput.connected) cameraCount++
        }
        return cameraCount
    }

    @get:AutoLogOutput(key = "Odometry/Pose Per Delta Time")
    val twistPerDt: Transform2d
        /**
         * Returns the current odometry pose.
         */
        get() = transform.times(.02)

    val rotation: Rotation2d
        /**
         * Returns the current odometry rotation.
         */
        get() = pose.rotation

    /**
     * Resets the current odometry pose.
     */
    private fun setPose(pose: Pose2d) {
        this.pose = pose
        if (isGyroConnected) poseEstimator.resetPosition(gyroInputs.yawPosition, swerveModulePositions, pose)
        else noGyroPoseEstimation!!.resetPosition(noGyroRotation, swerveModulePositions, pose)
    }

    @get:AutoLogOutput(key = "[BAD] Angular Velocity")
    val anglularVelocity: Measure<Velocity<Angle>>
        get() = angularVelocity.negate()

    fun shuffleboardMethod(): Array<Module?> {
        return modules
    }

    val detectedNote: Double?
        /**
         *
         * @return Heading to note, null if no note sighted
         */
        get() = if (!noteInputs.tv) {
            null
        } else {
            noteInputs.tx
        }

    companion object {
        const val maxLinearSpeedMetersPerSec: Double = 5.75
        private val TRACK_WIDTH_X = Units.inchesToMeters(20.75)
        private val TRACK_WIDTH_Y = Units.inchesToMeters(20.75)
        private val DRIVE_BASE_RADIUS = hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0)
        val maxAngularSpeedRadPerSec: Double = maxLinearSpeedMetersPerSec / DRIVE_BASE_RADIUS

        val moduleTranslations: Array<Translation2d>
            /**
             * Returns an array of module translations.
             */
            get() = arrayOf(
                Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),  //br
                Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),  //bl
                Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),  //fr
                Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),  //fl
            )
    }
}
