package frc.robot.subsystems.drive

import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.drive.VisionIO.VisionIOInputs
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.function.Supplier

class VisionIOSim(name: String?, private var pose2dSupplier: Supplier<Pose2d>) : VisionIO {
    private var cameraSystem: VisionSystemSim
    private var camera: PhotonCamera
    private var cameraProp: SimCameraProperties = SimCameraProperties()

    /*Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
 // The given target model at the given pose
 VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

 // Add this vision target to the vision system simulation to make it visible
 cameraSystem.addVisionTargets(visionTarget);*/
    init {
        val aprilTagFieldLayout =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
        // A 640 x 480 camera with a 0 degree diagonal FOV.
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70.0)) // rotation not updated
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.9309056222667738, 0.08)
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.fps = 26.1 // not updated
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.avgLatencyMs = 20.663 // not updated
        cameraProp.latencyStdDevMs = 5.811 // not updated

        //         Update robotToCam with cameraSystem mounting pos
        cameraSystem = VisionSystemSim("main")
        cameraSystem.addAprilTags(aprilTagFieldLayout)

        camera = PhotonCamera(name)
        val cameraSim = PhotonCameraSim(camera, cameraProp)
        cameraSim.enableRawStream(false)
        cameraSim.enableProcessedStream(true)
        cameraSim.enableDrawWireframe(false)
        cameraSystem.addCamera(cameraSim, robotToCam.get(0))
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        cameraSystem.update(pose2dSupplier.get())
        inputs.cameraResult = camera.latestResult
        inputs.latencyMillis = camera.latestResult.latencyMillis
        inputs.driverMode = camera.driverMode
        inputs.timestampSeconds = camera.latestResult.timestampSeconds
    }
}
