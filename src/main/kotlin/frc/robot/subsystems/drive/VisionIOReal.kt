package frc.robot.subsystems.drive

import frc.robot.subsystems.drive.VisionIO.VisionIOInputs
import org.photonvision.PhotonCamera

class VisionIOReal(cameraName: String?) : VisionIO {
    // Update with camera name
    private val cam: PhotonCamera = PhotonCamera(cameraName)

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.cameraResult = cam.latestResult
        inputs.latencyMillis = cam.latestResult.latencyMillis
        inputs.driverMode = cam.driverMode
        inputs.timestampSeconds = cam.latestResult.timestampSeconds
        inputs.connected = cam.isConnected
        inputs.name = cam.name
    }
}
