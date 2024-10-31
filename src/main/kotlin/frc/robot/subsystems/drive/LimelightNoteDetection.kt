package frc.robot.subsystems.drive

import frc.robot.subsystems.drive.NoteDetectionIO.NoteDetectionIOInputs
import frc.robot.util.LimelightHelpers

open class LimelightNoteDetection : NoteDetectionIO {
    override val cameraName: String
        get() = "limelight"

    override fun turnOffLEDs() {
        LimelightHelpers.setLEDMode_ForceOff(cameraName)
    }

    override fun updateInputs(inputs: NoteDetectionIOInputs) {
        inputs.name = cameraName
        //        inputs.jsonDump = LimelightHelpers.getJSONDump(getCameraName());
        inputs.tx = LimelightHelpers.getTX(cameraName)
        inputs.ty = LimelightHelpers.getTY(cameraName)
        inputs.ta = LimelightHelpers.getTA(cameraName)
        inputs.tv = LimelightHelpers.getTV(cameraName)
    }
}
