package frc.robot.subsystems.drive

import org.littletonrobotics.junction.AutoLog

interface NoteDetectionIO {
    val cameraName: String
        get() {
            return Companion.cameraName
        }

    fun updateInputs(inputs: NoteDetectionIOInputs) {
    }

    fun turnOffLEDs() {
    }

    @AutoLog
    class NoteDetectionIOInputs {
        @JvmField
        var name: String = ""
        @JvmField
        var jsonDump: String = ""
        @JvmField
        var tx: Double = 0.0
        @JvmField
        var ty: Double = 0.0
        @JvmField
        var ta: Double = 0.0
        @JvmField
        var tv: Boolean = false
    }

    companion object {
        const val cameraName: String = ""
    }
}
