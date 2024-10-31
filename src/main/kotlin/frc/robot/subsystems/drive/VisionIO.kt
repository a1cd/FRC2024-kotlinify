package frc.robot.subsystems.drive

import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.LogTable.LogValue
import org.littletonrobotics.junction.inputs.LoggableInputs
import org.photonvision.common.dataflow.structures.Packet
import org.photonvision.targeting.PhotonPipelineResult

interface VisionIO {
    val cameraName: String
        get() {
            return Companion.cameraName
        }

    fun updateInputs(inputs: VisionIOInputs) {
    }

    class VisionIOInputs : LoggableInputs {
        var driverMode: Boolean = false
        var latencyMillis: Double = 0.0
        var timestampSeconds: Double = 0.0
        var connected: Boolean = false
        var name: String = ""
        var cameraResult: PhotonPipelineResult = PhotonPipelineResult()
        private var aPacketSerde: PhotonPipelineResult.APacketSerde? = null

        constructor(name: String) {
            this.name = name
        }

        constructor()

        fun arrayToPosition(value: DoubleArray): Array<Transform3d?> {
            val length: Int = ((value.size / 7.0).toInt())
            val data: Array<Transform3d?> = arrayOfNulls(length + 1)
            for (i in 0 until length) {
                val X: Double = value[i * 7]
                val Y: Double = value[i * 7 + 1]
                val Z: Double = value[i * 7 + 2]
                val rotW: Double = value[i * 7 + 3]
                val rotX: Double = value[i * 7 + 4]
                val rotY: Double = value[i * 7 + 5]
                val rotZ: Double = value[i * 7 + 6]
                data[i] =
                    Transform3d(
                        Translation3d(X, Y, Z), Rotation3d(Quaternion(rotW, rotX, rotY, rotZ))
                    )
            }
            return data
        }

        fun positionToArray(vararg value: Transform3d): DoubleArray {
            val data: DoubleArray = DoubleArray(value.size * 7)
            for (i in value.indices) {
                data[i * 7] = value[i].x
                data[i * 7 + 1] = value[i].y
                data[i * 7 + 2] = value[i].z
                data[i * 7 + 3] = value[i].rotation.quaternion.w
                data[i * 7 + 4] = value[i].rotation.quaternion.x
                data[i * 7 + 5] = value[i].rotation.quaternion.y
                data[i * 7 + 6] = value[i].rotation.quaternion.z
            }
            return data
        }

        override fun toLog(table: LogTable) {
            table.put("LatencyMillis", latencyMillis)
            table.put("TimestampSeconds", timestampSeconds)
            table.put("Name", name)

            val packet: Packet = Packet(cameraResult.packetSize)
            aPacketSerde = PhotonPipelineResult.APacketSerde()
            aPacketSerde!!.pack(packet, cameraResult)
            table.put("CameraResultData", packet.data)
        }

        override fun fromLog(table: LogTable) {
            //            driverMode = table.getBoolean("DriverMode", driverMode);
            latencyMillis = table.get("LatencyMillis", latencyMillis)
            timestampSeconds = table.get("TimestampSeconds", timestampSeconds)
            name = table.get("Name", name)
            val cameraResultData: LogValue? = table.get("CameraResultData")
            if (cameraResultData != null) {
                cameraResultData.raw
                cameraResult = PhotonPipelineResult.serde.unpack(Packet(cameraResultData.raw))
            } else {
                connected = false
            }

            cameraResult.timestampSeconds = timestampSeconds
        }

        override fun equals(o: Any?): Boolean {
            if (this === o) return true
            if (o !is VisionIOInputs) return false

            val that: VisionIOInputs = o

            if (driverMode != that.driverMode) return false
            if (timestampSeconds.compareTo(that.timestampSeconds) != 0) return false
            return cameraResult == that.cameraResult
        }

        override fun hashCode(): Int {
            var result: Int = (if (driverMode) 1 else 0)
            var temp: Long = java.lang.Double.doubleToLongBits(latencyMillis)
            result = 31 * result + (temp xor (temp ushr 32)).toInt()
            temp = java.lang.Double.doubleToLongBits(timestampSeconds)
            result = 31 * result + (temp xor (temp ushr 32)).toInt()
            result = 31 * result + cameraResult.hashCode()
            return result
        }
    }

    companion object {
        const val cameraName: String = ""
    }
}
