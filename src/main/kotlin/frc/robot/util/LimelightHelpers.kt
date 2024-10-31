//LimelightHelpers v1.4.0 (March 21, 2024)
package frc.robot.util

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance

object LimelightHelpers {

    private fun sanitizeName(name: String?): String {
        if (name === "" || name == null) {
            return "limelight"
        }
        return name
    }

    private fun getLimelightNTTable(tableName: String?): NetworkTable {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName))
    }

    private fun getLimelightNTTableEntry(tableName: String?, entryName: String?): NetworkTableEntry {
        return getLimelightNTTable(tableName).getEntry(entryName)
    }

    private fun getLimelightNTDouble(tableName: String?, entryName: String?): Double {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0)
    }

    private fun setLimelightNTDouble(tableName: String?, entryName: String?, `val`: Double) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(`val`)
    }

    private fun getLimelightNTDoubleArray(tableName: String?, entryName: String?): DoubleArray {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(DoubleArray(0))
    }

    /////
    /////
    fun getTX(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "tx")
    }

    fun getTY(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "ty")
    }

    fun getTA(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "ta")
    }

    /**
     * Switch to getBotPose
     *
     * @param limelightName
     * @return
     */
    @Deprecated("")
    fun getBotpose(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose")
    }

    /**
     * Switch to getBotPose_wpiRed
     *
     * @param limelightName
     * @return
     */
    @Deprecated("")
    fun getBotpose_wpiRed(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired")
    }

    /**
     * Switch to getBotPose_wpiBlue
     *
     * @param limelightName
     * @return
     */
    @Deprecated("")
    fun getBotpose_wpiBlue(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue")
    }

    fun getTV(limelightName: String?): Boolean {
        return 1.0 == getLimelightNTDouble(limelightName, "tv")
    }


    fun setLEDMode_ForceOff(limelightName: String?) {
        setLimelightNTDouble(limelightName, "ledMode", 1.0)
    }


    /////
    /////

}