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

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.RobotBase

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    private val isInReplayTestMode: Boolean =
        (System.getenv().getOrDefault("TEST_RUN_MODE", "false").equals("true", ignoreCase = true))
    @JvmField
    val currentMode: Mode =
        if ((RobotBase.isReal())) Mode.REAL else (if ((isInReplayTestMode)) Mode.REPLAY else Mode.SIM)

    @JvmField
    var robotToCam: Array<Transform3d>

    init {
        val ShootSideCamera = Transform3d(
            Translation3d(
                Units.Meters.of(0.258572),  //-0.25009 y
                Units.Meters.of(0.1796796),  //-0.1854 x
                Units.Meters.of(0.280162) //-0.34316 z
            ),
            Rotation3d(
                Units.Rotations.of(.5).`in`(Units.Radians),
                Units.Degrees.of(-30.0).`in`(Units.Radians),
                Units.Degrees.of(3.0).`in`(Units.Radians)
            )
        )
        robotToCam = arrayOf(
            ShootSideCamera,
            Transform3d(
                Translation3d(
                    Units.Meters.of(0.258572).minus(Units.Inches.of(-0.323914)),
                    Units.Meters.of(0.1796796).minus(Units.Inches.of(14.265874)),
                    Units.Meters.of(0.280162).plus(Units.Inches.of(4.938808))
                ),
                Rotation3d(
                    Units.Rotations.of(.5).`in`(Units.Radians),
                    Units.Degrees.of(-30.0).plus(Units.Radians.of(0.1839466536)).`in`(Units.Radians),
                    Units.Degrees.of(3.0).plus(Units.Radians.of(-1.500654)).`in`(Units.Radians)
                )
            )
        )
    }

    enum class Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }
}
