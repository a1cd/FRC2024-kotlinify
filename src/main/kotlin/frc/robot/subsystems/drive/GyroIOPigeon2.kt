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

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs

/** IO implementation for Pigeon2  */
class GyroIOPigeon2 : GyroIO {
    private val pigeon = Pigeon2(20)
    private val yaw: StatusSignal<Double> = pigeon.yaw
    private val yawVelocity: StatusSignal<Double> = pigeon.angularVelocityZWorld

    private val accelerationX: StatusSignal<Double> = pigeon.accelerationX
    private val accelerationY: StatusSignal<Double> = pigeon.accelerationY
    private val accelerationZ: StatusSignal<Double> = pigeon.accelerationZ

    private val getMagFieldX: StatusSignal<Double> = pigeon.magneticFieldX
    private val getMagFieldY: StatusSignal<Double> = pigeon.magneticFieldY
    private val getMagFieldZ: StatusSignal<Double> = pigeon.magneticFieldZ

    private val quatW: StatusSignal<Double> = pigeon.quatW
    private val quatX: StatusSignal<Double> = pigeon.quatX
    private val quatY: StatusSignal<Double> = pigeon.quatY
    private val quatZ: StatusSignal<Double> = pigeon.quatZ

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)
        StatusSignal.setUpdateFrequencyForAll(
            100.0,
            yaw, yawVelocity, accelerationY, accelerationX, accelerationZ, getMagFieldX,
            getMagFieldY, getMagFieldZ,
            quatW, quatX, quatY, quatZ
        )
        pigeon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIOInputs) {
        val x = BaseStatusSignal.refreshAll(
            yaw, yawVelocity, accelerationX, accelerationY, accelerationZ,
            getMagFieldX, getMagFieldY, getMagFieldZ,
            quatW, quatX, quatY, quatZ
        )
        inputs.connected = x == StatusCode.OK
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.valueAsDouble)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)
        inputs.accelX = accelerationX.valueAsDouble
        inputs.accelY = accelerationY.valueAsDouble
        inputs.accelZ = accelerationZ.valueAsDouble
        inputs.quatW = quatW.valueAsDouble
        inputs.quatX = quatX.valueAsDouble
        inputs.quatY = quatY.valueAsDouble
        inputs.quatZ = quatZ.valueAsDouble
    }
}
