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
package frc.robot.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MusicTone
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Dimensionless
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Velocity
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs

class ShooterIOTalonFX : ShooterIO {
    private val leader = TalonFX(40)
    private val follower = TalonFX(41)

    private val leaderPosition: StatusSignal<Double> = leader.position
    private val leaderVelocity: StatusSignal<Double> = leader.velocity
    private val leaderAppliedVolts: StatusSignal<Double> = leader.motorVoltage
    private val leaderCurrent: StatusSignal<Double> = leader.statorCurrent
    private val leaderDeviceTemp: StatusSignal<Double> = leader.deviceTemp
    private val leaderAncillaryDeviceTemp: StatusSignal<Double> = leader.ancillaryDeviceTemp
    private val leaderProcessorTemp: StatusSignal<Double> = leader.processorTemp
    private val followerAppliedVolts: StatusSignal<Double> = follower.motorVoltage
    private val followerCurrent: StatusSignal<Double> = follower.statorCurrent
    private val followerDeviceTemp: StatusSignal<Double> = follower.deviceTemp
    private val followerAncillaryDeviceTemp: StatusSignal<Double> = follower.ancillaryDeviceTemp
    private val followerProcessorTemp: StatusSignal<Double> = follower.processorTemp

    init {
        val config = TalonFXConfiguration()
        config.Audio.AllowMusicDurDisable = true
        config.Audio.BeepOnConfig = false
        config.Audio.BeepOnBoot = false
        config.CurrentLimits.StatorCurrentLimit = 60.0
        config.CurrentLimits.StatorCurrentLimitEnable = true
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast
        leader.configurator.apply(config)
        follower.configurator.apply(config)
        follower.setControl(Follower(leader.deviceID, false))

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent
        )
        BaseStatusSignal.setUpdateFrequencyForAll(
            20.0,
            leaderDeviceTemp,
            leaderAncillaryDeviceTemp,
            leaderProcessorTemp,
            followerDeviceTemp,
            followerAncillaryDeviceTemp,
            followerProcessorTemp
        )
        leader.optimizeBusUtilization()
        follower.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ShooterIOInputs) {
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent, leaderDeviceTemp,
            leaderAncillaryDeviceTemp, leaderProcessorTemp, followerDeviceTemp, followerAncillaryDeviceTemp,
            followerProcessorTemp
        )
        inputs.flywheelPositionRad = leaderPosition.valueAsDouble * GEAR_RATIO
        inputs.flywheelVelocityRadPerSec =
            Units.rotationsToRadians(leaderVelocity.valueAsDouble) * GEAR_RATIO
        inputs.flywheelAppliedVolts = leaderAppliedVolts.valueAsDouble
        inputs.flywheelVoltages = doubleArrayOf(leaderAppliedVolts.valueAsDouble, followerAppliedVolts.valueAsDouble)
        inputs.flywheelCurrentAmps =
            doubleArrayOf(leaderCurrent.valueAsDouble, followerCurrent.valueAsDouble)
        inputs.flywheelTemperature
        = doubleArrayOf(leaderDeviceTemp.valueAsDouble, followerDeviceTemp.valueAsDouble)
        inputs.flywheelAncillaryTemperature
        = doubleArrayOf(leaderAncillaryDeviceTemp.valueAsDouble, followerAncillaryDeviceTemp.valueAsDouble)
        inputs.flywheelProcessorTemperature
        = doubleArrayOf(leaderProcessorTemp.valueAsDouble, followerProcessorTemp.valueAsDouble)
    }

    fun playTone(tone: Measure<Velocity<Dimensionless?>?>) {
        val musicTone = MusicTone(tone.`in`(edu.wpi.first.units.Units.Value.per(edu.wpi.first.units.Units.Second)))
        leader.setControl(musicTone)
    }

    override fun setFlywheelVoltage(volts: Double) {
        leader.setVoltage(volts)
    }

    override fun flywheelStop() {
        leader.stopMotor()
    }

    companion object {
        private const val GEAR_RATIO = 1.5
    }
}
