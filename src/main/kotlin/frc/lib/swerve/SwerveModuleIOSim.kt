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
package frc.lib.swerve

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.lib.swerve.SwerveModuleIO.ModuleIOInputs

/**
 * Physics sim implementation of module IO.
 *
 *
 * Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
class SwerveModuleIOSim(val index: Int) : SwerveModuleIO {
    private val driveSim = DCMotorSim(createDCMotorSystem(DCMotor.getNEO(10), 0.025, 6.75), DCMotor.getNEO(10))
    private val turnSim = DCMotorSim(createDCMotorSystem(DCMotor.getNEO(10), 0.004, 150.0 / 7.0), DCMotor.getNEO(10))

    private val turnAbsoluteInitPosition = Rotation2d(Math.random() * 2.0 * Math.PI)
    private var driveAppliedVolts = 0.0
    private var turnAppliedVolts = 0.0
    override val turnPIDController: PIDController = PIDController(0.009, 0.0, 0.0)

    override fun updateInputs(inputs: ModuleIOInputs) {
        driveSim.update(LOOP_PERIOD_SECS)
        turnSim.update(LOOP_PERIOD_SECS)

        //FIX
        inputs.drivePositionMeters = inputs.drivePositionMeters + driveSim.angularVelocityRadPerSec * 0.002
        inputs.driveVelocityMetersPerSec = driveSim.angularVelocityRadPerSec

        inputs.turnAbsolutePosition =
            Rotation2d(turnSim.angularPositionRad).plus(turnAbsoluteInitPosition)
        inputs.turnPosition = inputs.turnPosition + Rotation2d.fromRotations(turnSim.angularVelocityRadPerSec * 0.02)
        inputs.turnVelocityRadPerSec = turnSim.angularVelocityRadPerSec
    }

    override fun setDriveVoltage(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        driveSim.setInputVoltage(driveAppliedVolts)
    }

    override fun setTurnVoltage(volts: Double) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        turnSim.setInputVoltage(turnAppliedVolts)
    }

    override fun setDriveBrakeMode(enable: Boolean) {
        //no...
    }

    override fun setTurnBrakeMode(enable: Boolean) {
        //no...
    }

    override fun reset() {
        //no...
    }

    companion object {
        private const val LOOP_PERIOD_SECS = 0.02
    }
}