package frc.lib.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.Constants
import kotlin.math.abs

/*
Props: drive motor, drive encoder, angle motor, angle encoder, absolute encoder
Get: Cancoder measurement, Module state(velocity) and position
Set: Module state
 */
class SwerveModule(val io: SwerveModuleIO, val index: Int) {
    private val inputs = ModuleIOInputsAutoLogged()
    private val ShuffleboardTab = Shuffleboard.getTab("Swerve Module " + index)
    val headingE = ShuffleboardTab.add("heading", 0.0).getEntry()
    val setpointE = ShuffleboardTab.add("setpoint", "nothing").getEntry()

    val turnController: PIDController = io.turnPIDController

    private var lastPosition: SwerveModulePosition = SwerveModulePosition()
    var delta: SwerveModulePosition = SwerveModulePosition()

    init {
        turnController.enableContinuousInput(-180.0, 180.0)
    }

    fun update() {
        io.updateInputs(inputs)
        delta = SwerveModulePosition(
            getPosition().distanceMeters
                    - lastPosition.distanceMeters,
            getPosition().angle
        );
        lastPosition = getPosition()
        headingE.setDouble(inputs.turnPosition.degrees)
    }

    fun resetToAbsolute() {
        io.reset()
    }

    fun getState() = SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.turnPosition)
    fun getPosition() = SwerveModulePosition(inputs.drivePositionMeters, inputs.turnPosition)

    fun setDesiredState(desiredState: SwerveModuleState) {
        if (abs(desiredState.speedMetersPerSecond) < 0.01) {
            stop()
            return
        }
        desiredState.optimize(getState().angle)
        desiredState.cosineScale(getState().angle)
        var error = (desiredState.angle - getState().angle).degrees

        setpointE.setString("error; " + error + "currentState: " + desiredState.angle.degrees)
        io.setDriveVoltage((desiredState.speedMetersPerSecond / SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND) * 12.0)
        if (Constants.mode == Constants.States.REAL) {
            io.setTurnVoltage(turnController.calculate(error, 0.0) * 12.0)
        } else {
            io.setTurnVoltage(turnController.calculate(getState().angle.degrees, desiredState.angle.degrees) * 12.0)
        }
    }

    fun stop() {
        io.setDriveVoltage(0.0)
        io.setTurnVoltage(0.0)
    }
}