package frc.lib.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d


interface SwerveModuleIO {
    open class ModuleIOInputs {
        var drivePositionMeters: Double = 0.0
        var driveVelocityMetersPerSec: Double = 0.0

        var turnAbsolutePosition: Rotation2d = Rotation2d()
        var turnPosition: Rotation2d = Rotation2d()
        var turnVelocityRadPerSec: Double = 0.0
    }

    val turnPIDController: PIDController

    /** Run the drive motor at the specified voltage.  */
    fun setDriveVoltage(volts: Double)

    /** Run the turn motor at the specified voltage.  */
    fun setTurnVoltage(volts: Double)

    /** Enable or disable brake mode on the drive motor.  */
    fun setDriveBrakeMode(enable: Boolean)

    /** Enable or disable brake mode on the turn motor.  */
    fun setTurnBrakeMode(enable: Boolean)

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs)

    fun reset()
}

class ModuleIOInputsAutoLogged : SwerveModuleIO.ModuleIOInputs() { }