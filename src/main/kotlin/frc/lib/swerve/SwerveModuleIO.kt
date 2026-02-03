package frc.lib.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface SwerveModuleIO {
    @AutoLog
    open class ModuleIOInputs {
        var drivePositionMeters: Double = 0.0
        var driveVelocityMetersPerSec: Double = 0.0

        var turnAbsolutePosition: Rotation2d = Rotation2d()
        var turnPosition: Rotation2d = Rotation2d()
        var turnVelocityRadPerSec: Double = 0.0
    }

    val turnPIDController: PIDController

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs)

    /** Run the drive motor at the specified voltage.  */
    fun setDriveVoltage(volts: Double)

    /** Run the turn motor at the specified voltage.  */
    fun setTurnVoltage(volts: Double)

    /** Enable or disable brake mode on the drive motor.  */
    fun setDriveBrakeMode(enable: Boolean)

    /** Enable or disable brake mode on the turn motor.  */
    fun setTurnBrakeMode(enable: Boolean)

    fun reset()
}

class ModuleIOInputsAutoLogged : SwerveModuleIO.ModuleIOInputs(), LoggableInputs {
    override fun toLog(table: LogTable) {
        table.put("drivePositionMeters", drivePositionMeters)
        table.put("driveVelocityMetersPerSec", driveVelocityMetersPerSec)
        table.put("turnAbsolutePosition", turnAbsolutePosition)
        table.put("turnPosition", turnPosition)
        table.put("turnVelocityRadPerSec", turnVelocityRadPerSec)
    }

    override fun fromLog(table: LogTable) {
        table.get("drivePositionMeters", drivePositionMeters)
        table.get("driveVelocityMetersPerSec", driveVelocityMetersPerSec)
        table.get("turnAbsolutePosition", turnAbsolutePosition)
        table.get("turnPosition", turnPosition)
        table.get("turnVelocityRadPerSec", turnVelocityRadPerSec)
    }
}
