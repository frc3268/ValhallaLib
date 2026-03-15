package frc.robot.subsystems.shooter

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IShooterIO {
    fun setStorageIntake(percentage: Double)
    fun setShooter(percentage: Double)
    fun setShooterIntake(percentage: Double)

    fun stop()
    open class Inputs {
        var shooterAppliedOutput = 0.0
        var intakeAppliedOutput = 0.0
    }
    class LoggedInputs : Inputs(), LoggableInputs {
        override fun toLog(table: LogTable) {
            table.put("shooterVelocity", shooterAppliedOutput)
            table.put("intakeVelocity", intakeAppliedOutput)
        }

        override fun fromLog(table: LogTable) {
            table.get("shooterVelocity", shooterAppliedOutput)
            table.get("intakeVelocity", intakeAppliedOutput)
        }
    }

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: Inputs)
}