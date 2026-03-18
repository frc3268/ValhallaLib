package frc.robot.subsystems.shooter

interface IShooterIO {
    fun setIntake(percentage: Double)
    fun setShooter(percentage: Double)
    fun setIntakeForShooter(percentage: Double)

    fun log()
    fun stop()
    open class Inputs {
        var shooterAppliedOutput = 0.0
        var intakeAppliedOutput = 0.0
    }
}