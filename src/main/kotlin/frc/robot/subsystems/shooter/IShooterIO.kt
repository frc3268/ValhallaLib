package frc.robot.subsystems.shooter

interface IShooterIO {
    fun setStorageIntake(percentage: Double)
    fun setShooter(percentage: Double)
    fun setShooterIntake(percentage: Double)

    fun stop()
}