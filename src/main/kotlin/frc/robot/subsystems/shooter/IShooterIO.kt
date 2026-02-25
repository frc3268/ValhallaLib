package frc.robot.subsystems.shooter

interface IShooterIO {
    fun setIntakePercent(percentage: Double)
    fun setShooterPercent(percentage: Double)
    fun stop()
}