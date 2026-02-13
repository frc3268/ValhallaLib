package frc.robot.subsystems.shooter

import edu.wpi.first.units.measure.AngularVelocity

interface IShooterIO {
    fun setIntakeVelocity(velocity: AngularVelocity)
    fun setShooterVelocity(velocity: AngularVelocity)
    fun stop()
}