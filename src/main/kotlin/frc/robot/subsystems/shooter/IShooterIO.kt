package frc.robot.subsystems.shooter

import edu.wpi.first.units.measure.AngularVelocity
import jdk.jfr.Percentage

interface IShooterIO {
    fun setIntakePercent(percentage: Double)
    fun setShooterVelocity(velocity: AngularVelocity)
    fun stop()
}