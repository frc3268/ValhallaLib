package frc.lib.tankdrive

import edu.wpi.first.units.measure.AngularVelocity

interface ITankDriveIO {
    fun setLeftVelocity(velocity: AngularVelocity)
    fun setRightVelocity(velocity: AngularVelocity)
    fun setVelocity(leftVel: AngularVelocity, rightVel: AngularVelocity)
    fun setVelocityBoth(velocity: AngularVelocity)
    fun getLeftDistance(): Double
    fun getRightDistance(): Double
    fun stop()

    fun periodic() {}
    fun simulationPeriodic() {}
}