package frc.lib.tankdrive

import edu.wpi.first.units.measure.LinearVelocity
import frc.lib.motor.Motor

class TankDriveIO(
    val left1: Motor, val left2: Motor,
    val right1: Motor, val right2: Motor
) {


    init {
        left2.follow(left1)
        right2.follow(right1)
        right1.forceInvert()
        right1.configure()
    }

    fun setLeftVelocity(velocity: LinearVelocity) {
        left1.setVelocity(velocity)
    }

    fun setRightVelocity(velocity: LinearVelocity) {
        right1.setVelocity(velocity)
    }

    fun setVelocity(leftVel: LinearVelocity, rightVel: LinearVelocity) {
        setLeftVelocity(leftVel)
        setRightVelocity(rightVel)
    }

    fun setVelocityBoth(velocity: LinearVelocity) {
        setVelocity(velocity, velocity)
    }

    fun stop() {
        left1.stop()
        right1.stop()
    }
}