package frc.lib.tankdrive

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

    fun setLeftVelocity(velocity: Double) {
        left1.setVelocity(velocity)
    }

    fun setRightVelocity(velocity: Double) {
        right1.setVelocity(velocity)
    }

    fun setVelocity(leftVel: Double, rightVel: Double) {
        setLeftVelocity(leftVel)
        setRightVelocity(rightVel)
    }

    fun setVelocity(velocity: Double) {
        setVelocity(velocity, velocity)
    }


    fun stop() {
        left1.stop()
        right1.stop()
    }
}