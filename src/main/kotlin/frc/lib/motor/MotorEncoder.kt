package frc.lib.motor

interface MotorEncoder {
//    var velocity: LinearVelocity
//
//    var position: Double

    fun configurate()

    fun resetPosition(position: Double)
}