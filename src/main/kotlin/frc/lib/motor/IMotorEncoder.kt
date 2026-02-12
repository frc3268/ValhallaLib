package frc.lib.motor

interface IMotorEncoder {
//    var velocity: LinearVelocity
//
//    var position: Double

    fun configurate()

    fun resetPosition(position: Double)
}