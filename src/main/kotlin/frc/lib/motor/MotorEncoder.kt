package frc.lib.motor

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage

interface MotorEncoder {
//    var velocity: LinearVelocity
//
//    var position: Double

    fun configurate()

    fun resetPosition(position: Double)
}