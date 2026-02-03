package frc.lib.motor.sparkmax

import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.CurrentUnit
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import frc.lib.motor.Motor
import frc.lib.motor.MotorEncoder

class SparkMaxMotor(
    override val encoder: SparkMaxEncoder,
    override var invert: Boolean
) : Motor {
    override fun setVoltage(
        voltage: Voltage,
        arbitraryFeedForward: Voltage
    ) {
        TODO("Not yet implemented")
    }

    override fun setPosition(position: Double) {
        TODO("Not yet implemented")
    }

    override fun setVelocity(velocity: LinearVelocity) {
        TODO("Not yet implemented")
    }

    override fun getVelocityRPMMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getAppliedVoltage(): Double {
        TODO("Not yet implemented")
    }

    override fun getPositionDegreeMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getCurrentAmps(): CurrentUnit {
        TODO("Not yet implemented")
    }

    override fun configure() {
        TODO("Not yet implemented")
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

    override fun close() {
        TODO("Not yet implemented")
    }

    override fun reset() {
        TODO("Not yet implemented")
    }

    override fun forceInvert() {

    }

    // TODO: Check if the right function gets called
    fun <SparkMaxMotor> follow(motor: SparkMaxMotor): Boolean {
        TODO("Not yet implemented")
    }

    override fun <T : Motor> follow(motor: T): Boolean {
        TODO("Not yet implemented")
    }

}