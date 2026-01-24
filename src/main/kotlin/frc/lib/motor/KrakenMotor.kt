package frc.lib.motor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle


class KrakenMotor(
    val id: Int,
    val motorConfig: TalonFXConfiguration = TalonFXConfiguration(),
    var positionSlot: Slot0Configs = Slot0Configs(),
    var velocitySlot: Slot1Configs = Slot1Configs()
) : Motor {

    val motor = TalonFX(id, "rio")

    init{
        configure()
    }
    override fun configure() {
        motor.configurator.apply(motorConfig)
        motor.configurator.apply(positionSlot)
        motor.configurator.apply(velocitySlot)
    }

    override fun setVoltage(voltage: Double, arbitraryFeedForward: Double) {
        TODO("Not yet implemented")
    }

    override fun setPosition(position: Double) {
        motor.setPosition(position)
    }

    override fun setVelocity(velocity: Double) {
        val request = VelocityVoltage(velocity).withSlot(1);
        motor.setControl(request)
    }

    override fun getVelocityRPMMeasurement(): Double {
        return motor.velocity.valueAsDouble
    }

    override fun getAppliedVoltage(): Double {
        return motor.motorVoltage.valueAsDouble
    }

    override fun getPositionDegreeMeasurement(): Double {
        return getAppliedVoltage() / 360
    }

    override fun getCurrentAmps(): DoubleArray {
        return doubleArrayOf(motor.statorCurrent.valueAsDouble)
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun close() {
        motor.close()
    }

    override fun reset() {
        //not totally sure if this works as intended
        //as intended means that it just changes the value reported by encoder
        motor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
    }

    override fun forceInvert() {
        TODO("Finish this")
    }

    override fun follow(motor: Motor) {
        TODO("Not yet implemented")
    }
}