package frc.lib.motor

import com.revrobotics.spark.ClosedLoopSlot
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.ControlType
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig

// We should use this class more fr fr
// Also not use as much PiD Controllers
class SparkMaxMotor(
    override val id: Int,
    var motorConfig: SparkMaxConfig = SparkMaxConfig()
) : Motor {

    val motor = SparkMax(id, SparkLowLevel.MotorType.kBrushless)
    var motorClosedLoop = motor.closedLoopController;

    init{
        configure()
    }

    override fun configure() {
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage)
    }

    override fun setPosition(position: Double) {
        motorClosedLoop.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0)
    }

    override fun setVelocity(velocity: Double) {
        motorClosedLoop.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1)
    }

    override fun getVelocityRPMMeasurement(): Double {
        return motor.getEncoder().velocity
    }

    override fun getAppliedVoltage(): Double {
        return motor.busVoltage
    }

    override fun getPositionDegreeMeasurement(): Double {
        return getAppliedVoltage() / 360
    }

    override fun getCurrentAmps(): DoubleArray {
        return doubleArrayOf(motor.outputCurrent)
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun close() {
        motor.close()
    }

    override fun reset() {
        motor.encoder.position = 0.0
    }
}
