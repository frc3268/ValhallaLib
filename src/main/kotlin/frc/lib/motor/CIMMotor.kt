package frc.lib.motor

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController

class CIMMotor(
    val id: Int,
    var motorConfig: SparkMaxConfig = SparkMaxConfig(),
    val pidController: PIDController = PIDController(0.0, 0.0, 0.0)
) : Motor {

    private val motor = SparkMax(id, SparkLowLevel.MotorType.kBrushed)
    private var motorClosedLoop = motor.closedLoopController
    private var targetVelocity = 0.0

    init {
        configure()
    }

    override fun configure() {
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters)
    }

    override fun setVoltage(voltage: Double, arbitraryFeedForward: Double) {
        motor.setVoltage(voltage + arbitraryFeedForward)
    }

    override fun setVelocity(velocity: Double) {
        targetVelocity = velocity
        motor.set(velocity.coerceIn(-1.0, 1.0))
    }

    override fun getAppliedVoltage(): Double {
        return motor.busVoltage
    }

    override fun getPositionDegreeMeasurement(): Double {
        return 0.0
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
        targetVelocity = 0.0
    }

    override fun forceInvert() {
        motorConfig.inverted(true)
    }

    override fun follow(motor: Motor) {
        if (motor is CIMMotor) {
            motorConfig.follow(motor.motor)
            configure()
        }
    }
}
