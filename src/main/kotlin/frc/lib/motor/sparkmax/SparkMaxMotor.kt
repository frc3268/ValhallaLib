package frc.lib.motor.sparkmax

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkClosedLoopController
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.CurrentUnit
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import frc.lib.motor.IMotor


class SparkMaxMotor(
    override val encoder: SparkMaxEncoder,
    val id: Int,
    val motorType: SparkLowLevel.MotorType,
    val pidController: PIDController = PIDController(0.0, 0.0, 0.0)
) : IMotor {

    override var invert: Boolean = false;


    var sparkMax: SparkMax = SparkMax(id, motorType)
    var controller: SparkClosedLoopController = sparkMax.getClosedLoopController()
    var config: SparkMaxConfig = SparkMaxConfig()

    init {
        config.closedLoop.p(pidController.p).i(pidController.i).d(pidController.d);
    }

    override fun setPercent(percent: Double) {
        sparkMax.set(percent);
    }

    override fun setVoltage(
        voltage: Voltage,
        arbitraryFeedForward: Voltage
    ) {
        sparkMax.setVoltage(voltage)
    }

    override fun setPosition(position: Double) {
        controller.setReference(position, SparkBase.ControlType.kPosition)
    }

    override fun setVelocity(velocity: AngularVelocity) {
        controller.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
    }

    override fun getVelocityRPMMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getAppliedVoltage(): Double = sparkMax.appliedOutput

    override fun getPositionDegreeMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getCurrentAmps(): CurrentUnit {
        TODO("Not yet implemented")
    }

    override fun configure() {
        sparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    }

    override fun stop() {
        sparkMax.stopMotor()
    }

    override fun close() {
        sparkMax.close()
    }

    override fun reset() {
        TODO("Not yet implemented")
    }

    override fun forceInvert() {

    }

    override fun follow(motor: IMotor): Boolean {
        if (motor is SparkMaxMotor) {
            return true;
        } else {
            assert(false)
            return false
        }
    }

}