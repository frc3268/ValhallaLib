package frc.lib.motor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.DCMotorSim


class KrakenSimMotor(
    val id: Int,
    val motorConfig: TalonFXConfiguration = TalonFXConfiguration(),
    var positionSlot: Slot0Configs = Slot0Configs(),
    var velocitySlot: Slot1Configs = Slot1Configs()
) : Motor {

    val motor = TalonFX(id, "rio")


    val motorDC = DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.005, // I don't know what these funny numbers mean. TODO: Can someone please fill these in?
            1.0),
        DCMotor.getNEO(1),
    )


    init{
        configure()
    }
    override fun configure() {
        motor.configurator.apply(motorConfig)
        motor.configurator.apply(positionSlot)
        motor.configurator.apply(velocitySlot)
    }

    override fun setVoltage(voltage: Double, arbitraryFeedForward: Double) {
        motor.setVoltage(voltage)
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
        // means that it just changes the value reported by encoder
        motor.setPosition(Angle.ofRelativeUnits(0.0, Units.Degree))
    }

    override fun forceInvert() {
        TODO("Not yet implemented")
    }

    override fun follow(motor: Motor) {
        TODO("Not yet implemented")
    }

    override fun simulationPeriodic() {
        var motorSim = motor.simState
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        var motorVoltage = motorSim.motorVoltage;

        // Use the motor voltage to calculate new position and velocity
        // Using WPILib's DCMotorSim class for physics simulation
        motorDC.inputVoltage = motorVoltage;
        motorDC.update(0.020); // Assume 20 ms loop time

        // I don't know what these funny numbers mean. TODO: Can someone please fill these in?
        motorSim.setRawRotorPosition(
            1 * motorDC.angularPositionRotations
        );
        motorSim.setRotorVelocity(
            1 * motorDC.angularVelocityRadPerSec
        );
    }
}