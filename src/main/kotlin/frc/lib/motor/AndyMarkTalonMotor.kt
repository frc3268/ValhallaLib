package frc.lib.motor

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.motorcontrol.Talon
import edu.wpi.first.wpilibj.Encoder;

class AndyMarkTalonMotor(
    override val id: Int,
    val motor: Talon,
    val pid: PIDController,
) : Motor {
    override fun setVoltage(voltage: Double) {
        motor.voltage = voltage;
    }

    override fun setPosition(position: Double) {
        //motor.set(pid.calculate());
    }

    override fun setVelocity(velocity: Double) {
        motor.set(velocity);
    }

    override fun getVelocityRPMMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getAppliedVoltage(): Double {
        return motor.voltage;
    }

    override fun getPositionDegreeMeasurement(): Double {
        TODO("Not yet implemented")
    }

    override fun getCurrentAmps(): DoubleArray {
        TODO("Not yet implemented")
    }

    override fun configure() {
        TODO("Not yet implemented")
    }

    override fun stop() {
        motor.stopMotor()
    }

    override fun close() {
        motor.close()
    }

    override fun reset() {
        TODO("Not yet implemented")
    }

    /** Follow the same type of motor */
    override fun setFollowSame(follow: Motor) {
        (follow as? AndyMarkTalonMotor)?.let { motor.addFollower((it).motor) }
    } // TODO: REMOVE

    /** Require Inversion */
    override fun forceInvert(invert: Boolean) {
        motor.inverted = invert;
    } // TODO: REMOVE
}