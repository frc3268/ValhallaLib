package frc.lib.motor

import edu.wpi.first.wpilibj.motorcontrol.Talon

class AndyMarkTalonMotor(override val id: Int, val motor: Talon) : Motor {
    override fun setVoltage(voltage: Double) {
        motor.voltage = voltage;
    }

    override fun setPosition(position: Double) {
        TODO("Not yet implemented")
    }

    override fun setVelocity(velocity: Double) {
        TODO("Not yet implemented")
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
        motor.addFollower((follow as AndyMarkTalonMotor).motor)
    } // TODO: REMOVE

    /** Require Inversion */
    override fun forceInvert(invert: Boolean) {
        motor.inverted = invert;
    } // TODO: REMOVE
}