package frc.lib.motor

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.CurrentUnit
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import frc.robot.Constants

/** ***Easy Cross Motor API Interface.***
 *
 * **Inverse & ID** should be configured automatically in init based off of arguments.
 *
 * Other configs, **such as [PIDController]**, needs to be configured differently based off of implementations.
 */
interface Motor {

    /** ID of the motor. Should be unique for each motor */
    // val id: Int

    /** Should the motor be reversed? */
    // var inverse: Boolean

    val encoder: MotorEncoder

    /** Should we invert the motor output?*/
    var invert: Boolean

    fun setPercent(percent: Double)

    /** Run the motor at the specified [voltage]
     * @param[voltage] Specified voltage to run motor
     */
    fun setVoltage(voltage: Voltage, arbitraryFeedForward: Voltage = Volts.of(0.0))

    /** Move the motor to the specified [position]
     * @param[position] The position to set the motor to
     */
    // TODO: use units instead of generic position
    fun setPosition(position: Double)

    /** Move the motor to the specified [velocity]
     * @param[velocity] Specified velocity to set motor to
     */
    fun setVelocity(velocity: LinearVelocity)

    /** Get the velocity in rotations per minute
     * @return Current velocity in RPM
     */
    fun getVelocityRPMMeasurement(): Double
    fun getAppliedVoltage(): Double
    fun getPositionDegreeMeasurement(): Double
    fun getCurrentAmps(): CurrentUnit

    /** Configure the motor.
     *
     *  It is best to refrain from calling this function, as it is automatically called on init */
    fun configure()

    /** Stop the motor **/
    fun stop()

    /** Close the motor and free up resources. This should not be called unless necessary **/
    fun close()

    /** Zero everything **/
    fun reset()

    /** Should this motor be forced to invert?*/
    fun forceInvert()

    /** Sets this motor to follow [motor].*/
    fun follow(motor: Motor): Boolean

    /** Calls a periodic function based off of current constants state */
    fun autoPeriodic() {
        when (Constants.mode) {
            Constants.States.REAL -> realPeriodic()
            Constants.States.SIM -> simulationPeriodic()
            Constants.States.REPLAY -> simulationPeriodic()
        }
        allPeriodic();
    }

    /** Do not call this function directly! Call autoPeriodic instead! */
    fun allPeriodic() {}

    /** Do not call this function directly! Call autoPeriodic instead! */
    fun realPeriodic() {}

    /** Do not call this function directly! Call autoPeriodic instead! */
    fun simulationPeriodic() {}
}
