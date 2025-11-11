package frc.lib.motor

import edu.wpi.first.math.controller.PIDController
import frc.robot.Constants

/** ***Easy Cross Motor API Interface.***
 *
 * **Inverse & ID** should be configured automatically in init based off of arguments.
 *
 * Other configs, **such as [PIDController]**, needs to be configured differently based off of implementations.
 */
interface Motor {

    /** ID of the motor. Should be unique for each motor */
    val id: Int

    /** Should the motor be reversed? */
//    var inverse: Boolean

    /** Run the motor at the specified [voltage]
     * @param[voltage] Specified voltage to run motor
     */
    fun setVoltage(voltage: Double)

    /** Move the motor to the specified [position]
     * @param[position] The position to set the motor to
     */
    fun setPosition(position: Double)

    /** Move the motor to the specified [velocity]
     * @param[velocity] Specified velocity to set motor to
     */
    fun setVelocity(velocity: Double)

    /** Get the velocity in rotations per minute
     * @return Current velocity in RPM
     */
    fun getVelocityRPMMeasurement(): Double
    fun getAppliedVoltage(): Double
    fun getPositionDegreeMeasurement(): Double
    fun getCurrentAmps(): DoubleArray

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

    /** Calls a periodic function based off of current constants state */
    fun autoPeriodic() {
        when (Constants.mode) {
            Constants.States.REAL -> realPeriodic()
            Constants.States.SIM -> simulationPeriodic()
            Constants.States.REPLAY -> simulationPeriodic()
        }
    }

    /** Do not call this function directly! Call autoPeriodic instead! */
    fun realPeriodic() {}

    /** Do not call this function directly! Call autoPeriodic instead! */
    fun simulationPeriodic() {}

    /** Follow the same type of motor */
    fun setFollowSame(follow: Motor) {} // TODO: REMOVE

    /** Require Inversion */
    fun forceInvert(invert: Boolean) {} // TODO: REMOVE
}
