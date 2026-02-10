package frc.lib.swerve

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.pow

class SwerveJoystickDrive(
    private val drive: SwerveDriveBase,
    private val translationX: DoubleSupplier,
    private val translationY: DoubleSupplier,
    private val rotation: DoubleSupplier,
    private val fieldOriented: BooleanSupplier
) : Command() {


    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        /* Get Values, Deadband, Convert to speeds */
        val xSpeed: Double =
            sigmoid(MathUtil.applyDeadband(translationX.asDouble, Constants.OperatorConstants.STICK_DEADBAND))
        val ySpeed: Double =
            sigmoid(MathUtil.applyDeadband(translationY.asDouble, Constants.OperatorConstants.STICK_DEADBAND))
        val turnSpeed: Double = MathUtil.applyDeadband(
            rotation.asDouble,
            Constants.OperatorConstants.STICK_DEADBAND
        ) * SwerveDriveConstants.DriveTrainConstants.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND

        /* Drive */
        drive.setModuleStates(
            drive.constructTeleopModuleStatesFromChassisSpeeds(
                xSpeed,
                ySpeed,
                turnSpeed,
                fieldOriented.asBoolean
            )
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }


    //sigmoid sigmoid on the wall...
    fun sigmoid(x: Double): Double {
        return (10 * (1 / (1 + Math.E.pow(-1.75 * x)))) - 5
    }
}