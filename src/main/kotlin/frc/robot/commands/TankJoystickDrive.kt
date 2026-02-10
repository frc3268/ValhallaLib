package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.math.MathUtil
import frc.lib.tankdrive.v2.TankDriveSubsystem
import java.util.function.*

import frc.robot.Constants

class TankJoystickDrive(
    private val drive: TankDriveSubsystem,
    private val forward: DoubleSupplier,
    private val rotation: DoubleSupplier,
    private val fieldOriented: BooleanSupplier
) : Command() {

    private var gain: Double = 1.0;

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        /* Get Values, Deadband, Convert to arcade drive components */
        val x: Double = MathUtil.applyDeadband(rotation.asDouble, Constants.OperatorConstants.STICK_DEADBAND)
        val y: Double = MathUtil.applyDeadband(forward.asDouble, Constants.OperatorConstants.STICK_DEADBAND)

        /* Drive */
        drive.arcadeDrive(x, y, gain)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }


}
