package frc.lib.tankdrive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import java.util.function.DoubleSupplier

class TankJoystickDrive(
    private val drive: TankDriveSubsystem,
    private val forward: DoubleSupplier,
    private val rotation: DoubleSupplier,
) : Command() {

    private val tab: ShuffleboardTab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
    private val gain = tab.add("TankDrive gain", 1.0).entry

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
        drive.arcadeDrive(x, y, gain.get().double)
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