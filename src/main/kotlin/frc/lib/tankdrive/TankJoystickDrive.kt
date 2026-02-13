package frc.lib.tankdrive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import java.util.function.DoubleSupplier

class TankJoystickDrive(
    private val drive: TankDriveSubsystem,
    private val forward: DoubleSupplier,
    private val rotation: DoubleSupplier,
) : Command() {

    private val tab: ShuffleboardTab = Shuffleboard.getTab(Constants.CALIBRATION_TAB)
    private val gain = tab.addPersistent("TankDrive gain", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(mapOf("min" to 0, "max" to 10)).entry

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
        drive.arcadeDrive(x, y, gain.getDouble(1.0)).schedule()
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