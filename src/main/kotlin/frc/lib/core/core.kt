package frc.lib.core

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.FieldLocation
import frc.lib.core.robotCore.initCore
import frc.lib.swerve.SwerveDriveBase
import frc.robot.Constants
import frc.robot.Constants.GENERAL_TAB
import frc.robot.commands.Routines
import frc.robot.commands.SwerveAutoDrive
import frc.robot.commands.SwerveJoystickDrive

/** A Basic core singleton. *WARNING: You must call [initCore] before using or acessing this singleton!* */
object robotCore {
    /** The drive subsystem. Must be accessed after calling [initCore] */
    lateinit var driveSubsystem: SwerveDriveBase
    lateinit var driverController: CommandXboxController
    val autoChooser = SendableChooser<Command>()

    val GeneralTab = Shuffleboard.getTab(GENERAL_TAB)
    val CalibrationTab = Shuffleboard.getTab(Constants.CALIBRATION_TAB)


    fun goto(goal: FieldLocation): Command {
        return SwerveAutoDrive(
            { goal },
            driveSubsystem
        )
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            return autoChooser.selected
        }


    fun initCore() {
        // initialize HAL
        check(HAL.initialize(500, 0)) { "Failed to initialize. Terminating." }

        // report robot language as Kotlin
        // 6 Means kotlin in French
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, 6)

        driveSubsystem = SwerveDriveBase(Pose2d());
        driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

        driverController.povDown().onTrue(Routines.inchBack(driveSubsystem))
        driverController.povUp().onTrue(Routines.inchForward(driveSubsystem))
        driverController.povRight().onTrue(Routines.inchRight(driveSubsystem))
        driverController.povLeft().onTrue(Routines.inchLeft(driveSubsystem))



        autoChooser.addOption("Do nothing", WaitCommand(3.0))
        autoChooser.setDefaultOption(
            "Taxi",
            SwerveJoystickDrive(driveSubsystem, { 1.0 }, { 0.0 }, { 0.0 }, { false }).withTimeout(1.0)
        )
    }

    // Idk if this should be called in initCore or not
    fun initWidgets() {
        GeneralTab
            .add("Autonomous Mode", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)
    }
}
