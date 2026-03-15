package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.tankdrive.TankDriveSubsystem
import frc.lib.tankdrive.TankJoystickDrive
import frc.lib.tankdrive.legacy.TankDriveIOSparkMax
import frc.lib.tankdrive.v2.AlignToAprilTagTank
import frc.robot.Constants.GENERAL_TAB
import frc.robot.commands.Routines
import frc.robot.subsystems.shooter.ShooterIOSparkMax
import frc.robot.subsystems.shooter.ShooterSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since [Command]-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, [Command]s, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val generalTab = Shuffleboard.getTab(GENERAL_TAB)
    private val calibrationTab = Shuffleboard.getTab(Constants.CALIBRATION_TAB)
    private val driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)

    private val timerTelemetry = TimerTelemetry()

    val autoChooser = SendableChooser<Command>()
    private val autoWait = calibrationTab.addPersistent("Auto wait", 1.0).entry

    val tankDrive = TankDriveSubsystem(
        TankDriveIOSparkMax(),
        Pose2d()
    )

    val shooter = ShooterSubsystem(
        ShooterIOSparkMax(),
    )

//    val teleopCommand = SwerveJoystickDrive(
//        driveSubsystem,
//        { driverController.getRawAxis(1) },
//        { driverController.getRawAxis(0) },
//        { -driverController.getRawAxis(4) },
//        { true }
//    )

    val teleopCommand = TankJoystickDrive(
        tankDrive,
        { -driverController.getRawAxis(1) },
        { -driverController.getRawAxis(4) },
    )


//    fun goto(goal: FieldLocation): Command {
//        return SwerveAutoDrive(
//            {goal},
//            driveSubsystem
//        )
//    }


    /** The container for the robot. Contains subsystems, OI devices, and [Command]s.  */
    init {
        val rightChooser = SendableChooser<Boolean>()

        rightChooser.setDefaultOption("left", false)
        rightChooser.addOption("right", true)

        // get selected level with levelChooser.selected
//        if (Constants.mode == Constants.States.REAL) {
//
//        } else {
//
//            println("Warning: Simulated subsystems do not exist as no IOClass for them exists!")
//            println("Abandon all hope ye who debug here")
//        }


        autoChooser.addOption("Do Nothing", WaitCommand(3.0))
        autoChooser.setDefaultOption(
            "Move-Back Auto",
            Routines.basicBackAuto(tankDrive, shooter, autoWait.getDouble(1.0))
        )

//        if (Constants.mode == Constants.States.SIM) {
//            Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
//                .add(AlignToAprilTagCommand(driveSubsystem, { rightChooser.selected }))
//        }
//        driverController.povDown().onTrue(Routines.inchBack(driveSubsystem))
//        driverController.povUp().onTrue(Routines.inchForward(driveSubsystem))
//        driverController.povRight().onTrue(Routines.inchRight(driveSubsystem))
//        driverController.povLeft().onTrue(Routines.inchLeft(driveSubsystem))
//        driveSubsystem.defaultCommand = teleopCommand

        driverController.leftTrigger().onTrue(shooter.startIntake()).onFalse(shooter.stop()) // Toggle on false?
        driverController.b().onTrue(shooter.stop()) // Toggle on false?

        driverController.rightTrigger().onTrue(shooter.revUpAndStartShoot()).onFalse(shooter.stop())

        Shuffleboard.getTab(Constants.GENERAL_TAB)
            .add(AlignToAprilTagTank(tankDrive, { rightChooser.selected }))

        tankDrive.defaultCommand = teleopCommand

        generalTab
            .add("Autonomous Mode", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1)
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            // return WaitCommand(1.0)
            //fix
            return autoChooser.selected
        }
}
