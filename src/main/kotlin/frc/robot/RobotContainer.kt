package frc.robot

import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.motorcontrol.Talon
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.FieldLocation
import frc.lib.FieldPositions
import frc.lib.motor.sparkmax.SparkMaxEncoder
import frc.lib.motor.sparkmax.SparkMaxMotor
import frc.lib.swerve.SwerveDriveBase
import frc.lib.tankdrive.TankDriveIO
import frc.lib.tankdrive.TankDriveSubsystem
import frc.robot.commands.AlignToAprilTagCommand
import frc.robot.commands.Routines
import frc.robot.commands.SwerveAutoDrive
import frc.robot.commands.SwerveJoystickDrive

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val GeneralTab = Shuffleboard.getTab("General")
    private val CalibrationTab = Shuffleboard.getTab(Constants.CALIBRATION_TAB)
    private val driverController = CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT)



    val autochooser = SendableChooser<Command>()

//    val teleopCommand = SwerveJoystickDrive(
//        driveSubsystem,
//        { driverController.getRawAxis(1) },
//        { driverController.getRawAxis(0) },
//        { -driverController.getRawAxis(4) },
//        { true }
//    )


//    fun goto(goal: FieldLocation): Command {
//        return SwerveAutoDrive(
//            {goal},
//            driveSubsystem
//        )
//    }

    
    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {


        var tankdrive = TankDriveSubsystem(TankDriveIO(
            SparkMaxMotor(SparkMaxEncoder(), 1, SparkLowLevel.MotorType.kBrushless),
            SparkMaxMotor(SparkMaxEncoder(), 2, SparkLowLevel.MotorType.kBrushless),
            SparkMaxMotor(SparkMaxEncoder(), 3, SparkLowLevel.MotorType.kBrushless),
            SparkMaxMotor(SparkMaxEncoder(), 4, SparkLowLevel.MotorType.kBrushless),
        ))

        val rightChooser = SendableChooser<Boolean>()

        rightChooser.setDefaultOption("left", false)
        rightChooser.addOption("right", true)



        if (Constants.mode == Constants.States.REAL) {


        } else {

            println("Warning: Simulated subsystems do not exist as no IOClass for them exists!")
            println("Abandon all hope ye who debug here")
        }

        val rbChooser = SendableChooser<Command>()

        driverController.leftTrigger().onTrue(tankdrive.driveForward(Units.FeetPerSecond.of(5.0)))
        driverController.rightTrigger().onTrue(tankdrive.driveForward(Units.FeetPerSecond.of(5.0)))

//        rbChooser.setDefaultOption(
//            "Align to April Tag",
//            AlignToAprilTagCommand(driveSubsystem, { rightChooser.selected })
//        )
//        rbChooser.addOption("Align to Source Left", goto(FieldPositions.sourceLeft))
//        rbChooser.addOption("Align to Source Right", goto(FieldPositions.sourceRight))
//
//        if (Constants.mode == Constants.States.SIM) {
//            Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
//                .add(AlignToAprilTagCommand(driveSubsystem, { rightChooser.selected }))
//        }
//        driverController.povDown().onTrue(Routines.inchBack(driveSubsystem))
//        driverController.povUp().onTrue(Routines.inchForward(driveSubsystem))
//        driverController.povRight().onTrue(Routines.inchRight(driveSubsystem))
//        driverController.povLeft().onTrue(Routines.inchLeft(driveSubsystem))
//        driveSubsystem.defaultCommand = teleopCommand

    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */
    val autonomousCommand: Command
        get() {
            return WaitCommand(1.0)
            //fix
            //autochooser.selected
        }
}
