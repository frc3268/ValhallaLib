package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveJoystickDrive
import frc.lib.tankdrive.TankDriveSubsystem
import frc.lib.wait
import frc.robot.subsystems.shooter.ShooterSubsystem

///** Setup [edu.wpi.first.wpilibj.shuffleboard.Shuffleboard] buttons
// *
// * TODO: Put controller stuff in here */

/** High level routines / [Command]s consisting of lower level [Command]s */
object Routines {

    fun inchForward(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { 0.1 }, { 0.0 }, { 0.0 }, { false }).withTimeout(0.5)

    fun inchBack(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { -0.1 }, { 0.0 }, { 0.0 }, { false }).withTimeout(0.5)

    fun inchLeft(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { 0.0 }, { -0.1 }, { 0.0 }, { false }).withTimeout(0.5)

    fun inchRight(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { 0.0 }, { 0.1 }, { 0.0 }, { false }).withTimeout(0.5)

    // Max: 20 seconds
    fun basicBackAuto(drive: TankDriveSubsystem, shoot: ShooterSubsystem, driveWait: Double): Command = SequentialCommandGroup(
        drive.commandArcadeDrive(0.0, 1.0, 0.35),
        driveWait.wait(),
        drive.stop(),
        shoot.revUpAndStartShoot(),
        10.0.wait(),
        shoot.stop()
    )
}