package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveJoystickDrive
import frc.lib.tankdrive.TankDriveSubsystem
import frc.lib.toRPM
import frc.lib.wait
import frc.robot.subsystems.shooter.ShooterSubsystem

/** Setup shuffleboard buttons
 *
 * TODO: Put controller stuff in here */

/** High level routines / commands consisting of lower level commands */
object Routines {

    fun inchForward(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { 0.1 }, { 0.0 }, { 0.0 }, { false }).withTimeout(0.5)

    fun inchBack(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { -0.1 }, { 0.0 }, { 0.0 }, { false }).withTimeout(0.5)

    fun inchLeft(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { 0.0 }, { -0.1 }, { 0.0 }, { false }).withTimeout(0.5)

    fun inchRight(drive: SwerveDriveBase) =
        SwerveJoystickDrive(drive, { 0.0 }, { 0.1 }, { 0.0 }, { false }).withTimeout(0.5)

    fun centerAuto(drive: TankDriveSubsystem, shoot: ShooterSubsystem): Command = SequentialCommandGroup(
        drive.driveForward(1.0.toRPM()),
        0.4.wait(),
        drive.stop(),
        shoot.revUpAndStartShoot(),
        10.0.wait(),
        shoot.stop()
    )
}