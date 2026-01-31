package frc.robot.commands

import frc.lib.swerve.SwerveDriveBase

/** Setup shuffleboard buttons
 *
 * TODO: Put controller stuff in here */

/** High level routines / commands consisting of lower level commands */
object Routines {

    /** Moves inch forward */
    fun inchForward(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {0.1}, {0.0}, {0.0}, {false}).withTimeout(0.5)

    /** Moves inch backward */
    fun inchBack(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {-0.1}, {0.0}, {0.0}, {false}).withTimeout(0.5)

    /** Moves inch left */
    fun inchLeft(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {0.0}, {-0.1}, {0.0}, {false}).withTimeout(0.5)

    /** Moves inch right */
    fun inchRight(drive: SwerveDriveBase) = SwerveJoystickDrive(drive, {0.0}, {0.1}, {0.0}, {false}).withTimeout(0.5)

}