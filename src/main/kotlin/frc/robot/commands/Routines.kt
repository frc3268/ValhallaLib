package frc.robot.commands

import frc.lib.swerve.SwerveDriveBase

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

}