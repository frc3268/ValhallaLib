/*
HEY!
If you're planning on using the Swerve Drive Base in this library on your own robot,
make sure to edit these constants based on your own needs! Info on this may appear here later,
but as of now [[https://github.com/Team364/Base
FalconSwerve]] is a great resource
for most constants used in this library.
 */
package frc.lib.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.wheelDiameterMeters

object SwerveDriveConstants {
    data class ModuleConstants(
            val moduleNumber: Int,
            val angleOffset: Rotation2d,
            val driveMotorId: Int,
            val angleMotorId: Int,
            val encoderId: Int,
            val driveMotorReversed: Boolean,
            val angleMotorReversed: Boolean,
            val pidController: PIDController
    )

    object DriveMotor {
        const val gearRatio: Double = 12.1 / 1.0
        const val positionConversionFactorMetersPerRotation: Double = gearRatio
        const val velocityConversionFactorMetersPerSecond = positionConversionFactorMetersPerRotation / 60.0
    }

    object AngleMotor {
        //for some reason 10:1 delivers the most accurate results
        const val positionConversionFactorDegreesPerRotation = 6.2 / 1.0

    }

    object Encoder {
        const val INVERT: Boolean = false
        const val positionConversionFactorDegreesPerRotation: Double = 360.0
    }

    object DrivetrainConsts {        /* Drivetrain Constants */
        val trackWidthMeters = Units.inchesToMeters(30.0)
        val wheelBaseMeters = Units.inchesToMeters(30.0)

        const val wheelDiameterMeters = 0.1016

        const val openLoopRampRateSeconds: Double = 0.25

        /* Swerve Profiling Values */
        const val maxSpeedMetersPerSecond = 3.0
        const val maxAngularVelocityDegreesPerSecond = 100.0
        const val maxAngularAccelerationDegreesPerSecondSquared = 100.0
        const val maxAccelerationMetersPerSecondSquared = 5.0

        val xPIDController = PIDController(1.8, 0.0, 0.0)
        val yPIDController = PIDController(1.8, 0.0, 0.0)
        val thetaPIDController = ProfiledPIDController(0.03, 0.0, 0.0, TrapezoidProfile.Constraints(
            maxAngularVelocityDegreesPerSecond, maxAngularAccelerationDegreesPerSecondSquared))

        //in the order they appear in modules list
        //assuming that 0,0 is the center of the robot, and (+,+) means (left, front)
        val kinematics =
                SwerveDriveKinematics(
                        Translation2d(wheelBaseMeters / 2.0, -trackWidthMeters / 2.0),
                        Translation2d(wheelBaseMeters / 2.0, trackWidthMeters / 2.0),
                        Translation2d(-wheelBaseMeters / 2.0, trackWidthMeters / 2.0),
                        Translation2d(-wheelBaseMeters / 2.0, -trackWidthMeters / 2.0),


                        )
    }


    val modules = listOf(
            ModuleConstants(1, Rotation2d.fromDegrees(145.1), 1, 2, 0, true, false, PIDController(0.013, 0.00, 0.00)),
            ModuleConstants(2, Rotation2d.fromDegrees(9.1), 3, 4, 1, false, false, PIDController(0.013, 0.000, 0.0000)),
            ModuleConstants(3, Rotation2d.fromDegrees(140.0), 5, 6, 2, false, false, PIDController(0.013, 0.00, 0.000)),
            ModuleConstants(4, Rotation2d.fromDegrees(-90.0), 7, 8, 3, false, false, PIDController(0.013, 0.00, 0.000))
    )
}