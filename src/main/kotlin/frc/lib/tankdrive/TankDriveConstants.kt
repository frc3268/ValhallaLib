package frc.lib.tankdrive

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.util.Units

object TankDriveConstants {

    object TankConstants {
        // Not true...
        val WHEEL_BASE_METERS = Units.inchesToMeters(30.0)
        val kinematics =
            DifferentialDriveKinematics(
                WHEEL_BASE_METERS / 2.0
            )
    }
}