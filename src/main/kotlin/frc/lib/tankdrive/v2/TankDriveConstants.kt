package frc.lib.tankdrive.v2

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.util.Units

object TankDriveConstants {

    const val TANK_DRIVE_TAB = "Tankdrive"

    object TankConstants {
        // Not true...
        val WHEEL_BASE_METERS = Units.inchesToMeters(30.0)
        val kinematics =
            DifferentialDriveKinematics(
                0.295
            )
    }
}