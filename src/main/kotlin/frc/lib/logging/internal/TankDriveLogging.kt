package frc.lib.logging.internal

import dev.doglog.DogLog
import frc.lib.tankdrive.TankDriveSubsystem

/** Do not call outside of [TankDriveSubsystem.periodic] */
class TankDriveLogging : DogLog() {
    companion object {
        fun log(key: String, drive: TankDriveSubsystem) {
            log("$key/LeftDistance", drive.io.getLeftDistance())
            log("$key/RightDistance", drive.io.getRightDistance())
            log("$key/Pose", drive.getPose())
        }
    }
}