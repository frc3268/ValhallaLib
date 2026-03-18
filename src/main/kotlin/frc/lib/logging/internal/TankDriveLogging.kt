package frc.lib.logging.internal

import dev.doglog.DogLog
import frc.lib.tankdrive.TankDriveSubsystem

/** Do not call outside of [TankDriveSubsystem.periodic] */
class TankDriveLogging : DogLog() {
    companion object {
        fun log(key: String, drive: TankDriveSubsystem) {
            log("$key/leftDistance", drive.io.getLeftDistance())
            log("$key/rightDistance", drive.io.getRightDistance())
            log("$key/pose", drive.getPose())
        }
    }
}