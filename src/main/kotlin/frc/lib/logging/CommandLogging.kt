package frc.lib.logging

import dev.doglog.DogLog
import edu.wpi.first.wpilibj2.command.Command

class CommandLogging : DogLog() {
    companion object {
        fun log(key: String, command: Command) {
            log("$key/Subsystem", command.subsystem)
            log("$key/IsFinished", command.isFinished)
            log("$key/IsScheduled", command.isScheduled)
        }
    }
}