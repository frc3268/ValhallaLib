package frc.lib.logging

import dev.doglog.DogLog
import edu.wpi.first.wpilibj2.command.Command

class CommandLogging : DogLog() {
    companion object {
        fun log(key: String, command: Command) {
            log("$key/subsystem", command.subsystem)
            log("$key/isFinished", command.isFinished)
            log("$key/isScheduled", command.isScheduled)
        }
    }
}