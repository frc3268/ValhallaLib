package frc.lib.logging

import com.revrobotics.spark.SparkBase
import dev.doglog.DogLog
import frc.robot.Constants

class SparkFaultLogging : DogLog() {
    companion object {
        fun log(key: String, fault: SparkBase.Faults) {
            if (!Constants.AutoLog.ENABLE_REV_AUTOLOG) {
                log("$key/can", fault.can)
                log("$key/motorType", fault.motorType)
                log("$key/sensor", fault.sensor)
                log("$key/other", fault.other)
                log("$key/firmware", fault.firmware)
                log("$key/escEeprom", fault.escEeprom)
                log("$key/gateDriver", fault.gateDriver)
                log("$key/temperature", fault.temperature)
            }
        }
    }
}