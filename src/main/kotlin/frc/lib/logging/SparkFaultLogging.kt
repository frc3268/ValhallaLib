package frc.lib.logging

import com.revrobotics.spark.SparkBase
import dev.doglog.DogLog
import frc.robot.Constants

class SparkFaultLogging : DogLog() {
    companion object {
        fun log(key: String, fault: SparkBase.Faults) {
            if (!Constants.AutoLog.ENABLE_REV_AUTOLOG) {
                log("$key/Can", fault.can)
                log("$key/MotorType", fault.motorType)
                log("$key/Sensor", fault.sensor)
                log("$key/Other", fault.other)
                log("$key/Firmware", fault.firmware)
                log("$key/EscEeprom", fault.escEeprom)
                log("$key/GateDriver", fault.gateDriver)
                log("$key/Temperature", fault.temperature)
            }
        }
    }
}