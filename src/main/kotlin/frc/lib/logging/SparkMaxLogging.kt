package frc.lib.logging

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog


class SparkMaxLogging : DogLog() {
    companion object {
        fun log(key: String, motor: SparkMax) {
            log("$key/AppliedOutput", motor.appliedOutput)
            log("$key/BusVoltage", motor.busVoltage)
            log("$key/isFollower", motor.isFollower)
            log("$key/outputCurrent", motor.outputCurrent, "amps")
            log("$key/setSpeed", motor.get())
            log("$key/motorTemperature", motor.motorTemperature)
            log("$key/encoder/position", motor.encoder.position)
            log("$key/encoder/velocity", motor.encoder.velocity)
        }
    }
}