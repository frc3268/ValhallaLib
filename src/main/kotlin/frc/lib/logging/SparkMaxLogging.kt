package frc.lib.logging

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog


class SparkMaxLogging : DogLog() {
    fun log(key: String, motor: SparkMax) {
        log("$key/AppliedOutput", motor.appliedOutput)
        log("$key/BusVoltage", motor.busVoltage)
        log("$key/isFollower", motor.isFollower)
        log("$key/outputCurrent", motor.outputCurrent)
        log("$key/deviceID", motor.deviceId)
        log("$key/motorTemperature", motor.motorTemperature)
        log("$key/encoder/position", motor.encoder.position)
        log("$key/encoder/velocity", motor.encoder.velocity)
    }
}