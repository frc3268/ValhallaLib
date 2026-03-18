package frc.lib.logging

import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog

class TalonLogging {
    companion object {
        fun log(key: String, motor: TalonFX) {
            DogLog.log("$key/status/isAlive", motor.isAlive)
            DogLog.log("$key/status/isConnected", motor.isConnected)
            DogLog.log("$key/acceleration", motor.acceleration.value)
            DogLog.log("$key/velocity", motor.velocity.value)
            DogLog.log("$key/position", motor.position.value)
            DogLog.log("$key/motorVoltage", motor.motorVoltage.value)
            DogLog.log("$key/supplyVoltage", motor.supplyVoltage.value)
            DogLog.log("$key/supplyCurrent", motor.supplyCurrent.value)
        }
    }
}