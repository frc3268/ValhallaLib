package frc.lib.logging

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.wpilibj.Alert
import frc.robot.Constants


class SparkMaxLogging : DogLog() {
    companion object {
        fun log(key: String, motor: SparkMax) {
            if (!Constants.AutoLog.ENABLE_REV_AUTOLOG) {
                log("$key/AppliedOutput", motor.appliedOutput)
                log("$key/BusVoltage", motor.busVoltage)
                log("$key/IsFollower", motor.isFollower)
                log("$key/OutputCurrent", motor.outputCurrent, "amps")
                log("$key/SetSpeed", motor.get())
                log("$key/MotorTemperature", motor.motorTemperature)

                log("$key/Encoder/Position", motor.encoder.position)
                log("$key/Encoder/Velocity", motor.encoder.velocity)

                // TODO: Use faults instead
                SparkFaultLogging.log("$key/Fault/Live", motor.faults)
                SparkFaultLogging.log("$key/Fault/Sticky", motor.stickyFaults)

                if (motor.hasActiveFault()) {
                    logFault("$key has an active fault", Alert.AlertType.kError);
                }
                if (motor.hasStickyFault()) {
                    logFault("$key has a sticky fault", Alert.AlertType.kError);
                }
            }
        }
    }
}