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
                log("$key/isFollower", motor.isFollower)
                log("$key/outputCurrent", motor.outputCurrent, "amps")
                log("$key/setSpeed", motor.get())
                log("$key/motorTemperature", motor.motorTemperature)

                log("$key/encoder/position", motor.encoder.position)
                log("$key/encoder/velocity", motor.encoder.velocity)

                // TODO: Use faults instead
                SparkFaultLogging.log("$key/fault/live", motor.faults)
                SparkFaultLogging.log("$key/fault/sticky", motor.stickyFaults)

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