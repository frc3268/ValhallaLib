package frc.lib.logging

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants

class ValhallaLogger : DogLog() {
    companion object {
        fun log(key: String, motor: TalonFX) {
            if (!Constants.AutoLog.ENABLE_PHOENIX_AUTOLOG) {
                log("$key/Status/isAlive", motor.isAlive)
                log("$key/Status/isConnected", motor.isConnected)
                log("$key/Status/isProLicensed", motor.isProLicensed.value)

                log("$key/Device/AncillaryDeviceTemp", motor.ancillaryDeviceTemp.value)
                log("$key/Device/DeviceTemp", motor.deviceTemp.value)
                log("$key/Device/MotorVoltage", motor.motorVoltage.value)
                log("$key/Device/SupplyVoltage", motor.supplyVoltage.value)
                log("$key/Device/SupplyCurrent", motor.supplyCurrent.value)
                
                log("$key/Position", motor.position.value)
                log("$key/Velocity", motor.velocity.value)
                log("$key/Acceleration", motor.acceleration.value)

                // Live Faults
                // TODO: Implement all of the faults into another logging class.
                log("$key/Fault/Live/Hardware", motor.fault_Hardware.value)
                log("$key/Fault/Live/UnderVoltage", motor.fault_Undervoltage.value)
                log("$key/Fault/Live/ProcTemp", motor.fault_ProcTemp.value)
                log("$key/Fault/Live/DeviceTemp", motor.fault_DeviceTemp.value)
                log("$key/Fault/Live/OverSupplyV", motor.fault_OverSupplyV.value)
                log("$key/Fault/Live/BootDuringEnable", motor.fault_BootDuringEnable.value)
                log("$key/Fault/Live/BridgeBrownout", motor.fault_BridgeBrownout.value)
                log("$key/Fault/Live/ForwardHardLimit", motor.fault_ForwardHardLimit.value)
                log("$key/Fault/Live/ForwardSoftLimit", motor.fault_ForwardSoftLimit.value)
                log("$key/Fault/Live/FusedSensorOutOfSync", motor.fault_FusedSensorOutOfSync.value)

            }
        }

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
                ValhallaLogger.log("$key/Fault/Live", motor.faults)
                ValhallaLogger.log("$key/Fault/Sticky", motor.stickyFaults)

                if (motor.hasActiveFault()) {
                    logFault("$key has an active fault", Alert.AlertType.kError);
                }
                if (motor.hasStickyFault()) {
                    logFault("$key has a sticky fault", Alert.AlertType.kError);
                }
            }
        }

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

        fun log(key: String, command: Command) {
            log("$key/Subsystem", command.subsystem)
            log("$key/IsFinished", command.isFinished)
            log("$key/IsScheduled", command.isScheduled)
        }
    }
}