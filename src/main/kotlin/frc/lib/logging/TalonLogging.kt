package frc.lib.logging

import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import frc.robot.Constants

class TalonLogging {
    companion object {
        fun log(key: String, motor: TalonFX) {
            if (!Constants.AutoLog.ENABLE_PHOENIX_AUTOLOG) {
                DogLog.log("$key/Status/isAlive", motor.isAlive)
                DogLog.log("$key/Status/isConnected", motor.isConnected)
                DogLog.log("$key/Status/isProLicensed", motor.isProLicensed.value)

                DogLog.log("$key/Device/AncillaryDeviceTemp", motor.ancillaryDeviceTemp.value)
                DogLog.log("$key/Device/DeviceTemp", motor.deviceTemp.value)
                DogLog.log("$key/Device/MotorVoltage", motor.motorVoltage.value)
                DogLog.log("$key/Device/SupplyVoltage", motor.supplyVoltage.value)
                DogLog.log("$key/Device/SupplyCurrent", motor.supplyCurrent.value)

                DogLog.log("$key/Position", motor.position.value)
                DogLog.log("$key/Velocity", motor.velocity.value)
                DogLog.log("$key/Acceleration", motor.acceleration.value)

                // Live Faults
                // TODO: Implement all of the faults into another logging class.
                DogLog.log("$key/Fault/Live/Hardware", motor.fault_Hardware.value)
                DogLog.log("$key/Fault/Live/UnderVoltage", motor.fault_Undervoltage.value)
                DogLog.log("$key/Fault/Live/ProcTemp", motor.fault_ProcTemp.value)
                DogLog.log("$key/Fault/Live/DeviceTemp", motor.fault_DeviceTemp.value)
                DogLog.log("$key/Fault/Live/OverSupplyV", motor.fault_OverSupplyV.value)
                DogLog.log("$key/Fault/Live/BootDuringEnable", motor.fault_BootDuringEnable.value)
                DogLog.log("$key/Fault/Live/BridgeBrownout", motor.fault_BridgeBrownout.value)
                DogLog.log("$key/Fault/Live/ForwardHardLimit", motor.fault_ForwardHardLimit.value)
                DogLog.log("$key/Fault/Live/ForwardSoftLimit", motor.fault_ForwardSoftLimit.value)
                DogLog.log("$key/Fault/Live/FusedSensorOutOfSync", motor.fault_FusedSensorOutOfSync.value)
            }
        }
    }
}