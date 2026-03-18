package frc.lib.logging

import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import frc.robot.Constants

class TalonLogging {
    companion object {
        fun log(key: String, motor: TalonFX) {
            if (!Constants.AutoLog.ENABLE_PHOENIX_AUTOLOG) {

                DogLog.log("$key/status/isAlive", motor.isAlive)
                DogLog.log("$key/status/isConnected", motor.isConnected)
                DogLog.log("$key/status/isProLicensed", motor.isProLicensed.value)

                DogLog.log("$key/device/ancillaryDeviceTemp", motor.ancillaryDeviceTemp.value)
                DogLog.log("$key/device/deviceTemp", motor.deviceTemp.value)
                DogLog.log("$key/device/motorVoltage", motor.motorVoltage.value)
                DogLog.log("$key/device/supplyVoltage", motor.supplyVoltage.value)
                DogLog.log("$key/device/supplyCurrent", motor.supplyCurrent.value)

                DogLog.log("$key/position", motor.position.value)
                DogLog.log("$key/velocity", motor.velocity.value)
                DogLog.log("$key/acceleration", motor.acceleration.value)

                // Live Faults
                // TODO: Implement all of the faults into another logging class.
                DogLog.log("$key/fault/live/hardware", motor.fault_Hardware.value)
                DogLog.log("$key/fault/live/underVoltage", motor.fault_Undervoltage.value)
                DogLog.log("$key/fault/live/procTemp", motor.fault_ProcTemp.value)
                DogLog.log("$key/fault/live/deviceTemp", motor.fault_DeviceTemp.value)
                DogLog.log("$key/fault/live/overSupplyV", motor.fault_OverSupplyV.value)
                DogLog.log("$key/fault/live/bootDuringEnable", motor.fault_BootDuringEnable.value)
                DogLog.log("$key/fault/live/bridgeBrownout", motor.fault_BridgeBrownout.value)
                DogLog.log("$key/fault/live/forwardHardLimit", motor.fault_ForwardHardLimit.value)
                DogLog.log("$key/fault/live/forwardSoftLimit", motor.fault_ForwardSoftLimit.value)
                DogLog.log("$key/fault/live/fusedSensorOutOfSync", motor.fault_FusedSensorOutOfSync.value)
            }
        }
    }
}