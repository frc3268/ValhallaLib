package frc.lib.gyro

import com.studica.frc.AHRS
import frc.lib.rotation2dFromDeg
import kotlin.math.IEEErem


class GyroIOKauai : GyroIO {
    private val gyro = AHRS(AHRS.NavXComType.kMXP_SPI)


    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = gyro.isConnected
        inputs.yawPosition = (gyro.rotation2d.degrees).IEEErem(360.0).rotation2dFromDeg()
        inputs.yawVelocityRadPerSec = gyro.rate.rotation2dFromDeg().radians
    }

    override fun zeroYaw() {
        gyro.zeroYaw()
    }
}