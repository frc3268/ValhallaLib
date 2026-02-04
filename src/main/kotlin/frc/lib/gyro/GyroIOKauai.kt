package frc.lib.gyro

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import frc.lib.rotation2dFromDeg
import kotlin.math.IEEErem


class GyroIOKauai : GyroIO {
    private val gyro = AHRS(SPI.Port.kMXP)


    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = gyro.isConnected
        inputs.yawPosition = (gyro.rotation2d.degrees).IEEErem(360.0).rotation2dFromDeg()
        inputs.yawVelocityRadPerSec = gyro.rate.rotation2dFromDeg().radians
    }

    override fun zeroYaw() {
        gyro.zeroYaw()
    }
}