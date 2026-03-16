package frc.lib.gyro

import edu.wpi.first.math.geometry.Rotation2d

interface GyroIO {
    open class GyroIOInputs {
        var connected: Boolean = false
        var yawPosition: Rotation2d = Rotation2d()
        var odometryYawTimestamps: DoubleArray = doubleArrayOf()
        var odometryYawPositions: Array<Rotation2d> = arrayOf()
        var yawVelocityRadPerSec: Double = 0.0
    }
    fun updateInputs(inputs: GyroIOInputs)

    fun zeroYaw()
}

class GyroIOInputsAutoLogged : GyroIO.GyroIOInputs() {

}