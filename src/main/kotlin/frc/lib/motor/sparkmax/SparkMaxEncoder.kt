package frc.lib.motor.sparkmax

import com.revrobotics.spark.config.SparkMaxConfig
import frc.lib.motor.IMotorEncoder

class SparkMaxEncoder(
    var motorConfig: SparkMaxConfig = SparkMaxConfig()
) : IMotorEncoder {
    override fun configurate() {
        TODO("Not yet implemented")
    }

    override fun resetPosition(position: Double) {
        TODO("Not yet implemented")
    }
}