package frc.lib.motor.sparkmax

import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import frc.lib.motor.MotorEncoder

class SparkMaxEncoder(
    var motorConfig: SparkMaxConfig = SparkMaxConfig()
) : MotorEncoder {

    override fun configurate() {
        TODO("Not yet implemented")
    }

    override fun resetPosition(position: Double) {
        TODO("Not yet implemented")
    }
}