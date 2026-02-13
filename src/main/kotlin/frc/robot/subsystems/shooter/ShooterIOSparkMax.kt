package frc.robot.subsystems.shooter

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import jdk.jfr.Percentage

class ShooterIOSparkMax : IShooterIO {
    // Those who know...
    private val motorBarney: SparkMax = SparkMax(2, SparkLowLevel.MotorType.kBrushed)
    private val motorIsaac: SparkMax = SparkMax(1, SparkLowLevel.MotorType.kBrushed)

    var configBarney: SparkMaxConfig = SparkMaxConfig()
    var configIsaac: SparkMaxConfig = SparkMaxConfig()
    init {
        // Pid, Needs tuning
        configBarney.closedLoop.p(1.0).i(0.0).d(0.0);
        configIsaac.closedLoop.p(1.0).i(0.0).d(0.0);

        motorBarney.configure(
            configBarney,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        motorIsaac.configure(
            configIsaac,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }

    override fun setIntakePercent(percentage: Double) {
        motorIsaac.set(percentage)
    }

    override fun setShooterVelocity(velocity: AngularVelocity) {
        motorBarney.closedLoopController.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
        motorIsaac.closedLoopController.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
    }

    override fun stop() {
        motorBarney.stopMotor()
        motorIsaac.stopMotor()
    }
}