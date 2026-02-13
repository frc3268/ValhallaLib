package frc.robot.subsystems.shooter

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity

class ShooterSparkMax {
    // Those who know...
    private val BarneyMotor: SparkMax = SparkMax(1, SparkLowLevel.MotorType.kBrushed)
    private val IsaacMotor: SparkMax = SparkMax(2, SparkLowLevel.MotorType.kBrushed)

    var configBarney: SparkMaxConfig = SparkMaxConfig()
    var configIsaac: SparkMaxConfig = SparkMaxConfig()
    init {
        // Pid, Needs tuning
        configBarney.closedLoop.p(1.0).i(0.0).d(0.0);
        configIsaac.closedLoop.p(1.0).i(0.0).d(0.0);

        BarneyMotor.configure(
            configBarney,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        IsaacMotor.configure(
            configIsaac,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }

    fun setIntakeVelocity(velocity: AngularVelocity) {
        IsaacMotor.closedLoopController.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
    }

    fun setShooterVelocity(velocity: AngularVelocity) {
        BarneyMotor.closedLoopController.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
        IsaacMotor.closedLoopController.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
    }
}