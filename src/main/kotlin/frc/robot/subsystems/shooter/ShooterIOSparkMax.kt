package frc.robot.subsystems.shooter

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import frc.lib.logging.SparkMaxLogging
import frc.lib.logging.TalonLogging

class ShooterIOSparkMax : IShooterIO {
    // Those who know...
    private val motorBarney: SparkMax = SparkMax(2, SparkLowLevel.MotorType.kBrushed)
    private val motorIsaac: SparkMax = SparkMax(1, SparkLowLevel.MotorType.kBrushed)

    private val bus: CANBus = CANBus.roboRIO()

    val motorAuxShooter: TalonFX = TalonFX(0, bus)

    var configBarney: SparkMaxConfig = SparkMaxConfig()
    var configIsaac: SparkMaxConfig = SparkMaxConfig()

    init {
        motorBarney.configure(
            configBarney,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        motorIsaac.configure(
            configIsaac,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
    }

    override fun setIntake(percentage: Double) {
        motorIsaac.set(-percentage)
        motorBarney.set(-percentage)
    }

    override fun setShooter(percentage: Double) {
        motorIsaac.set(-percentage)
        motorAuxShooter.set(percentage)
    }

    override fun setIntakeForShooter(percentage: Double) {
        motorBarney.set(percentage)
    }

    override fun log() {
        SparkMaxLogging.log("shooter/barney", motorBarney)
        SparkMaxLogging.log("shooter/isaac", motorIsaac)
        TalonLogging.log("shooter/aux", motorAuxShooter)
    }

    override fun stop() {
        motorBarney.stopMotor()
        motorIsaac.stopMotor()
        motorAuxShooter.stopMotor()
    }
}