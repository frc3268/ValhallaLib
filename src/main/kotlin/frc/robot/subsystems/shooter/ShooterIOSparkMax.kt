package frc.robot.subsystems.shooter

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.logging.ValhallaLogger
import frc.robot.Constants


class ShooterIOSparkMax : IShooterIO {
    // Those who know...
    private val motorBarney: SparkMax = SparkMax(2, SparkLowLevel.MotorType.kBrushed)
    private val motorIsaac: SparkMax = SparkMax(1, SparkLowLevel.MotorType.kBrushed)

    val motorAuxShooter: TalonFX = TalonFX(7, CANBus.roboRIO())

    var configBarney: SparkMaxConfig = SparkMaxConfig()
    var configIsaac: SparkMaxConfig = SparkMaxConfig()

    private val calibrationTab = Shuffleboard.getTab(Constants.CALIBRATION_TAB)

    private val auxShootSpeed =
        calibrationTab.addPersistent("Aux Talon Coefficient", 0.8).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(mapOf("min" to 0, "max" to 1)).entry

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
        motorAuxShooter.set(-percentage * auxShootSpeed.getDouble(0.0))
    }

    override fun setIntakeForShooter(percentage: Double) {
        motorBarney.set(percentage)
    }

    override fun log() {
        ValhallaLogger.log("Shooter/Barney", motorBarney)
        ValhallaLogger.log("Shooter/Isaac", motorIsaac)
        ValhallaLogger.log("Shooter/Aux", motorAuxShooter)
    }

    override fun stop() {
        motorBarney.stopMotor()
        motorIsaac.stopMotor()
        motorAuxShooter.stopMotor()
    }
}