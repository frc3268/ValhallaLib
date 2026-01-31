package frc.lib.swerve

import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.AnalogEncoder
import frc.lib.swerve.SwerveModuleIO.ModuleIOInputs
import frc.lib.rotation2dFromDeg
import kotlin.math.IEEErem


class SwerveModuleIOSparkMax(val moduleConstants: SwerveDriveConstants.ModuleConstants) : SwerveModuleIO {

    private val driveMotor = SparkMax(moduleConstants.driveMotorId, SparkLowLevel.MotorType.kBrushless)
    private val angleMotor = SparkMax(moduleConstants.angleMotorId, SparkLowLevel.MotorType.kBrushless)

    override val turnPIDController: PIDController = moduleConstants.pidController

    private val driveConfig: SparkMaxConfig = SparkMaxConfig()
    private val angleConfig: SparkMaxConfig = SparkMaxConfig()

    private val absoluteEncoder = AnalogEncoder(moduleConstants.encoderId)

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private val DRIVE_GEAR_RATIO: Double = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)
    private val TURN_GEAR_RATIO: Double = 150.0 / 7.0

    init {
                driveConfig.encoder.positionConversionFactor(SwerveDriveConstants.DriveMotor.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION)
                driveConfig.encoder.velocityConversionFactor(SwerveDriveConstants.AngleMotor.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION)

                driveConfig.inverted(moduleConstants.driveMotorReversed)
                angleConfig.inverted(moduleConstants.angleMotorReversed)

                driveConfig.openLoopRampRate(SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS)
                angleConfig.openLoopRampRate(SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS)


                driveConfig.signals.primaryEncoderPositionPeriodMs(15)
                driveConfig.signals.primaryEncoderPositionPeriodMs(15)
                driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

    }
    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.drivePositionMeters =
            -driveMotor.encoder.position
        inputs.driveVelocityMetersPerSec =
            -driveMotor.encoder.velocity

        inputs.turnAbsolutePosition =
            ((absoluteEncoder.get() * 360.0) + moduleConstants.angleOffset.degrees).rotation2dFromDeg()
        inputs.turnPosition =
            (( inputs.turnAbsolutePosition.degrees).IEEErem(360.0).rotation2dFromDeg())
        inputs.turnVelocityRadPerSec = (
                Units.rotationsPerMinuteToRadiansPerSecond(angleMotor.encoder.velocity)
                        / TURN_GEAR_RATIO)
    }

    override fun setDriveVoltage(volts: Double) {
        driveMotor.setVoltage(volts)
    }

    override fun setTurnVoltage(volts: Double) {
        angleMotor.setVoltage(volts)
    }

    // TODO: Test
    override fun setDriveBrakeMode(enable: Boolean) {
        driveConfig.idleMode(if (enable) IdleMode.kBrake else IdleMode.kCoast)
    }

    override fun setTurnBrakeMode(enable: Boolean) {
        angleConfig.idleMode(if (enable) IdleMode.kBrake else IdleMode.kCoast)
    }

    override fun reset() {
        driveMotor.encoder.position = 0.0
        angleMotor.encoder.position = ((absoluteEncoder.get() * 360.0) + moduleConstants.angleOffset.degrees)
    }
}