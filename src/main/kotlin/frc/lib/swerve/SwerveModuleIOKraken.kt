package frc.lib.swerve

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.RelativeEncoder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.AnalogEncoder
import frc.lib.swerve.SwerveModuleIO.ModuleIOInputs
import frc.lib.rotation2dFromDeg
import kotlin.math.IEEErem


class SwerveModuleIOKraken(val moduleConstants: SwerveDriveConstants.ModuleConstants) : SwerveModuleIO {

    private val driveMotor = TalonFX(moduleConstants.DRIVE_MOTOR_ID, "rio")
    private val angleMotor = TalonFX(moduleConstants.ANGLE_MOTOR_ID,"rio")

    override val turnPIDController: PIDController = moduleConstants.PID_CONTROLLER


    private val absoluteEncoder = AnalogEncoder(moduleConstants.ENCODER_ID)

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private val DRIVE_GEAR_RATIO: Double = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)


    val dconfig = TalonFXConfiguration()
    val tconfig = TalonFXConfiguration()

    init {
                dconfig.Feedback.SensorToMechanismRatio=
                    SwerveDriveConstants.DriveMotor.POSITION_CONVERSION_FACTOR_METERS_PER_ROTATION
                tconfig.Feedback.SensorToMechanismRatio =
                    SwerveDriveConstants.AngleMotor.POSITION_CONVERSION_FACTOR_DEGREES_PER_ROTATION

                dconfig.MotorOutput.Inverted = if (moduleConstants.DRIVE_MOTOR_REVERSED) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
                tconfig.MotorOutput.Inverted =  if (moduleConstants.ANGLE_MOTOR_REVERSED) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive

                dconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
                tconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast
//                dconfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS
//                tconfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveDriveConstants.DrivetrainConsts.OPEN_LOOP_RAMP_RATE_SECONDS

                driveMotor.position.setUpdateFrequency(50.0, 15.0)
                //todo: fix? below
                angleMotor.position.setUpdateFrequency(50.0, 15.0)



                driveMotor.configurator.apply(dconfig)
                angleMotor.configurator.apply(tconfig)

        //apply config
        turnPIDController.enableContinuousInput(-180.0, 180.0)
    }


    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.drivePositionMeters =
            -driveMotor.position.valueAsDouble
        inputs.driveVelocityMetersPerSec =
            -driveMotor.velocity.valueAsDouble
        inputs.turnAbsolutePosition =
            ((absoluteEncoder.get()  * 360.0) + moduleConstants.ANGLE_OFFSET.degrees).rotation2dFromDeg()
        inputs.turnPosition =
            (((inputs.turnAbsolutePosition.degrees).IEEErem(360.0)).rotation2dFromDeg())
        inputs.turnVelocityRadPerSec = (
                Units.rotationsPerMinuteToRadiansPerSecond(angleMotor.velocity.valueAsDouble)
                        )

    }

    override fun setDriveVoltage(volts: Double) {
        driveMotor.setVoltage(volts)
    }

    override fun setTurnVoltage(volts: Double) {
        angleMotor.setVoltage(volts)
    }

    //todo: fix the two functions below this
    override fun setDriveBrakeMode(enable: Boolean) {
        dconfig.MotorOutput.NeutralMode = (if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
        driveMotor.configurator.apply(dconfig)
    }

    override fun setTurnBrakeMode(enable: Boolean) {
        tconfig.MotorOutput.NeutralMode = (if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
        angleMotor.configurator.apply(tconfig)
    }

    override fun reset() {
        driveMotor.setPosition(0.0)
        angleMotor.setPosition((absoluteEncoder.get() * 360.0) + moduleConstants.ANGLE_OFFSET.degrees)
    }
}