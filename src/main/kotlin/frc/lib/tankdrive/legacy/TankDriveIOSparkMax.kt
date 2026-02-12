package frc.lib.tankdrive.legacy

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.tankdrive.ITankDriveIO
import frc.lib.tankdrive.v2.TankDriveConstants.TANK_DRIVE_TAB


class TankDriveIOSparkMax : ITankDriveIO {
    private val shuffleboardTab = Shuffleboard.getTab(TANK_DRIVE_TAB)

    private var lastLeftVelocity = shuffleboardTab.add("Left Velocity", 0.0).entry
    private var lastRightVelocity = shuffleboardTab.add("Right Velocity", 0.0).entry

    // TODO: Get the device IDs
    private val left1: SparkMax = SparkMax(3, SparkLowLevel.MotorType.kBrushed)
    private val left2: SparkMax = SparkMax(4, SparkLowLevel.MotorType.kBrushed)
    private val right1: SparkMax = SparkMax(5, SparkLowLevel.MotorType.kBrushed)
    private val right2: SparkMax = SparkMax(6, SparkLowLevel.MotorType.kBrushed)

    var configLeft1: SparkMaxConfig = SparkMaxConfig()
    var configLeft2: SparkMaxConfig = SparkMaxConfig()
    var configRight1: SparkMaxConfig = SparkMaxConfig()
    var configRight2: SparkMaxConfig = SparkMaxConfig()

    init {
        configLeft2.follow(left1)
        configRight2.follow(right1)
        configRight1.inverted(true);

        // left1.encoder.position

        // Pid, Needs tuning
        configLeft1.closedLoop.p(1.0).i(0.0).d(0.0);
        configRight1.closedLoop.p(1.0).i(0.0).d(0.0);


        left1.configure(configLeft1, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
        left2.configure(configLeft2, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        right1.configure(
            configRight1,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        right2.configure(
            configRight2,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

    }

    override fun setLeftVelocity(velocity: AngularVelocity) {
        left1.closedLoopController.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
        lastLeftVelocity.setDouble(velocity.`in`(Units.RPM));
    }

    override fun setRightVelocity(velocity: AngularVelocity) {
        right1.closedLoopController.setReference(velocity.`in`(Units.RPM), SparkBase.ControlType.kVelocity)
        lastRightVelocity.setDouble(velocity.`in`(Units.RPM));
    }

    override fun setVelocity(leftVel: AngularVelocity, rightVel: AngularVelocity) {
        setLeftVelocity(leftVel)
        setRightVelocity(rightVel)
    }

    override fun setVelocityBoth(velocity: AngularVelocity) {
        setVelocity(velocity, velocity)
    }

    override fun getLeftDistance(): Double = left1.encoder.position
    override fun getRightDistance(): Double = right1.encoder.position

    override fun stop() {
        left1.stopMotor()
        right1.stopMotor()
        lastLeftVelocity.setDouble(0.0);
        lastLeftVelocity.setDouble(0.0);
    }

}