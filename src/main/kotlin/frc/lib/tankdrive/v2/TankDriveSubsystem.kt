package frc.lib.tankdrive.v2

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.camera.Camera
import frc.lib.gyro.GyroIO
import frc.lib.gyro.GyroIOInputsAutoLogged
import frc.lib.gyro.GyroIOKauai
import frc.lib.tankdrive.legacy.TankDriveIOSparkMax
import frc.lib.tankdrive.v2.TankDriveConstants.TANK_DRIVE_TAB
import frc.robot.Constants
import frc.robot.Constants.CAMERA_NAME
import kotlin.math.abs
import kotlin.math.max

class TankDriveSubsystem(val io: TankDriveIOSparkMax, startingPose: Pose2d) : SubsystemBase() {

    private val shuffleboardTab = Shuffleboard.getTab(TANK_DRIVE_TAB)
    private var camera: Camera? = null
    private var poseEstimator: DifferentialDrivePoseEstimator

    private val gyroInputs = GyroIOInputsAutoLogged()
    private val gyro = when (Constants.mode) {
        Constants.States.REAL -> {
            GyroIOKauai()
        }

        Constants.States.REPLAY -> {
            object : GyroIO {
                override fun updateInputs(inputs: GyroIO.GyroIOInputs) {}
                override fun zeroYaw() {}
            }
        }

        Constants.States.SIM -> {
            object : GyroIO {
                override fun updateInputs(inputs: GyroIO.GyroIOInputs) {}
                override fun zeroYaw() {}
            }
        }
    }

    private var poseXEntry = shuffleboardTab.add("Pose X", 0.0).entry
    private var poseYEntry = shuffleboardTab.add("Pose Y", 0.0).entry
    private var headingEntry =
        shuffleboardTab.add("Robot Heading", gyroInputs.yawPosition.degrees).withWidget(BuiltInWidgets.kGyro).entry

    init {
        if (Constants.mode != Constants.States.REPLAY) {
            camera = Camera(CAMERA_NAME)
        }
        poseEstimator = DifferentialDrivePoseEstimator(
            TankDriveConstants.TankConstants.kinematics,
            getYaw(),
            0.0,
            0.0,
            startingPose,
        )
    }

    override fun periodic() {

    }

    fun driveForward(velocity: AngularVelocity): Command = run {
        io.setVelocityBoth(velocity)
    }

    fun turnLeft(velocity: AngularVelocity): Command = run {
        io.setVelocity(-velocity, velocity)
    }

    fun turnRight(velocity: AngularVelocity): Command = run {
        io.setVelocity(velocity, -velocity)
    }

    fun arcadeDrive(rotate: Double, drive: Double): Command = run {
        val maximum = Units.RPM.of(max(abs(drive), abs(rotate)))
        val total = Units.RPM.of(rotate + drive)
        val difference = Units.RPM.of(drive - rotate)
        if (drive >= 0) {
            if (rotate >= 0) {
                io.setVelocity(maximum, difference)
            } else {
                io.setVelocity(total, maximum)
            }
        } else {
            if (rotate >= 0) {
                io.setVelocity(total, -maximum)
            } else {
                io.setVelocity(-maximum, difference)
            }
        }
    }

    fun stop() {
        io.stop()
    }

    private fun getYaw(): Rotation2d = gyroInputs.yawPosition

    fun getPose(): Pose2d = Pose2d(
        poseEstimator.estimatedPosition.x,
        poseEstimator.estimatedPosition.y,
        poseEstimator.estimatedPosition.rotation
    )
}