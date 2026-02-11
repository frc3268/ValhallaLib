package frc.lib.tankdrive

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.camera.Camera
import frc.lib.gyro.GyroIO
import frc.lib.gyro.GyroIOInputsAutoLogged
import frc.lib.gyro.GyroIOKauai
import frc.lib.rotation2dFromRad
import frc.lib.swerve.SwerveDriveConstants
import frc.lib.tankdrive.v2.TankDriveConstants
import frc.robot.Constants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.max

class TankDriveSubsystem(val io: ITankDriveIO, startingPose: Pose2d) : SubsystemBase() {

    private val shuffleboardTab = Shuffleboard.getTab(TankDriveConstants.TANK_DRIVE_TAB)
    private var camera: Camera? = null
    private var poseEstimator: DifferentialDrivePoseEstimator

    var field: Field2d

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
            camera = Camera(Constants.CAMERA_NAME)
        }
        poseEstimator = DifferentialDrivePoseEstimator(
            TankDriveConstants.TankConstants.kinematics,
            getYaw(),
            io.getLeftDistance(),
            io.getRightDistance(),
            startingPose,
        )
        field = Field2d()
        shuffleboardTab.add(field).withWidget(BuiltInWidgets.kField)

    }

    override fun periodic() {

        io.periodic()

        if (Constants.mode == Constants.States.REAL) {
            gyro.updateInputs(gyroInputs)
        } else {
            //val deltas = modules.map { it.delta }.toTypedArray()
            val twist = TankDriveConstants.TankConstants.kinematics.toTwist2d(
                io.getLeftDistance(),
                io.getRightDistance(),
            )
            gyroInputs.yawPosition = (gyroInputs.yawPosition.plus(twist.dtheta.rotation2dFromRad()))
        }

        camera!!.captureFrame()
        poseEstimator.update(
            getYaw(),
            io.getLeftDistance(),
            io.getRightDistance()
        )

        field.robotPose = getPose()
        poseXEntry.setDouble(getPose().x)
        poseYEntry.setDouble(getPose().y)
        Logger.recordOutput("Robot/Pose", getPose())
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

    fun arcadeDrive(x: Double, y: Double, gain: Double): Command = run {
        val maximum = Units.RPM.of(max(abs(y), abs(x)))
        val total = Units.RPM.of(x + y)
        val difference = Units.RPM.of(y - x)
        if (y >= 0) {
            if (x >= 0) {
                io.setVelocity(maximum * gain, difference * gain)
            } else {
                io.setVelocity(total * gain, maximum * gain)
            }
        } else {
            if (x >= 0) {
                io.setVelocity(total * gain, -maximum * gain)
            } else {
                io.setVelocity(-maximum * gain, difference * gain)
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

    override fun simulationPeriodic() {
        io.simulationPeriodic()
    }
}