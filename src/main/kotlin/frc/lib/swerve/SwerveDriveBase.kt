package frc.lib.swerve

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.camera.Camera
import frc.lib.gyro.GyroIO
import frc.lib.gyro.GyroIOInputsAutoLogged
import frc.lib.gyro.GyroIOKauai
import frc.lib.rotation2dFromDeg
import frc.lib.rotation2dFromRad
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_SPEED_METERS_PER_SECOND
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.kinematics
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.thetaPIDController
import frc.robot.Constants
import frc.robot.Constants.CAMERA_NAME
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import java.util.*

class SwerveDriveBase(startingPose: Pose2d) : SubsystemBase() {
    private val shuffleboardTab = Shuffleboard.getTab("Drivetrain")
    private val gyroInputs = GyroIOInputsAutoLogged()
    private var poseEstimator: SwerveDrivePoseEstimator
    private val modules: List<SwerveModule> =
        when (Constants.mode){
            Constants.States.REAL -> {
                SwerveDriveConstants.modules.mapIndexed { _, swerveMod -> SwerveModule(SwerveModuleIOKraken(swerveMod),swerveMod.moduleNumber) }
            }
            Constants.States.REPLAY -> {
                SwerveDriveConstants.modules.mapIndexed { _, swerveMod -> SwerveModule(object: SwerveModuleIO {
                    override val turnPIDController: PIDController = swerveMod.pidController
                    override fun updateInputs(inputs: SwerveModuleIO.ModuleIOInputs) {
                        //no
                    }

                    override fun setDriveVoltage(volts: Double) {
                        //no
                    }

                    override fun setTurnVoltage(volts: Double) {
                        //no
                    }

                    override fun setDriveBrakeMode(enable: Boolean) {
                        //no
                    }

                    override fun setTurnBrakeMode(enable: Boolean) {
                        //no
                    }

                    override fun reset() {
                        //no
                    }
                } ,swerveMod.moduleNumber) }
            }
            Constants.States.SIM -> {
                SwerveDriveConstants.modules.mapIndexed { _, swerveMod -> SwerveModule(SwerveModuleIOSim(swerveMod.moduleNumber),swerveMod.moduleNumber) }
            }
        }
    private val gyro = when (Constants.mode){
        Constants.States.REAL -> {
            GyroIOKauai()
        }
        Constants.States.REPLAY -> {
            object : GyroIO {
                override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
                    //no
                }

                override fun zeroYaw() {
                    //no
                }
            }
        }
        Constants.States.SIM -> {
            object : GyroIO {
                override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
                    //no
                }

                override fun zeroYaw() {
                    //no
                }
            }
        }
    }

    private var poseXEntry = shuffleboardTab.add("Pose X", 0.0).entry
    private var poseYEntry = shuffleboardTab.add("Pose Y", 0.0).entry
    private var headingEntry = shuffleboardTab.add("Robot Heading", gyroInputs.yawPosition.degrees).withWidget(BuiltInWidgets.kGyro).entry
    private var seesAprilTag = shuffleboardTab.add("Sees April Tag?", false).withWidget(BuiltInWidgets.kBooleanBox).entry
    var field:Field2d
    var field2:Field2d

    var camera: Camera? = null

    init {
        thetaPIDController.enableContinuousInput(
                180.0, -180.0
        )

        // This might still work in REPLAY mode
        if(Constants.mode != Constants.States.REPLAY){
            camera = Camera(CAMERA_NAME)
        }

        zeroYaw()
        // https://github.com/Team364/BaseFalconSwerve/issues/8#issuecomment-1384799539
        // wtf? weird issue
        Timer.delay(1.0)
        resetModulesToAbsolute()
        shuffleboardTab.add("Zero Heading", zeroHeadingCommand()).withWidget(BuiltInWidgets.kCommand)
        poseEstimator = SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), startingPose, VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.5, 0.5, 0.5))
        field= Field2d()
        field2 = Field2d()
        shuffleboardTab.add(field).withWidget(BuiltInWidgets.kField)
        shuffleboardTab.add("Field", field2).withWidget(BuiltInWidgets.kField)
        field.getObject("obr").setPoses(Pose2d(
            13.0856, 4.0259, 0.0.rotation2dFromDeg()
        ), Pose2d(
            4.4895, 4.0259, 0.0.rotation2dFromDeg()
        ))
    }

    override fun periodic() {
        if(Constants.mode == Constants.States.REAL) {
            gyro.updateInputs(gyroInputs)
        } else{
            val deltas = modules.map { it.delta }.toTypedArray()
            val twist = kinematics.toTwist2d(*deltas)
            gyroInputs.yawPosition = (gyroInputs.yawPosition.plus(twist.dtheta.rotation2dFromRad()))
        }
        camera!!.captureFrame()
        //estimate robot pose based on what the encoders say
        poseEstimator.update(getYaw(), getModulePositions())
        //estimate robot pose based on what the camera sees
        if(gyroInputs.yawVelocityRadPerSec < Math.PI) {
            seesAprilTag.setBoolean(camera!!.frame.hasTargets())
            if(camera!!.frame.hasTargets() && DriverStation.isDisabled()){
                val visionEst: Optional<EstimatedRobotPose>? = camera!!.getEstimatedPose()
                visionEst?.ifPresent { est ->
                    field2.robotPose = Pose2d(est.estimatedPose.toPose2d().x, est.estimatedPose.toPose2d().y, est.estimatedPose.toPose2d().rotation)
                        poseEstimator.addVisionMeasurement(
                        Pose2d(est.estimatedPose.toPose2d().x, est.estimatedPose.toPose2d().y, est.estimatedPose.toPose2d().rotation),
                        est.timestampSeconds,
                        camera!!.getEstimationStdDevs(est.estimatedPose.toPose2d())
                        )
                }
            }
        }
        //update modules
        for (mod in modules) {
            mod.update()
        }
        //update drivetrain tab on shuffleboard
        field.robotPose = getPose()
        poseXEntry.setDouble(getPose().x)
        poseYEntry.setDouble(getPose().y)
        Logger.recordOutput("Robot/Pose", getPose())


    }
    // Go to http://localhost:1181/ to see preprocessed stream, http://localhost:1182/ to see processed stream.
    override fun simulationPeriodic() {
        camera?.simPeriodic(poseEstimator)
    }

    private fun zeroYaw() {
        gyro.zeroYaw()
    }

    private fun resetModulesToAbsolute() {
        for (mod in modules) {
            mod.resetToAbsolute()
        }
    }

    fun setModuleStates(desiredStates: Array<SwerveModuleState>) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
            MAX_SPEED_METERS_PER_SECOND
        )

        for (mod in modules) {
            mod.setDesiredState(desiredStates[mod.index - 1])
        }
    }

    fun constructModuleStatesFromChassisSpeeds(xSpeedMetersPerSecond: Double, ySpeedMetersPerSecond: Double, turningSpeedDegreesPerSecond: Double, fieldOriented: Boolean): Array<SwerveModuleState> {
        return kinematics.toSwerveModuleStates(
                    if (fieldOriented){
                        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, turningSpeedDegreesPerSecond.rotation2dFromDeg().radians, (poseEstimator.estimatedPosition.rotation))
                    }else{
                        ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, turningSpeedDegreesPerSecond.rotation2dFromDeg().radians)}
            )
    }

    fun constructTeleopModuleStatesFromChassisSpeeds(xSpeedMetersPerSecond: Double, ySpeedMetersPerSecond: Double, turningSpeedDegreesPerSecond: Double, fieldOriented: Boolean): Array<SwerveModuleState> {
        val red = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        return kinematics.toSwerveModuleStates(
            if (fieldOriented){
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, turningSpeedDegreesPerSecond.rotation2dFromDeg().radians, if (red) {(poseEstimator.estimatedPosition.rotation + 180.0.rotation2dFromDeg())} else {(poseEstimator.estimatedPosition.rotation)})
            }else{
                ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, turningSpeedDegreesPerSecond.rotation2dFromDeg().radians)}
        )
    }

    fun stop() {
        for (mod in modules) {
            mod.stop()
        }
    }

    //reset yaw on gyro so that wherever the gyro is pointing is the new forward(0) value
    fun zeroHeadingCommand(): Command {
        return runOnce {
            //fakeGyroOffset = getYaw().degrees
            zeroYaw()
        }
    }

    //getters
    private fun getYaw(): Rotation2d = gyroInputs.yawPosition
    fun getPose(): Pose2d = Pose2d(poseEstimator.estimatedPosition.x, poseEstimator.estimatedPosition.y, poseEstimator.estimatedPosition.rotation)
    fun getModuleStates(): Array<SwerveModuleState> = modules.map { it.getState() }.toTypedArray()
    private fun getModulePositions(): Array<SwerveModulePosition> = modules.map { it.getPosition() }.toTypedArray()
}
