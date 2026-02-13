package frc.lib.camera

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.Constants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.targeting.PhotonPipelineResult
import java.io.IOException
import java.util.*

class Camera(name: String) {
    private val limelight = PhotonCamera(name)
    private val shuffleboardTab = Shuffleboard.getTab("Camera Feed")
    var frame = PhotonPipelineResult()
    private var poseEstimator: PhotonPoseEstimator? = null
    private var seesAprilTag =
        shuffleboardTab.add("Sees April Tag?", false).withWidget(BuiltInWidgets.kBooleanBox).entry

    private var visionSim: VisionSystemSim? = null;

    private var robotToCam = Transform3d(
        // TODO: Check sign of these measurements
        Translation3d(
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(12.4),
            Units.inchesToMeters(-14.74),
        ),
        Rotation3d(
            0.0,
            0.0,
            0.0
        )
    )

    //private var cameraProp: SimCameraProperties? = null
    //private var visionSim: VisionSystemSim?: = null

    init {
        try {

            poseEstimator =
                PhotonPoseEstimator(
                    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCam
                )


        } catch (e: IOException) {
            DriverStation.reportError("AprilTag: Failed to Load", e.stackTrace)
            // !add some way to lock down apriltag features after this
        }

        if (Constants.mode == Constants.States.SIM) {
            val cameraProp = SimCameraProperties();
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = VisionSystemSim(name);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            // TODO: Configure the camera!
            cameraProp.setCalibration(
                Constants.CameraSimulationCalibration.RES_WIDTH,
                Constants.CameraSimulationCalibration.RES_HEIGHT,
                Rotation2d.fromDegrees(Constants.CameraSimulationCalibration.FOV_DIAGONAL)
            );
            cameraProp.setCalibError(
                Constants.CameraSimulationCalibration.AVG_ERROR_PX,
                Constants.CameraSimulationCalibration.ERROR_STD_DEV_PX
            );

            cameraProp.setFPS(Constants.CameraSimulationConstants.CAMERA_FPS);
            cameraProp.setAvgLatencyMs(Constants.CameraSimulationConstants.AVERAGE_LATENCY);
            cameraProp.setLatencyStdDevMs(Constants.CameraSimulationConstants.LATENCY_STD_DEV_MS);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            var cameraSim = PhotonCameraSim(limelight, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.

            // Or we could use robotToCam
            visionSim!!.addCamera(cameraSim, poseEstimator!!.robotToCameraTransform);

            // Enable the raw and processed streams. These are enabled by default.
            cameraSim.enableRawStream(Constants.CameraSimulationConstants.ENABLE_RAW_STREAM);
            cameraSim.enableProcessedStream(Constants.CameraSimulationConstants.ENABLE_PROCESSED_STREAM);

            // Enable wireframe mode.
            cameraSim.enableDrawWireframe(Constants.CameraSimulationConstants.USE_WIREFRAME);

            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            val tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile)
            visionSim!!.addAprilTags(tagLayout)

        }
    }

    //call periodically
    //does this work?? consult documentation
    fun captureFrame() {
        val results = limelight.allUnreadResults
        if (results.isNotEmpty()) {
            frame = results.last()
            seesAprilTag.setBoolean(frame.hasTargets())
        }
    }


    // called periodically in a simulation
    fun simPeriodic(driveTrain: SwerveDrivePoseEstimator) {
        visionSim?.update(driveTrain.estimatedPosition);
        // val debugField = visionSim?.debugField
    }


    fun resetPose(pose: Pose2d) {
        poseEstimator ?: return
        poseEstimator?.setReferencePose(pose)
    }

    fun getEstimatedPose(): Optional<EstimatedRobotPose>? =
        poseEstimator?.update(frame)

    fun getEstimationStdDevs(estimatedPose: Pose2d): Matrix<N3, N1>? {
        //todo: expiriment with vecbuilder values(somehow)
        var estStdDevs = VecBuilder.fill(.7, .7, .9)
        val targets = frame.getTargets()
        var numTags = 0
        var avgDist = 0.0

        for (tgt in targets) {
            val tagPose = poseEstimator?.fieldTags?.getTagPose(tgt.fiducialId) ?: continue
            if (tagPose.isEmpty) continue
            numTags++
            avgDist +=
                tagPose.get().toPose2d().translation.getDistance(estimatedPose.translation)
        }
        if (numTags == 0) return estStdDevs
        avgDist /= numTags
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 2.0)
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30))

        return estStdDevs
    }
}