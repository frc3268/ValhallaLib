package frc.robot.commands

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.rotation2dFromDeg
import frc.lib.swerve.SwerveDriveBase
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.thetaPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.xPIDController
import frc.lib.swerve.SwerveDriveConstants.DrivetrainConsts.yPIDController
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.function.Supplier
import kotlin.math.abs

class AlignToAprilTagCommand(val drive: SwerveDriveBase, val onRight: Supplier<Boolean>) : Command() {
    lateinit var bestTarget: PhotonTrackedTarget

    var fidID = -1
    var targetDelta = Pose2d()
    val field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
    var targetloc = Pose2d()

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        if (!drive.camera!!.frame.hasTargets()) {
            end(false)
        } else {
            bestTarget = drive.camera!!.frame.bestTarget
            fidID = bestTarget.fiducialId
            targetloc = field.getTagPose(fidID).get().toPose2d()
            //replace 0.5 with real target distance
            if (onRight.get()) {
                targetloc = Pose2d(
                    targetloc.x + targetloc.rotation.sin * -0.5,
                    targetloc.y - targetloc.rotation.cos * -0.5,
                    targetloc.rotation
                )
            }
        }
    }

    override fun execute() {
        if (fidID != -1) {
            targetDelta = Pose2d(
                targetloc.x - drive.getPose().x,
                targetloc.y - drive.getPose().y,
                (targetloc.rotation.degrees + 180 - drive.getPose().rotation.degrees).rotation2dFromDeg()
            )
            println(targetDelta.x)
            drive.setModuleStates(
                drive.constructModuleStatesFromChassisSpeeds(
                    -xPIDController.calculate(targetDelta.x, 0.0),
                    -yPIDController.calculate(targetDelta.y, 0.0),
                    -thetaPIDController.calculate(
                        targetDelta.rotation.degrees,
                        0.0
                    ) * MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND / 2,
                    true
                )

            )
        }
    }

    override fun isFinished(): Boolean {
        if (fidID == -1) {
            return true
        } else {
            //bestTarget.yaw < 5.0 &&
            return (abs(targetDelta.x) < 0.2 && abs(targetDelta.y) < 0.2 && abs(targetDelta.rotation.degrees) < 1)
        }
    }
}