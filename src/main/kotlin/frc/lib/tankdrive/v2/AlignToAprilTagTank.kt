package frc.lib.tankdrive.v2

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.camera.Camera
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

class AlignToAprilTagTank(
    val drive: TankDriveSubsystem,
    val camera: Camera,
    val getPose: () -> Pose2d,
    val onRight: () -> Boolean
) : Command() {

    private val field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    // TODO: Tune the PID
    private val turnPID = PIDController(0.0, 0.0, 0.0)
    private val drivePID = PIDController(0.0, 0.0, 0.0)

    private var fidID = -1
    private var apriltagLoc = Pose2d()

    /* There are three phases the robot can be in when trying to align to april tag
    TURN_TO_TARGET, DRIVE_TO_TARGET, DONE
     */
    private var phase = Phase.TURN_TO_TARGET

    private enum class Phase { TURN_TO_TARGET, DRIVE_TO_TARGET, DONE }

    init {
        addRequirements(drive)
        turnPID.setTolerance(2.0)
        drivePID.setTolerance(0.1)
    }

    override fun initialize() {
        phase = Phase.TURN_TO_TARGET
        fidID = -1

        if (!camera.frame.hasTargets()) {
            phase = Phase.DONE
            return
        }

        val bestTarget = camera.frame.bestTarget
        fidID = bestTarget.fiducialId
        apriltagLoc = field.getTagPose(fidID).get().toPose2d()

        if (onRight()) {
            apriltagLoc = Pose2d(
                apriltagLoc.x + apriltagLoc.rotation.sin * -0.5,
                apriltagLoc.y - apriltagLoc.rotation.cos * -0.5,
                apriltagLoc.rotation
            )
        }
    }

    override fun execute() {
        if (fidID == -1) return

        val currentPose = getPose()
        val dx = apriltagLoc.x - currentPose.x
        val dy = apriltagLoc.y - currentPose.y
        val distance = hypot(dx, dy)
        val angleToTarget = Math.toDegrees(atan2(dy, dx))
        val currentAngle = currentPose.rotation.degrees
        val finalAngle = apriltagLoc.rotation.degrees + 180

        when (phase) {
            /* Calculates angle between the bot and apriltag and rotates facing it */
            Phase.TURN_TO_TARGET -> {
                val turnError = normalizeAngle(angleToTarget - currentAngle)
                val turnOutput = Units.RPM.of(turnPID.calculate(turnError, 0.0))
                // turns the robot
                drive.io.setVelocity(-turnOutput, turnOutput)

                // if its within 5 degrees of the april tag, move to next phase (drive towards it)
                if (abs(turnError) < 5.0) {
                    phase = Phase.DRIVE_TO_TARGET
                }
            }
            /* Drive tankdrive towards apriltag */
            Phase.DRIVE_TO_TARGET -> {
                val driveOutput = Units.RPM.of(drivePID.calculate(distance, 0.0))
                drive.io.setVelocity(-driveOutput, -driveOutput)

                if (distance < 0.2) {
                    phase = Phase.DONE
                }
            }

            Phase.DONE -> drive.stop()
        }
    }

    override fun isFinished(): Boolean = phase == Phase.DONE

    override fun end(interrupted: Boolean) {
        drive.stop()
    }

    /* converts angles to between -180 and 180 */
    private fun normalizeAngle(angle: Double): Double {
        var a = angle % 360
        if (a > 180) a -= 360
        if (a < -180) a += 360
        return a
    }
}
