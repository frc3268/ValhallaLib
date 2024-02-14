package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.lib.basics.SwerveDriveBase
import frc.lib.utils.TrajectoryOrchestrator
import frc.lib.utils.rotation2dFromDeg
import frc.robot.subsystems.ExampleSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin

class Autos private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        /** Example static factory for an autonomous command.  */
        fun exampleAuto(subsystem: ExampleSubsystem): Command {
            return Commands.sequence(subsystem.exampleMethodCommand(), ExampleCommand(subsystem))
        }

        fun taxiAuto(drive: SwerveDriveBase): Command{
            val color = DriverStation.getAlliance()
            var to = Pose2d(2.1336, 0.0, Rotation2d.fromDegrees(0.0)).relativeTo(drive.getPose())
            color?.ifPresent{
                color ->
               if(color == DriverStation.Alliance.Red){
                    to = Pose2d(-2.1336, 0.0, Rotation2d.fromDegrees(0.0)).relativeTo(drive.getPose())
                }

            }
            return TrajectoryOrchestrator.beelineCommand(
                drive,
                to
            )

        }

        fun goToSpeaker(drive:SwerveDriveBase):Command{
            val color = DriverStation.getAlliance()
            //todo: fix x
            var to = Pose2d(1.0, 5.547868, 0.0.rotation2dFromDeg())
            color?.ifPresent { color ->
                if (color == DriverStation.Alliance.Red) {
                    to = Pose2d(15.579342, 5.547868, 180.0.rotation2dFromDeg())
                }
            }
            return TrajectoryOrchestrator.beelineCommand(
                drive,
                to
            )

        }

        fun goToAmpCommand(drive:SwerveDriveBase):Command{
            val color = DriverStation.getAlliance()
            //todo: fix x
            var to = Pose2d(1.84404, 8.2042, 270.0.rotation2dFromDeg())
            color?.ifPresent { color ->
                if (color == DriverStation.Alliance.Red) {
                    to = Pose2d(14.929358, 8.2042, 270.0.rotation2dFromDeg())
                }
            }
            return TrajectoryOrchestrator.beelineCommand(
                drive,
                to
            )

        }

        fun goToSourceCommand(drive:SwerveDriveBase, closerToBaseLine: Boolean):Command{
            val color = DriverStation.getAlliance()
            var to = Pose2d(0.356108, 0.883666, 60.0.rotation2dFromDeg())
            if(!closerToBaseLine){
                to = Pose2d(1.461516, 0.245872, 60.0.rotation2dFromDeg())
            }
            color?.ifPresent{ color ->
                if (color == DriverStation.Alliance.Blue){
                    to = Pose2d(15.079472, 0.245872, 120.0.rotation2dFromDeg())
                    if(!closerToBaseLine){
                        to = Pose2d(16.185134, 0.883666, 120.0.rotation2dFromDeg())
                    }
                }

            }
            return TrajectoryOrchestrator.beelineCommand(
                drive, to
            )
        }

        fun goToSourceAndIntakeCommand(drive:SwerveDriveBase, closerToBaseLine: Boolean, shooter: ShooterSubsystem): Command{
            return SequentialCommandGroup(
                goToSourceCommand(drive, closerToBaseLine),
                sourceIntakeCommand(shooter)
            )
        }

        fun goWithinSpeakerCommand(drive:SwerveDriveBase): Command {
            val color = DriverStation.getAlliance()
            //todo: fix x
            var to =
                        Pose2d(0.0, 5.547868, 0.0.rotation2dFromDeg())
            color.ifPresent {
                color ->
                if (color == DriverStation.Alliance.Red) {
                    to = Pose2d(14.579342, 5.547868, 180.0.rotation2dFromDeg())
                }
            }
            //todo: make this real
            val pose = drive.getPose()
            val c = 1.0
            val x = pose.x - to.x
            val y = pose.y - to.y
            val theta = atan(y/x).rotation2dFromDeg()
            return TrajectoryOrchestrator.beelineCommand(
                drive, Pose2d(to.x + cos(theta.radians)*c, to.y+ sin(theta.radians)*c, theta + pose.rotation))
        }


        fun driveUpAndShootSpeakerCommand(drive:SwerveDriveBase, intake:IntakeSubsystem, shooter:ShooterSubsystem) : Command{
            return SequentialCommandGroup(
                goToSpeaker(drive),
                intake.takeOutCommand(),
                shooter.shootCommand()
            )
        }

        fun groundIntakeCommand(intake:IntakeSubsystem) : Command {
            return SequentialCommandGroup(
                intake.poweredArmDownCommand(),
                intake.takeInCommand(),
                intake.poweredArmUpCommand()
            )
        }

        fun shootSpeakerCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem): Command{
            return SequentialCommandGroup(
                intake.takeOutCommand(),
                shooter.shootCommand()
            )
        }

        fun shootAmpCommand(intake: IntakeSubsystem, shooter: ShooterSubsystem): Command {
        return SequentialCommandGroup(
            intake.takeOutCommand(),
            shooter.ampCommand()
        )
        }

        fun sourceIntakeCommand(shooter: ShooterSubsystem): Command{
            return shooter.takeInCommand()
        }

        fun driveUpAndIntakeSourceCommand(drive: SwerveDriveBase,shooter: ShooterSubsystem, closerToBaseLine: Boolean) : Command {
            return SequentialCommandGroup(
                    goToSourceCommand(drive, closerToBaseLine),//fix closer to baseline
                    shooter.takeInCommand()
            )
        }

        //Find Intersetion is not complete. FindIntersection and calculateIntersection need testing.
// I'm sure there's a more efficent way to do this too
        fun findIntersectionWithStages(startPose:Pose2d, endPose:Pose2d): Command{
            val clear: Boolean = true;
            val startX: Double = startPose.getX()
            val startY: Double = startPose.getY()
            val endX: Double = endPose.getX()
            val endY: Double = endPose.getY()
            clear = calculateIntersection(3.0734, 4.105656, 5.770626, 6.66293, startX, startY, endX, endY) //blue stage ab
            clear = calculateIntersection(5.770626, 6.66293, 5.77026, 2.548382 startX, startY, endX, endY) //blue stage ac
            clear = calculateIntersection(3.0734, 4.105656, 5.770626, 2.548382, startX, startY, endX, endY) //blue stage bc
            clear = calculateIntersection(10.617454, 4.105656, 13.31468, 4.105656, startX, startY, endX, endY) //red stage ba
            clear = calculateIntersection(10.617454, 2.548382, 13.31468, 4.105656, startX, startY, endX, endY) //red stage ca
            clear = calculateIntersection(10.617454, 4.105656, 10.617454, 2.548382, startX, startY, endX, endY) //red stage bc
            if (clear == false) {
                //find another path
            }
        }

        fun calculateIntersection(hazardStartX:double, hazardStartY:double, hazardEndX:double, hazardEndY:double, robotStartX:double, robotStartY:double, robotEndX:double, robotEndY:double) : Boolean {
            val slopeRobot: Double = (robotStartY - robotEndY) / (robotStartX - robotEndX)
            val slopeHazard: Double (robotStartY-robotEndY)/(robotStartX-robotEndX)
            if (slopeRobot == slopeHazard){ //paralell line case
                return true
            }
            val intersectionX: Double = (slopeRobot * robotStartX - slopeHazard * hazardStartX + hazardStartY - robotStartY) / (slopeRobot - slopeHazard)
            val intersectionY: Double = slopeRobot(intersectionX - robotStartX) + robotStartY
            if (intersectionX >= hazardStartX || intersectionX <= hazardEndX) {
                if (intersectionY >= hazardStartY || intersectionY <= hazardEndY) {
                    return false
                }
            }
        }


    }
}
