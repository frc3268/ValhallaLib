package frc.lib.tankdrive

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.camera.Camera
import kotlin.math.abs
import kotlin.math.max

class TankDriveSubsystem(val io: TankDriveIO) : SubsystemBase() {

    private val shuffleboardTab = Shuffleboard.getTab("Tankdrive")
    private var camera: Camera? = null
    //private var poseEstimator: DifferentialDrivePoseEstimator

    init {
//        poseEstimator = DifferentialDrivePoseEstimator(
//
//        )
    }

    fun driveForward(velocity: LinearVelocity): Command = run {
        io.setVelocityBoth(velocity)
    }

    fun turnLeft(velocity: LinearVelocity): Command = run {
        io.setVelocity(-velocity, velocity)
    }

    fun turnRight(velocity: LinearVelocity): Command = run {
        io.setVelocity(velocity, -velocity)
    }

    fun arcadeDrive(rotate: Double, drive: Double): Command = run {
        val maximum = Units.FeetPerSecond.of(max(abs(drive), abs(rotate)))
        val total = Units.FeetPerSecond.of(rotate + drive)
        val difference = Units.FeetPerSecond.of(drive - rotate)
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
        io.left1.stop()
        io.right1.stop()
    }
}