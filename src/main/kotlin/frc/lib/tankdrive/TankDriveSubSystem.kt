package frc.lib.tankdrive

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import kotlin.math.max

class TankDriveSubsystem(val io: TankDriveIO) : SubsystemBase() {

    private val shuffleboardTab = Shuffleboard.getTab("Tankdrive")

    fun driveForward(velocity: Double): Command = run {
        io.setVelocity(velocity)
    }

    fun turnLeft(velocity: Double): Command = run {
        io.setVelocity(-velocity, velocity)
    }

    fun turnRight(velocity: Double): Command = run {
        io.setVelocity(velocity, -velocity)
    }

    fun arcadeDrive(rotate: Double, drive: Double): Command = run {
        val maximum = max(abs(drive), abs(rotate))
        val total = rotate+drive
        val difference = drive-rotate

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