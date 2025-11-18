package frc.lib.tankdrive

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.Command

class TankDriveSubSystem(val io: TankDriveIO) : SubsystemBase(){
    fun turnLeft(velocity : Double): Command = run {
        io.setVelocity(-velocity, velocity)
    }
    fun turnRight(velocity: Double) : Command = run {
        io.setVelocity(velocity,-velocity)
    }
    fun stop(): Command = run {
        io.stop();
    }
}