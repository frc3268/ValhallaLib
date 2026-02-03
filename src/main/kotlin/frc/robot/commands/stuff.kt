package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.lib.motor.SparkMaxMotor

class xboxthing(
    private val motor: SparkMaxMotor,
    private val controller: CommandXboxController
) : Command() {

    override fun execute() {
        val speed = -controller.leftY
        motor.setVelocity(speed)
    }

    override fun end(interrupted: Boolean) {
        motor.stop()
    }
}
