package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Constants

class ShooterSubsystem(val io: IShooterIO) : SubsystemBase() {

    private val shuffleboardTab = Shuffleboard.getTab(Constants.GENERAL_TAB)
    private val troubleshootingTab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)

    private val debugShooterSpeed =
        troubleshootingTab.addPersistent("Direct shooter speed", 0.4).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(mapOf("min" to -1, "max" to 1)).entry
    private val debugIntakeSpeed =
        troubleshootingTab.addPersistent("Direct intake speed", 0.4).withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(mapOf("min" to -1, "max" to 1)).entry

    init {
        shuffleboardTab.add("Start Intake", startIntake()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Rev Up & Shoot", revUpAndStartShoot()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Stop Shooter & Intake", stop()).withWidget(BuiltInWidgets.kCommand)

        troubleshootingTab.add("Shoot (Direct)", directShoot()).withWidget(BuiltInWidgets.kCommand)
        troubleshootingTab.add("Intake (Direct)", directIntake()).withWidget(BuiltInWidgets.kCommand)
    }

    fun startIntake(): Command = run {
        io.setIntake(0.7)
    }

    fun revUpAndStartShoot(): Command = SequentialCommandGroup(
        runOnce {
            io.setShooter(1.0)
        },
        WaitCommand(0.5),
        runOnce {
            io.setIntakeForShooter(0.8)
        }
    )

    fun stop(): Command = runOnce {
        io.stop()
    }

    private fun directShoot(): Command = runOnce {
        io.setShooter(debugShooterSpeed.getDouble(0.0))
    }

    private fun directIntake(): Command = runOnce {
        io.setIntakeForShooter(debugIntakeSpeed.getDouble(0.0))
    }

    override fun periodic() {
        io.log()
    }
}