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

    init {
        shuffleboardTab.add("Start Intake", startIntake()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Shoot", revUpAndStartShoot()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Stop", stop()).withWidget(BuiltInWidgets.kCommand)
    }

    fun startIntake(): Command = run {
        io.setStorageIntake(0.7)
    }

    fun revUpAndStartShoot(): Command = SequentialCommandGroup(
        runOnce {
            io.setShooter(1.0)
        },
        WaitCommand(0.5),
        runOnce {
            io.setShooterIntake(0.8)
        }
    )


    fun stop(): Command = run {
        io.stop()
    }
}