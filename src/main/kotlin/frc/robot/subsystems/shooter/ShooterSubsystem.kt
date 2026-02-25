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
        shuffleboardTab.add("Stop Intake", stop()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Shoot", shoot()).withWidget(BuiltInWidgets.kCommand)
    }

    fun startIntake(): Command = run {
        io.setIntakePercent(0.4)
    }

    fun shoot(): Command = SequentialCommandGroup(
        runOnce {
            io.setShooterPercent(1.0)
        },
        WaitCommand(1.0),
        runOnce {
            io.stop()
        }
    )


    fun stop(): Command = run {
        io.stop()
    }
}