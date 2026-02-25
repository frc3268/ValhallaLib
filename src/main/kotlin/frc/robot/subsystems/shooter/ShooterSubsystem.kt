package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Constants


class ShooterSubsystem(val io: IShooterIO) : SubsystemBase() {
    private val shuffleboardTab = Shuffleboard.getTab(Constants.GENERAL_TAB)

    init {
        shuffleboardTab.add("Start Intake", startIntake()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("End Intake", stopIntake()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Stop Intake", stop()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("Shoot", shoot()).withWidget(BuiltInWidgets.kCommand)
    }

    fun startIntake(): Command = run {
        io.setIntakePercent(0.4)
    }

    fun stopIntake(): Command = run {
        io.setIntakePercent(0.0)
    }

    fun shoot(): Command = run {
        io.setShooterPercent(0.4)
    }.andThen(
        WaitCommand(0.3)
    ).andThen(
        run {
            io.setShooterPercent(0.0)
        }
    )


    fun stop(): Command = run {
        io.stop()
    }
}