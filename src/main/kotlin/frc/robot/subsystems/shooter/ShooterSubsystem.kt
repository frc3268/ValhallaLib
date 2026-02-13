package frc.robot.subsystems.shooter

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.WidgetType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.tankdrive.v2.TankDriveConstants
import frc.robot.Constants


class ShooterSubsystem(val io: IShooterIO): SubsystemBase() {
    private val shuffleboardTab = Shuffleboard.getTab(Constants.GENERAL_TAB)

    init {
        shuffleboardTab.add("startIntake", startIntake()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("endIntake", stopIntake()).withWidget(BuiltInWidgets.kCommand)
        shuffleboardTab.add("stopshooter", stop()).withWidget(BuiltInWidgets.kCommand)

    }

    fun startIntake(): Command = run {
        io.setIntakePercent(0.4)
    }

    fun stopIntake(): Command = run {
        io.setIntakePercent(0.0)
    }

    fun stop(): Command = run {
        io.stop()
    }
}