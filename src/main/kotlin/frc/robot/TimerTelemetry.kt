package frc.robot

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard

class TimerTelemetry : SubsystemBase() {

    private val timerTab = Shuffleboard.getTab(Constants.GENERAL_TAB)
    private var Timer = timerTab.add("Time:", 0.0).entry

    override fun periodic() {
        // Timer.setDouble()
    }
}