package frc.robot

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase


// Check out https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
// It has data on when you can shoot
class TimerTelemetry : SubsystemBase() {

    private val timerTab = Shuffleboard.getTab(Constants.GENERAL_TAB)
    private var timerWidget = timerTab.add("Time:", 0.0).entry
    private var canScoreWidget = timerTab.add("Can score?", false).entry

    init {

    }

    override fun periodic() {
        val ct = Timer.getMatchTime() // Current time
        timerWidget.setDouble(ct)
        if (ct > 20) {
            canScoreWidget.setBoolean(true)
        }
    }
}