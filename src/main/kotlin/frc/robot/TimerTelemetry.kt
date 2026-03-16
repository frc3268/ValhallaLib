package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase



// Check out https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
// It has data on when you can shoot
class TimerTelemetry : SubsystemBase() {

    private val timerTab = Shuffleboard.getTab(Constants.GENERAL_TAB)
    private var timerWidget = timerTab.add("Time until shift:", 0.0).entry
    private var canScoreWidget = timerTab.add("Can score?", false).entry

    override fun periodic() {
        canScoreWidget.setBoolean(isHubActive())
    }

    private fun isHubActive(): Boolean {
        val alliance = DriverStation.getAlliance()
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty) {
            return false
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false
        }

        // We're teleop enabled, compute.
        val matchTime = DriverStation.getMatchTime()
        val gameData = DriverStation.getGameSpecificMessage()
        // If we have no game data, we cannot compute, assume hub is active, as It's likely early in teleop.
        if (gameData.isEmpty()) {
            return true
        }
        val redInactiveFirst: Boolean = when (gameData[0]) {
            'R' -> true
            'B' -> false
            else -> {
                // If we have invalid game data, assume hub is active.
                return true
            }
        }
        // Shift was active for blue if red won auto, or red if blue won auto.
        val shift1Active = when (alliance.get()) {
            Alliance.Red -> !redInactiveFirst
            Alliance.Blue -> redInactiveFirst
        }
        if (matchTime > 130) {
            // Transition shift, hub is active.
            timerWidget.setDouble(matchTime - 130)
            return true
        } else if (matchTime > 105) {
            // Shift 1
            timerWidget.setDouble(matchTime - 105)
            return shift1Active
        } else if (matchTime > 80) {
            // Shift 2
            timerWidget.setDouble(matchTime - 80)
            return !shift1Active
        } else if (matchTime > 55) {
            // Shift 3
            timerWidget.setDouble(matchTime - 55)
            return shift1Active
        } else if (matchTime > 30) {
            // Shift 4
            timerWidget.setDouble(matchTime - 30)
            return !shift1Active
        } else {
            // End game, hub always active.
            timerWidget.setDouble(matchTime)
            return true
        }
    }
}