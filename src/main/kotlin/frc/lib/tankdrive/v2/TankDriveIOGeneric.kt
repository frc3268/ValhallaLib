package frc.lib.tankdrive.v2

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.lib.motor.IMotor
import frc.lib.tankdrive.ITankDriveIO
import frc.lib.tankdrive.v2.TankDriveConstants.TANK_DRIVE_TAB

class TankDriveIOGeneric(
    val left1: IMotor, val left2: IMotor,
    val right1: IMotor, val right2: IMotor
) : ITankDriveIO {
    private val shuffleboardTab = Shuffleboard.getTab(TANK_DRIVE_TAB)

    private var lastLeftVelocity = shuffleboardTab.add("Left Velocity", 0.0).entry
    private var lastRightVelocity = shuffleboardTab.add("Right Velocity", 0.0).entry

    init {
        left2.follow(left1)
        right2.follow(right1)
        right1.forceInvert()
        right1.configure()
    }

    override fun setLeftVelocity(velocity: AngularVelocity) {
        left1.setVelocity(velocity)
        lastLeftVelocity.setDouble(velocity.`in`(Units.RPM));
        println(velocity.toLongString())
    }

    override fun setRightVelocity(velocity: AngularVelocity) {
        right1.setVelocity(velocity)
        lastRightVelocity.setDouble(velocity.`in`(Units.RPM));
    }

    override fun setVelocity(leftVel: AngularVelocity, rightVel: AngularVelocity) {
        setLeftVelocity(leftVel)
        setRightVelocity(rightVel)
    }

    override fun setVelocityBoth(velocity: AngularVelocity) {
        setVelocity(velocity, velocity)
    }

    // Not sure if getPositionDegreeMeasurement would work in this case
    override fun getLeftDistance(): Double {
        return left1.getPositionDegreeMeasurement()
    }

    override fun getRightDistance(): Double {
        return right1.getPositionDegreeMeasurement()
    }

    override fun stop() {
        left1.stop()
        right1.stop()
        lastLeftVelocity.setDouble(0.0);
        lastLeftVelocity.setDouble(0.0);
    }
}