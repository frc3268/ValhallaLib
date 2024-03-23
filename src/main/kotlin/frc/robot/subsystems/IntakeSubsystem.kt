package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.*
import frc.lib.utils.*
import frc.robot.Constants

class IntakeSubsystem: SubsystemBase() {
    private val intakeMotor = Motor(9)
    private val armMotor = Motor(10)
    val armEncoder: RelativeEncoder = armMotor.encoder
    val intakeEncoder: RelativeEncoder = intakeMotor.encoder
    val armPIDController = PIDController(0.7/170,0.0,0.0)

    val shuffleboardTab = Shuffleboard.getTab(Constants.TROUBLESHOOTING_TAB)
    val intakeArmPositionEntry = shuffleboardTab.add("Arm pos", 0.0)
            .withPosition(3, 0)
            .entry
    val intakePositionEntry = shuffleboardTab.add("Intake pos", 0.0)
            .withPosition(3, 1)
            .entry
    val intakeVelocityEntry = shuffleboardTab.add("Intake velocity", 0.0)
            .withPosition(3, 2)
            .entry

    // TODO replace with actual channel
    val limitSwitch = DigitalInput(0)

    companion object {
        const val INTAKE_SPEED = 0.30
        const val OUTTAKE_ADJUST_SPEED = -0.3
        const val OUTTAKE_SPEED = -0.9
        const val SHOOT_AMP_SPEED = -1.0
        //based momento...

        const val UP_ANGLE = 5.0
        const val DOWN_ANGLE = 169.0
    }

    init {
        armMotor.inverted = true
        armEncoder.positionConversionFactor = 360 / 112.5
        intakeEncoder.velocityConversionFactor = 1.0 / 1600
        intakeEncoder.positionConversionFactor = 1.0/12.0
    }

    fun stopIntake(): Command =
        runOnce { intakeMotor.stopMotor() }

    /*
    intakeAndStopCommand: Command
    runs the intake motor at the speed necessary to intake a game piece
    waits until a game piece enters the intake
    waits until the game piece has been compressed to a sufficient level
    stops the intake motor

    this is done by checking the velocity of the intake motor, given that the motor will run slower when the motion of the wheels is inhibited by game pieces
     */
    fun intakeAndStopCommand(): Command =
        run{intakeMotor.setVoltage(INTAKE_SPEED * 12.0)}.withTimeout(1.0).andThen(
            run{}.until{intakeEncoder.velocity < 1.0}.andThen(runOnce{intakeEncoder.setPosition(0.0)}.andThen(
               run{}.until{intakeEncoder.position > 1.1}.andThen(stopIntake()))
            )
        )

    fun stopArm(): Command =
        runOnce { armMotor.stopMotor() }

    /**
     * Stops both the arm and intake gears immediately.
     */
    fun stopAllCommand(): Command =
        SequentialCommandGroup(
            stopIntake(),
            stopArm()
        )

    /**
     * Runs the intake gears at [speed].
     */
    fun runIntakeAtSpeed(speed: Double): Command =
        runOnce { intakeMotor.setVoltage(speed * 12.0) }

    fun armUpCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, UP_ANGLE-5.0)) }
            .until { getArmPosition().degrees < UP_ANGLE }
            .andThen(stopArm())

    fun armDownCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, DOWN_ANGLE+5.0)) }
            .until { getArmPosition().degrees > DOWN_ANGLE }
            .andThen(runOnce{armMotor.set(-0.01)})

    /**
     * Sets the arm to the amp shoot or source intake angle, which are the same.
     */
    fun armToAmpAngleCommand(): Command =
        run { armMotor.set(armPIDController.calculate(getArmPosition().degrees, 105.0)) }
            .until { getArmPosition().degrees > 100.0 }
            .andThen(stopArm())

    /**
     * Brings the arm up to the amp angle and shoots the note into the amp.
     */
    fun ampCommand(): Command =
        SequentialCommandGroup(
            armToAmpAngleCommand(),
            runIntakeAtSpeed(SHOOT_AMP_SPEED),
            WaitCommand(2.0),
            stopIntake(),
            armUpCommand()
        )

    /**
     * Brings the arm up to the source intake angle and then intakes a note.
     */
    fun armUpAndIntakeCommand(): Command =
        SequentialCommandGroup(
            armToAmpAngleCommand(),
            runIntakeAtSpeed(INTAKE_SPEED),
            WaitCommand(0.1),
            stopIntake(),
            armUpCommand()
        )

    fun takeInCommand(): Command =
        SequentialCommandGroup(
            runIntakeAtSpeed(INTAKE_SPEED),
            armDownCommand(),
            WaitCommand(0.8),
        )

    fun takeOutCommand(): Command =
        SequentialCommandGroup(
            armUpCommand(),
            runIntakeAtSpeed(OUTTAKE_SPEED)
        )

    fun zeroArmEncoderCommand(): Command =
        runOnce { armEncoder.position = 0.0 }

    fun getArmPosition() : Rotation2d =
        armEncoder.position.rotation2dFromDeg()

    override fun periodic() {

        // Stop arm guard in case it screws itself over

        if (getArmPosition().degrees >= 190.0 || getArmPosition().degrees <= -5.0)
            stopArm().schedule()

        intakeArmPositionEntry.setDouble(armEncoder.position)
        intakeVelocityEntry.setDouble(intakeEncoder.velocity)
        intakePositionEntry.setDouble(intakeEncoder.position)
    }

    override fun simulationPeriodic() {
    }
}