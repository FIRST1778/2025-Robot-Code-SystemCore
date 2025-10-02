package org.chillout1778

import choreo.Choreo
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.hal.HAL
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.chillout1778.commands.AutoRunnerCommand
import org.chillout1778.commands.TeleopDriveCommand
import org.chillout1778.commands.TeleopSuperstructureCommand
import org.chillout1778.subsystems.*
import kotlin.system.measureTimeMillis


object Robot : TimedRobot() {
    val isRedAlliance get() =
        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red

    val isOnRedSide get() =
        Swerve.estimatedPose.x > (Constants.Field.FIELD_X_SIZE / 2)

    val enableCoastModeSwitch = DigitalInput(Constants.DioIds.DISABLE_BREAK_MODE)

    var wasEnabled = false
    var wasEnabledLED = false
    private val autoChooser: SendableChooser<Trajectory<SwerveSample>> = SendableChooser()

    init {
        // Report to the FMS that we use Kotlin
        HAL.reportUsage("Language", "Kotlin")

        // Start logging NetworkTables changes to a USB drive or the RoboRIO.
        DataLogManager.start()

        // Initialize subsystem objects just by referencing them
        Arm
//        AutoContainer
        Elevator
        Intake
        Superstructure
        LoggingManager
        Swerve
        Vision
//        Lights

        SmartDashboard.putData(Arm)
        SmartDashboard.putData(Elevator)
        SmartDashboard.putData(Intake)
        SmartDashboard.putData(Superstructure)
        SmartDashboard.putData(Vision)
//
        for(trajectoryName in Choreo.availableTrajectories().filterNot{it == "VariablePoses"}) {
            autoChooser.addOption(trajectoryName, Choreo.loadTrajectory<SwerveSample>(trajectoryName).get())
        }
//

        autoChooser.onChange { t ->
            autoTrajectory = t
            initializeAutonomousCommand()
        }
//
        SmartDashboard.putData(autoChooser)

        DriverStation.silenceJoystickConnectionWarning(true)
    }

    var tickNumber: Long = 0
    override fun robotPeriodic() {
        tickNumber++
        CommandScheduler.getInstance().run()

        val totalLoggingTime = measureTimeMillis {
            LoggingManager.update()
        }
        if (totalLoggingTime > 5) {
            println("3D logging code took $totalLoggingTime ms")
            println("${LoggingManager.times}")
        }
    }

    var wasCoastModeEnabled: Boolean = false

    override fun disabledInit() {
        wasEnabled = false
    }
    override fun disabledPeriodic() {
        val pressed = !enableCoastModeSwitch.get()
        if (!wasCoastModeEnabled && pressed) { // rising edge
            Elevator.setCoastEnabled(true)
            Arm.setCoastEnabled(true)
            wasCoastModeEnabled = true
        } else if (wasCoastModeEnabled && !pressed) { // falling edge
            Elevator.setCoastEnabled(false)
            Arm.setCoastEnabled(false)
            wasCoastModeEnabled = false
        }
    }

    override fun disabledExit() {
        Elevator.setCoastEnabled(false)
        Arm.setCoastEnabled(false)
        wasEnabled = true
        wasEnabledLED = true
    }
    // This is code for running autonomous Commands only in auto mode
    private var didAutoRun = false
    private lateinit var autoTrajectory: Trajectory<SwerveSample>

    private var autonomousCommand: Command = InstantCommand()
    private fun initializeAutonomousCommand() {
        autonomousCommand = Superstructure.makeZeroAllSubsystemsCommand().andThen( // EXTREMELY IMPORTANT TO ZERO
            AutoRunnerCommand(autoTrajectory)
        ).andThen(TeleopDriveCommand(Controls::emptyInputs))
//        Shuffleboard.getTab("Robot").add("Auto runner command", runnerSubCommand)
    }

    override fun autonomousInit() {
        Arm.hasObject = true
        Arm.autoTimer.reset()
        Arm.autoTimer.start()
        didAutoRun = true
        autonomousCommand.schedule()
    }

    override fun autonomousExit() {
        autonomousCommand.cancel()
    }

    override fun teleopInit() {
        if (!didAutoRun)
            Swerve.gyroAngle = if(isRedAlliance) Math.PI else 0.0
        Superstructure.makeZeroAllSubsystemsCommand().schedule()
        Swerve.defaultCommand = TeleopDriveCommand(Controls::driverInputs)
        Superstructure.defaultCommand = TeleopSuperstructureCommand()
    }


    override fun teleopExit() {
        Swerve.removeDefaultCommand()
        Superstructure.removeDefaultCommand()
    }

    override fun testInit() {
        CommandScheduler.getInstance().cancelAll()
        Superstructure.makeZeroAllSubsystemsCommand().schedule()
        Swerve.gyroAngle = 0.0
    }
}
