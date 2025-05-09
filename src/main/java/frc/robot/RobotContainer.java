// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    XboxController m_xbox = new XboxController(0);
    Joystick m_farm = new Joystick(1);

    // The robot's subsystems and commands are defined here...
    private final DriveTrain m_driveTrain = new DriveTrain();
    // private final Vision m_vision = new Vision(m_driveTrain);
    // private final Climber m_climber = new Climber();
    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake();
    
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // only used for tuning
        // JoystickButton xboxYButton = new JoystickButton(m_xbox, Constants.XBOX_Y);
        // xboxYButton.whileHeld(new TuneShooterCommand(m_shooter, m_intake));

        // FOR TESTING!!
        // DriverStation.silenceJoystickConnectionWarning(true);
        
        // // vacuum mode
        // JoystickButton xboxYButton = new JoystickButton(m_xbox, Constants.XBOX_Y);
        // xboxYButton.whileTrue(new VacuumMode(m_shooter, m_intake));

        // // actual shooter command

        // // shooting for upperHub
        // // JoystickButton xboxXButton = new JoystickButton(m_xbox, Constants.XBOX_X);
        // // xboxXButton.onTrue(new TurnAndShoot(m_shooter, m_intake, m_driveTrain, m_vision, m_driveCommand));

        // shooting for upperHub from tarmac
        JoystickButton xboxAButton = new JoystickButton(m_xbox, Constants.XBOX_A);
        xboxAButton.onTrue(new ShooterCommand(m_shooter, m_intake, Constants.TARMAC_DEFAULT_DISTANCE, true));

        // shooting for lowerHub
        JoystickButton xboxBButton = new JoystickButton(m_xbox, Constants.XBOX_B);
        xboxBButton.onTrue(new ShooterCommand(m_shooter, m_intake, ShooterCommand.DEFAULT_DISTANCE_TO_THE_HUB, false));

        // Intake commands
        
        JoystickButton bumperRight = new JoystickButton(m_xbox, Constants.XBOX_RB);
        bumperRight.whileTrue(new IntakeCommand(m_intake, Constants.INTAKE_SPEED));
    
        JoystickButton bumperLeft = new JoystickButton(m_xbox, Constants.XBOX_LB);
        bumperLeft.whileTrue(new IntakeCommand(m_intake, -Constants.INTAKE_SPEED));

        // // farm controller
        // JoystickButton farm1 = new JoystickButton(m_farm, 1);
        // farm1.onTrue(new SetElevatorHeightTest(m_climber));

        // JoystickButton farm2 = new JoystickButton(m_farm, 2);
        // farm2.onTrue(new SetArmAngleTest(m_climber));

        // JoystickButton farm3 = new JoystickButton(m_farm, 3);
        // farm3.onTrue(new SetOneElevatorHeightTest(m_climber));

        // JoystickButton farm4 = new JoystickButton(m_farm, 4);
        // farm4.onTrue(new AdjustRobotAngleTest(m_driveTrain));

        // JoystickButton farm6 = new JoystickButton(m_farm, 6);
        // farm6.onTrue(new SetClimber(m_climber));

        // JoystickButton farm7 = new JoystickButton(m_farm, 7);
        // farm7.onTrue(new RaiseToBar(m_climber).withTimeout(Constants.RAISE_TO_BAR_TIMEOUT));

        // JoystickButton farm8 = new JoystickButton(m_farm, 8);
        // farm8.onTrue(new ClimbToNextBar(m_climber).withTimeout(Constants.CLIMB_TO_NEXT_BAR_TIMEOUT));

        // JoystickButton farm11 = new JoystickButton(m_farm, 11);
        // farm11.onTrue(new ResetClimber(m_climber));

        // Additional manual shooter position buttons(orange buttons)
        // JoystickButton farm4 = new JoystickButton(m_farm, 4);
        // farm4.onTrue(new ShooterCommand(m_shooter, m_intake, Constants.TARMAC_DEFAULT_DISTANCE, true));
    
        // JoystickButton farm5 = new JoystickButton(m_farm, 5);
        // farm5.onTrue(new ShooterCommand(m_shooter, m_intake, Constants.JUST_OUTSIDE_TARMAC, true));

        // JoystickButton farm9 = new JoystickButton(m_farm, 9);
        // farm9.onTrue(new ShooterCommand(m_shooter, m_intake, Constants.CLOSE_LAUNCHPAD_SHOOTER_DISTANCE, true));

        // JoystickButton farm10 = new JoystickButton(m_farm, 10);
        // farm10.onTrue(new ShooterCommand(m_shooter, m_intake, Constants.FAR_LAUNCHPAD_SHOOTER_DISTANCE, true));
        // // end additional shooter buttons

        // // Arm Brake/Coast buttons
        // JoystickButton farm13 = new JoystickButton(m_farm, 13);
        // farm13.onTrue(new SetArmCoast(m_climber));

        // JoystickButton farm15 = new JoystickButton(m_farm, 15);
        // farm15.onTrue(new SetArmBrake(m_climber));        
        
        // //Bind buttons for vision modes.
        // JoystickButton farm12 = new JoystickButton(m_farm, 12);
        // farm12.onTrue(new SetVisionMode(m_vision, VisionMode.INTAKE)); 

        // JoystickButton farm14 = new JoystickButton(m_farm, 14);
        // farm14.onTrue(new SetVisionMode(m_vision, VisionMode.SHOOTER)); 

        // JoystickButton farm16 = new JoystickButton(m_farm, 16);
        // farm16.onTrue(new SetVisionMode(m_vision, VisionMode.HUBFINDER)); 

        // For Testing
        // JoystickButton farm10 = new JoystickButton(m_farm, 10);
        // xboxYButton.onTrue(new FaceShootingTarget(m_driveTrain, m_vision, Constants.TURN_TOLERANCE_DEG, m_driveCommand)); 
    }

    private class Throttle implements DoubleSupplier {
        @Override
        public double getAsDouble() {
            // the controller does <0 is forward
            return -m_xbox.getLeftY();
        }
    }

    private class Turn implements DoubleSupplier {
        @Override
        public double getAsDouble() {
            return -0.75 * m_xbox.getRightX();
        }
    }

    /*
     * Getters for Commands and Subsystems. Notice that it's public, meaning that
     * outsiders can access it.
     */
    
    public DriveCommand getDriveCommand() {
        return new DriveCommand(m_driveTrain, new Throttle(), new Turn());
    }

    public DriveTrain getDriveTrain(){
        return m_driveTrain;
    }
    
    // public Vision getVision() {
    //     return m_vision;
    // }
    
    // public Climber getClimber(){
    //     return m_climber;
    // }

    // public Shooter getShooter(){
    //     return m_shooter;
    // }

    // public Intake getIntake(){
    //     return m_intake;
    // }

    // LigerBots: we don't use this function.
    // Autonomous is controlled by a Chooser defined in Robot.
    //
    // public Command getAutonomousCommand() {
    // return null;
    // }
}