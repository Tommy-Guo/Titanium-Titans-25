// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.AutoScore;
import frc.robot.commands.ControlArmWithJoystickCommand;
import frc.robot.commands.ControlWheelsWithJoystickCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.MoveElevatorToSetpoint;
import frc.robot.commands.MoveIntakeCommand;
// import frc.robot.commands.RotateToAngleCommand;

public class RobotContainer {

        final CommandXboxController operatorXbox = new CommandXboxController(1);
        final CommandXboxController driverXbox = new CommandXboxController(2);
        private ElevatorSubsystem s_ElevatorSubsystem;
        private IntakeSubsystem s_IntakeSubsystem;
        private ArmSubsystem s_ArmSubsystem;

        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));

        private final SendableChooser<Command> autoChooser;

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        // SwerveInputStream driveAngularVelocity =
        // SwerveInputStream.of(drivebase.getSwerveDrive(),
        // () -> driverXbox.getLeftY() * -1,
        // () -> driverXbox.getLeftX() * -1)
        // .withControllerRotationAxis(driverXbox::getRightX)
        // .deadband(OperatorConstants.DEADBAND)
        // .scaleTranslation(0.8)
        // .allianceRelativeControl(true);
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * 0.75,
                        () -> driverXbox.getLeftX() * -0.75)
                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.5)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                        2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverXbox.getRawAxis(
                                                        2) *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        driverXbox.getRawAxis(
                                                                        2) *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(
                                        0));

        public void RegisterNamedCommands() {
                NamedCommands.registerCommand("Down",
                                new AutoScore(s_ElevatorSubsystem, s_IntakeSubsystem, 0));
                NamedCommands.registerCommand("AutoScoreL1",
                                new AutoScore(s_ElevatorSubsystem, s_IntakeSubsystem, ElevatorConstants.kLevel1));
                NamedCommands.registerCommand("AutoScoreL2",
                                new AutoScore(s_ElevatorSubsystem, s_IntakeSubsystem, ElevatorConstants.kLevel2));
                NamedCommands.registerCommand("AutoScoreL3",
                                new AutoScore(s_ElevatorSubsystem, s_IntakeSubsystem, ElevatorConstants.kLevel3));
                NamedCommands.registerCommand("AutoScoreL4",
                                new AutoScore(s_ElevatorSubsystem, s_IntakeSubsystem, ElevatorConstants.kLevel4));
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                s_ArmSubsystem = new ArmSubsystem();
                s_IntakeSubsystem = new IntakeSubsystem();
                s_ElevatorSubsystem = new ElevatorSubsystem();

                RegisterNamedCommands();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
                UsbCamera usbCamera = CameraServer.startAutomaticCapture("Intake Camera", 0);
                usbCamera.setResolution(160, 120);
                UsbCamera usbCamera2 = CameraServer.startAutomaticCapture("Chute Camera", 1);
                usbCamera2.setResolution(160, 120);
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                // driverXbox.start().whileTrue(Commands.none());
                // driverXbox.back().whileTrue(Commands.none());

                if (s_ElevatorSubsystem != null) {
                        operatorXbox.start().onTrue(
                                        new SequentialCommandGroup(
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel0)));
                        operatorXbox.x().onTrue(
                                        new SequentialCommandGroup(
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel1)));
                        operatorXbox.y().onTrue(
                                        new SequentialCommandGroup(
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel2)));
                        operatorXbox.b().onTrue(
                                        new SequentialCommandGroup(
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel3)));
                        operatorXbox.a().onTrue(
                                        new SequentialCommandGroup(
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel4)));
                }

                if (s_IntakeSubsystem != null) {
                        operatorXbox.leftBumper().onTrue(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0.3).schedule();
                        })).onFalse(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0).schedule();
                        }));

                        operatorXbox.rightBumper().onTrue(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, -0.3).schedule();
                        })).onFalse(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0).schedule();
                        }));
                }

                // operatorXbox.back().onTrue(new InstantCommand(() -> {
                //         new MoveArmCommand(s_ArmSubsystem, 0).schedule();
                // }));

                // s_ArmSubsystem.setDefaultCommand(new ControlWheelsWithJoystickCommand(s_ArmSubsystem, operatorXbox));
                // s_ArmSubsystem.setDefaultCommand(new ControlArmWithJoystickCommand(s_ArmSubsystem, operatorXbox));

                operatorXbox.a().onTrue(new SequentialCommandGroup(
                                new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel4),
                                new MoveIntakeCommand(s_IntakeSubsystem, 0.3)));

                driverXbox.povRight().onTrue(new AlignToReefTagRelative(true, drivebase).withTimeout(7));
                driverXbox.povLeft().onTrue(new AlignToReefTagRelative(false, drivebase).withTimeout(7));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return drivebase.getAutonomousCommand("New Auto");
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
