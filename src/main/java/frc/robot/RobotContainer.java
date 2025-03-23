// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.MoveElevatorToSetpoint;
import frc.robot.commands.MoveIntakeCommand;
import frc.robot.commands.TimedDriveCommand;

public class RobotContainer {

        final CommandXboxController driverXbox = new CommandXboxController(0);
        final CommandXboxController operatorXbox = new CommandXboxController(1);
        private ElevatorSubsystem s_ElevatorSubsystem = null;
        private IntakeSubsystem s_IntakeSubsystem = null;
        private ArmSubsystem s_ArmSubsystem = new ArmSubsystem();

        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));

        private static final String auto_None = "No auto";
        private static final String auto_TaxiOut = "Taxi Out";
        private static final String auto_Level1 = "Score Level 1";
        private static final String auto_Level2 = "Score Level 2";
        private static final String auto_Level3 = "Score Level 3";
        private static final String auto_Level4 = "Score Level 4";
        private String m_autoSelected;
        private final SendableChooser<String> m_chooser = new SendableChooser<>();

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * 0.75,
                        () -> driverXbox.getLeftX() * 0.75)
                        .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.5)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        public RobotContainer() {

                m_chooser.setDefaultOption("No Auto", auto_None);
                m_chooser.addOption("Taxi Out", auto_TaxiOut);
                m_chooser.addOption("Score Level 1", auto_Level1);
                m_chooser.addOption("Score Level 2", auto_Level2);
                m_chooser.addOption("Score Level 3", auto_Level3);
                m_chooser.addOption("Score Level 4", auto_Level4);
                SmartDashboard.putData("Auto choices", m_chooser);

                configureBindings();

                DriverStation.silenceJoystickConnectionWarning(true);
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command testDrive = drivebase.driveFieldOriented(driveRobotOriented);

                drivebase.setDefaultCommand(testDrive);
                if (s_ElevatorSubsystem != null) {
                        driverXbox.start().onTrue(
                                        new SequentialCommandGroup(
                                                        new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel0)));
                        driverXbox.x().onTrue(
                                        new SequentialCommandGroup(
                                                        new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel1)));
                        driverXbox.y().onTrue(
                                        new SequentialCommandGroup(
                                                        new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel2)));
                        driverXbox.b().onTrue(
                                        new SequentialCommandGroup(
                                                        new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel3)));
                        driverXbox.a().onTrue(
                                        new SequentialCommandGroup(
                                                        new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                        ElevatorConstants.kLevel4)));
                }

                if (s_IntakeSubsystem != null) {
                        driverXbox.leftBumper().onTrue(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0.4).schedule();
                        })).onFalse(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0).schedule();
                        }));

                        driverXbox.rightBumper().onTrue(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, -0.4).schedule();
                        })).onFalse(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0).schedule();
                        }));
                }

                // operatorXbox.x().onTrue(new InstantCommand(() -> {
                // new ArmMoveCommand(s_ArmSubsystem, 0.3).schedule();
                // })).onFalse(new InstantCommand(() -> {
                // new ArmMoveCommand(s_ArmSubsystem, 0).schedule();
                // }));
                // operatorXbox.b().onTrue(new InstantCommand(() -> {
                // new ArmMoveCommand(s_ArmSubsystem, -0.3).schedule();
                // })).onFalse(new InstantCommand(() -> {
                // new ArmMoveCommand(s_ArmSubsystem, 0).schedule();
                // }));

                // operatorXbox.y().onTrue(new InstantCommand(() -> {
                // new ArmWheelSpinCommand(s_ArmSubsystem, 0.3).schedule();
                // })).onFalse(new InstantCommand(() -> {
                // new ArmWheelSpinCommand(s_ArmSubsystem, 0).schedule();
                // }));
                // operatorXbox.a().onTrue(new InstantCommand(() -> {
                // new ArmWheelSpinCommand(s_ArmSubsystem, -0.3).schedule();
                // })).onFalse(new InstantCommand(() -> {
                // new ArmWheelSpinCommand(s_ArmSubsystem, 0).schedule();
                // }));

                // // operatorXbox.back().onTrue(new InstantCommand(() -> {
                // new MoveArmCommand(s_ArmSubsystem, 0).schedule();
                // }));

                // s_ArmSubsystem.setDefaultCommand(new
                // ControlWheelsWithJoystickCommand(s_ArmSubsystem, operatorXbox));
                // s_ArmSubsystem.setDefaultCommand(new
                // ControlArmWithJoystickCommand(s_ArmSubsystem, operatorXbox));

                // operatorXbox.a().onTrue(new SequentialCommandGroup(
                // new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel4),
                // new MoveIntakeCommand(s_IntakeSubsystem, 0.3)));

                // driverXbox.povRight().onTrue(new AlignToReefTagRelative(true,
                // drivebase).withTimeout(7));
                // driverXbox.povLeft().onTrue(new AlignToReefTagRelative(false,
                // drivebase).withTimeout(7));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                m_autoSelected = m_chooser.getSelected();
                switch (m_autoSelected) {
                        case auto_None:
                                return null; // Do fucking nothing at all.
                        case auto_TaxiOut:
                                return new SequentialCommandGroup(
                                                new TimedDriveCommand(drivebase, new Translation2d(0.7, 0), 3));
                        case auto_Level1:
                                return new SequentialCommandGroup(
                                                new TimedDriveCommand(drivebase, new Translation2d(0.7, 0), 3),
                                                new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                ElevatorConstants.kLevel1),
                                                new WaitCommand(1),
                                                new MoveIntakeCommand(s_IntakeSubsystem, -0.2));
                        case auto_Level2:
                                return new SequentialCommandGroup(
                                                new TimedDriveCommand(drivebase, new Translation2d(0.7, 0), 3),
                                                new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                ElevatorConstants.kLevel2),
                                                new WaitCommand(1),
                                                new MoveIntakeCommand(s_IntakeSubsystem, -0.2));
                        case auto_Level3:
                                return new SequentialCommandGroup(
                                                new TimedDriveCommand(drivebase, new Translation2d(0.7, 0), 3),
                                                new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                ElevatorConstants.kLevel3),
                                                new WaitCommand(1),
                                                new MoveIntakeCommand(s_IntakeSubsystem, -0.2));
                        case auto_Level4:
                                return new SequentialCommandGroup(
                                                new TimedDriveCommand(drivebase, new Translation2d(0.7, 0), 3),
                                                new MoveElevatorToSetpoint(s_ElevatorSubsystem,
                                                                ElevatorConstants.kLevel4),
                                                new WaitCommand(1),
                                                new MoveIntakeCommand(s_IntakeSubsystem, -0.2));
                        default:
                                return null;
                }
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
