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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.commands.ArmWheelSpinCommand;
import frc.robot.commands.DistanceDriveCommand;
import frc.robot.commands.MoveElevatorToSetpoint;
import frc.robot.commands.MoveIntakeCommand;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.commands.TimedIntakeCommand;

public class RobotContainer {

        final CommandXboxController driver_controller = new CommandXboxController(0);
        final CommandXboxController operator_controller = new CommandXboxController(1);
        private ElevatorSubsystem s_ElevatorSubsystem = new ElevatorSubsystem();
        private IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
        private ArmSubsystem s_ArmSubsystem = new ArmSubsystem();

        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve/neo"));

        private String m_autoSelected;
        private final SendableChooser<String> m_chooser = new SendableChooser<>();
        private static final String auto_None = "No auto";
        private static final String auto_TaxiOut = "Taxi Out";
        private static final String auto_Level1 = "Score Level 1";
        private static final String auto_Level2 = "Score Level 2";
        private static final String auto_Level3 = "Score Level 3";
        private static final String auto_Level4 = "Score Level 4";

        SwerveInputStream driveRobotOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driver_controller.getLeftY() * -0.75,
                        () -> driver_controller.getLeftX() * -0.75)
                        .withControllerRotationAxis(() -> driver_controller.getRightX() * -0.6)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .robotRelative(true)
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

        public SequentialCommandGroup generateElevatorCommand(double setpoint) {
                double currentPosition = s_ElevatorSubsystem.getElevator1Position();

                if (setpoint == ElevatorConstants.kLevel4) {
                        return new SequentialCommandGroup(new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem, 190),
                                        new MoveElevatorToSetpoint(s_ElevatorSubsystem, setpoint));
                } else {
                        if (currentPosition < setpoint) {
                                // Moving up: Use an intermediate setpoint
                                double intermediateSetpoint = (currentPosition + setpoint) / 2.0;
                                return new SequentialCommandGroup(
                                                new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                                new MoveElevatorToSetpoint(s_ElevatorSubsystem, intermediateSetpoint),
                                                new MoveElevatorToSetpoint(s_ElevatorSubsystem, setpoint));
                        } else {
                                // Moving down: Go directly to the final setpoint
                                return new SequentialCommandGroup(
                                                new ArmMoveCommand(s_ArmSubsystem, ArmConstants.armSafe),
                                                new MoveElevatorToSetpoint(s_ElevatorSubsystem, setpoint));
                        }
                }

        }

        private void configureBindings() {
                Command testDrive = drivebase.driveFieldOriented(driveRobotOriented);

                drivebase.setDefaultCommand(testDrive);
                if (s_ElevatorSubsystem != null) {
                        driver_controller.start().onTrue(
                                        generateElevatorCommand(ElevatorConstants.kLevel0));
                        driver_controller.x().onTrue(
                                        generateElevatorCommand(ElevatorConstants.kLevel1));
                        driver_controller.y().onTrue(
                                        generateElevatorCommand(ElevatorConstants.kLevel2));
                        driver_controller.b().onTrue(
                                        generateElevatorCommand(ElevatorConstants.kLevel3));
                        driver_controller.a().onTrue(
                                        generateElevatorCommand(ElevatorConstants.kLevel4));
                }

                if (s_IntakeSubsystem != null) {
                        driver_controller.leftBumper().onTrue(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0.4).schedule();
                        })).onFalse(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0).schedule();
                        }));

                        driver_controller.rightBumper().onTrue(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, -0.4).schedule();
                        })).onFalse(new InstantCommand(() -> {
                                new MoveIntakeCommand(s_IntakeSubsystem, 0).schedule();
                        }));
                }

                operator_controller.y().onTrue(
                                new SequentialCommandGroup(new ArmMoveCommand(s_ArmSubsystem, 0)));
                operator_controller.a().onTrue(
                                new SequentialCommandGroup(new ArmMoveCommand(s_ArmSubsystem, 10)));

                operator_controller.x().onTrue(new InstantCommand(() -> {
                        new ArmWheelSpinCommand(s_ArmSubsystem, 0.3).schedule();
                })).onFalse(new InstantCommand(() -> {
                        new ArmWheelSpinCommand(s_ArmSubsystem, 0).schedule();
                }));
                operator_controller.b().onTrue(new InstantCommand(() -> {
                        new ArmWheelSpinCommand(s_ArmSubsystem, -0.3).schedule();
                })).onFalse(new InstantCommand(() -> {
                        new ArmWheelSpinCommand(s_ArmSubsystem, 0).schedule();
                }));
        }

        public Command scoreCoral() {
                return new TimedIntakeCommand(s_IntakeSubsystem, -0.2, 3);
        }

        private final int REEF_DISTANCE = 85;

        public Command getAutonomousCommand() {
                m_autoSelected = m_chooser.getSelected();
                switch (m_autoSelected) {
                        case auto_None:
                                return null; // Do fucking nothing at all.
                        case auto_TaxiOut:
                                return new SequentialCommandGroup(
                                                new DistanceDriveCommand(drivebase, new Translation2d(-1, 0), 40, 0.3));
                        case auto_Level1:
                                return new SequentialCommandGroup(
                                                new DistanceDriveCommand(drivebase, new Translation2d(-1, 0),
                                                                REEF_DISTANCE, 0.3),
                                                generateElevatorCommand(ElevatorConstants.kLevel1),
                                                scoreCoral(),
                                                generateElevatorCommand(ElevatorConstants.kLevel0));
                        case auto_Level2:
                                return new SequentialCommandGroup(
                                                new DistanceDriveCommand(drivebase, new Translation2d(-1, 0),
                                                                REEF_DISTANCE, 0.3),
                                                generateElevatorCommand(ElevatorConstants.kLevel2),
                                                scoreCoral(),
                                                generateElevatorCommand(ElevatorConstants.kLevel0));
                        case auto_Level3:
                                return new SequentialCommandGroup(
                                                new DistanceDriveCommand(drivebase, new Translation2d(-1, 0),
                                                                REEF_DISTANCE, 0.3),
                                                generateElevatorCommand(ElevatorConstants.kLevel3),
                                                scoreCoral(),
                                                generateElevatorCommand(ElevatorConstants.kLevel0));
                        case auto_Level4:
                                return new SequentialCommandGroup(
                                                new DistanceDriveCommand(drivebase, new Translation2d(-1, 0), 80, 0.3),
                                                generateElevatorCommand(ElevatorConstants.kLevel4),
                                                scoreCoral(),
                                                generateElevatorCommand(ElevatorConstants.kLevel0),
                                                new DistanceDriveCommand(drivebase, new Translation2d(1, 0), 10, 0.3));
                        default:
                                return null;
                }
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
