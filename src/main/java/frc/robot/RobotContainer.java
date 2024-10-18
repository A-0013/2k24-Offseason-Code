// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.module.ModuleConstants;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Intake intake;
  private Vision vision;
  private Climber climber;
  private Shooter shooter;
  private Arm arm;
  private Leds leds;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard Inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case WHIPLASH:
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(ModuleConstants.FRONT_LEFT),
                  new ModuleIOTalonFX(ModuleConstants.FRONT_RIGHT),
                  new ModuleIOTalonFX(ModuleConstants.REAR_LEFT),
                  new ModuleIOTalonFX(ModuleConstants.REAR_RIGHT));
          intake = new Intake(new IntakeIOTalonFX());
          vision = new Vision();
          climber = new Climber(new ClimberIOTalonFX());
          shooter = new Shooter(new ShooterIOTalonFX());
          arm = new Arm(new ArmIOTalonFX());
          leds = new Leds();
          break;
        case ROBOT_SIM:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          intake = new Intake(new IntakeIOSim());
          vision = new Vision();
          climber = new Climber(new ClimberIOSim());
          shooter = new Shooter(new ShooterIOSim());
          arm = new Arm(new ArmIOSim());
          leds = new Leds();
          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (intake == null) {
      intake = new Intake(new IntakeIO() {});
    }
    if (vision == null) {
      vision = new Vision();
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    if (shooter == null) {
      shooter = new Shooter(new ShooterIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }

    // Configure auto choices.
    autoChooser = new LoggedDashboardChooser<>("Auto Routines");
    autoChooser.addDefaultOption("None", AutoRoutines.none());
    autoChooser.addOption(
        "Amp Side", AutoRoutines.getBoatBattleAmpAuto(drive, intake, arm, shooter));
    autoChooser.addOption(
        "Source Side", AutoRoutines.getBoatBattleSourceAuto(drive, intake, arm, shooter));
    if (Constants.TUNING_MODE) {
      autoChooser.addOption("Shooter Characterization", shooter.runCharacterization());
      autoChooser.addOption(
          "Arm Quasistatic Forward", arm.runQuasistaticCharacterization(Direction.kForward));
      autoChooser.addOption(
          "Arm Quasistatic Reverse", arm.runQuasistaticCharacterization(Direction.kReverse));
      autoChooser.addOption(
          "Arm Dynamic Forward", arm.runDynamicCharacterization(Direction.kForward));
      autoChooser.addOption(
          "Arm Dynamic Reverse", arm.runDynamicCharacterization(Direction.kReverse));
      autoChooser.addOption(
          "Drive Quasistatic Forward",
          DriveCommands.runSysIdQuasistatic(drive, Direction.kForward));
      autoChooser.addOption(
          "Drive Quasistatic Reverse",
          DriveCommands.runSysIdQuasistatic(drive, Direction.kReverse));
      autoChooser.addOption(
          "Drive Dynamic Forward", DriveCommands.runSysIdDynamic(drive, Direction.kForward));
      autoChooser.addOption(
          "Drive Dynamic Reverse", DriveCommands.runSysIdDynamic(drive, Direction.kReverse));
    }

    Shuffleboard.getTab("Autonomous")
        .add("Autonomous Mode", autoChooser.getSendableChooser())
        .withPosition(0, 0)
        .withSize(2, 2);
    Shuffleboard.getTab("Teleoperated")
        .addBoolean("Note?", intake::hasNoteLocked)
        .withPosition(0, 0)
        .withSize(8, 5);

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> driver.getRightX()));
    driver.y().onTrue(CompositeCommands.resetHeading(drive));//
    driver.leftBumper().whileTrue(CompositeCommands.collect(intake, arm));//
    driver.leftTrigger().whileTrue(CompositeCommands.eject(intake, arm));//
    driver.rightBumper().whileTrue(CompositeCommands.shootSubwoofer(intake, arm, shooter));
    driver.rightTrigger().whileTrue(CompositeCommands.shootAmp(intake, arm, shooter));
    driver.b().whileTrue(CompositeCommands.shootFeed(intake, arm, shooter));
    operator.povUp().whileTrue(climber.unlock());//
    operator.povDown().whileTrue(climber.climb());//
    operator.y().whileTrue(climber.deClimb());//
    driver.a().whileTrue(intake.shoot());//
    operator.leftBumper().whileTrue(CompositeCommands.collect(intake, arm));//left bumper intake
    operator.leftTrigger().whileTrue(CompositeCommands.eject(intake, arm));//left trigger outtake
    operator.rightTrigger().whileTrue(CompositeCommands.shootAmp(intake, arm, shooter));//right trigger spins it
    operator.b().whileTrue(CompositeCommands.shootFeed(intake, arm, shooter));//B shoot
    operator.rightBumper().whileTrue(CompositeCommands.shootSubwoofer(intake, arm, shooter));//right bumper autoshoots
  }

  public void robotPeriodic() {
    RobotState.periodic(
        drive.getRotation(),
        drive.getYawVelocity(),
        drive.getFieldRelativeVelocity(),
        drive.getModulePositions(),
        vision.getCameras(),
        vision.getValidTarget(),
        vision.getPrimaryVisionPoses(),
        vision.getSecondaryVisionPoses(),
        vision.getFrameTimestamps(),
        intake.hasNoteLocked(),
        intake.isIntaking(),
        climber.isClimbed());
    leds.periodic();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
