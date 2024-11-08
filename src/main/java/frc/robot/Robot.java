// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Yeeter;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.ArmExtension.ExtensionPosition;
import frc.robot.subsystems.ArmRotation.RotationAngle;
import frc.robot.subsystems.Intake.intakePosition;
import frc.logging.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

public class Robot extends TimedRobot {
  private ArmExtension armExtension = new ArmExtension();
  private ArmRotation armRotation = new ArmRotation();
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();
  private Yeeter yeeter = new Yeeter();
  private Intake intake = new Intake();
  private Lights lights = new Lights();

  private SendableChooser<Command> autonSelector = null;
  private GenericEntry brakeModeDisable = Shuffleboard.getTab("Autonomous").add("Disable Brake Mode", false).getEntry();

  private Command autonCommand = none();

  public Command setBlueCommand(){
    return runOnce(()->{lights.setBlue(1);});
  }

  public Command changeColorsCommand(){
    return runOnce(()->{lights.setColor();});
  }
  
  public Command getShootCommand() {
    return either(
        sequence(
            parallel(
                intake.SetIntakePosition(intakePosition.raised),
                armRotation.rotateToPosition(RotationAngle.Raised),
                runOnce(() -> {
                  yeeter.SetVoltage(12);
                }, yeeter)),
            runOnce(() -> {
              intake.setVoltage(12);
            }, intake), waitSeconds(.5),
            parallel(
                intake.SetIntakePosition(intakePosition.up),
                armRotation.rotateToPosition(RotationAngle.Down)))
            .finallyDo(() -> {
              intake.stopIntake();
              yeeter.Stop();
            }),
        sequence(
            runOnce(() -> {
              yeeter.SetVoltage(8);
            }, yeeter),
            waitSeconds(0.5),
            runOnce(() -> {
              yeeter.Stop();
            }),
            either(
                sequence(armExtension.extendToPosition(ExtensionPosition.In),
                    armRotation.rotateToPosition(RotationAngle.Down),
                    intake.SetIntakePosition(intakePosition.up)),
                none(), () -> {
                  return armRotation.getRotation() == RotationAngle.Amp;
                }))
            .finallyDo(() -> {
              yeeter.Stop();
            }),
        () -> {
          return intake.hasNote();
        });
  }

  public Command getHandoffCommand() {

    return either(sequence(
        parallel(
            intake.SetIntakePosition(intakePosition.raised),
            armRotation.rotateToPosition(RotationAngle.Raised)),
        runOnce(() -> {
          intake.setVoltage(6);
          yeeter.SetVoltage(2);
        }, intake, yeeter),
        waitUntil(() -> {
          return !intake.hasNote();
        }),
        runOnce(() -> {
          intake.stopIntake();
          yeeter.Stop();
        }, intake, yeeter)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming),
        none(), () -> {
          return intake.hasNote();
        });
  }

  @Override
  public void robotInit() {
    SmartDashboard.putNumberArray("Angles", new double[]{18, 37} );
    Logger.AutoLog(this);
    configureAuton();

    SetupLogging();

    new Trigger(() -> {
      return brakeModeDisable.getBoolean(false);
    }).onTrue(runOnce(() -> {
      armRotation.setBrake(false);
      armExtension.setBrake(false);
      intake.setBrake(false);
    }).ignoringDisable(true)).onFalse(runOnce(() -> {
      armRotation.setBrake(true);
      armExtension.setBrake(true);
      intake.setBrake(true);
    }).ignoringDisable(true));

    disabled().onTrue(
        runOnce(() -> {
          armExtension.disable();
          armRotation.disable();
          intake.disable();
        }).ignoringDisable(true))
        .onFalse(runOnce(() -> {
          armExtension.enable();
          armRotation.enable();
          intake.enable();
        }).ignoringDisable(true));

    m_driveTrain.setDefaultCommand(m_driveTrain.getDriveCommand(() -> {
      return -controller.getRawAxis(0);
    }, () -> {
      return -controller.getRawAxis(1);
    }, () -> {
      return controller.getRawAxis(4);
    }, true));
    controller.back().onTrue(runOnce(() -> {
      m_driveTrain.zero();
    }));

    controller.rightBumper().whileTrue(intake.intakeNote(controller))
        .onFalse(intake.SetIntakePosition(intakePosition.up));

    controller.leftBumper().whileTrue(run(() -> {
      intake.setVoltage(-4);
    }).finallyDo(() -> {
      intake.stopIntake();
    }));

    controller.y().onTrue(getShootCommand());
    controller.povRight().onTrue(changeColorsCommand());
    controller.povLeft().onTrue(setBlueCommand());

    controller.b().and(() -> {
      return armRotation.atGoal() && armRotation.getRotation() == ArmRotation.RotationAngle.Down;
    }).onTrue(
        sequence(
            getHandoffCommand(),
            armRotation.rotateToPosition(RotationAngle.Amp),
            armExtension.extendToPosition(ExtensionPosition.Amp)));

    controller.povDown().onTrue(
        sequence(
            runOnce(() -> {
              armExtension.climbSpeed();
              armRotation.climbSpeed();
            }),
            armExtension.extendToPosition(ExtensionPosition.In),
            armRotation.rotateToPosition(RotationAngle.Down)).finallyDo(() -> {
              armExtension.normalSpeed();
              armRotation.normalSpeed();
            }));

    controller.povUp().and(() -> {
      return armRotation.atGoal() && armRotation.getRotation() == ArmRotation.RotationAngle.Down;
    }).onTrue(
        sequence(
            armRotation.rotateToPosition(RotationAngle.Trap),
            sequence(
                runOnce(() -> {
                  yeeter.SetVoltage(-.65);
                }),
                waitUntil(() -> {
                  return !yeeter.hasNote();
                }).withTimeout(.5)).finallyDo(() -> {
                  yeeter.Stop();
                }),
            armExtension.extendToPosition(ExtensionPosition.Trap)));

    
    test().and(controller.back()).onTrue(
      runOnce(()->{
        Filesystem.getDeployDirectory().delete();
      })
    );
  }

  private double[] getAnglesForDistance(double distFromSpeaker) {
    double[] ret = SmartDashboard.getNumberArray("Angles", new double[]{0, 14});
    SmartDashboard.putNumber("Shot Distance", distFromSpeaker);
    return ret;
  }

  void configureAuton() {
    NamedCommands.registerCommand("Shoot", getShootCommand().asProxy());
    NamedCommands.registerCommand("Amp", sequence(
        getHandoffCommand(),
        armRotation.rotateToPosition(RotationAngle.Amp),
        armExtension.extendToPosition(ExtensionPosition.Amp),
        getShootCommand()
        ));
    SmartDashboard.putData(intake);
    autonSelector = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Chooser", autonSelector);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
 
  @Override
  public void autonomousInit() {
    Command cmd = intake.intakeNote(controller).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    intake.setDefaultCommand(cmd);
    autonCommand = autonSelector.getSelected();
    autonCommand.schedule();
  }

  @Override
  public void teleopInit() {
    Command cmd = none();
    cmd.addRequirements(intake);
    intake.setDefaultCommand(cmd);
    autonCommand.cancel();

  }

  public void SetupLogging(){
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }
}
