// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Blinkin;

public class RobotContainer implements Sendable{
    public CommandXboxController driveController = new CommandXboxController(0);
    CommandXboxController coDriveController = new CommandXboxController(1);
    CommandXboxController ohShitController = new CommandXboxController(2);

    private final Trigger driveRightTrigger = driveController.rightTrigger(0.5);
    private final Trigger drivekLeftBumper = driveController.leftBumper();
    private final Trigger drivekRightBumper = driveController.rightBumper();

    public final Drivetrain drivetrain = new Drivetrain(Constants.Drivetrain.moduleArray, driveController, Constants.Drivetrain.CameraName);
    
    public final Indexer indexer = new Indexer(Constants.Indexer.TopRollerID);
    public final Shooter shooter = new Shooter(Constants.Shooter.MotorID);
    public final Intake intake = new Intake(Constants.Intake.RollerID);
    public final Arm arm = new Arm(Constants.Arm.ArmLeaderID, Constants.Arm.ArmFollowerID);

    public final Blinkin blinkin = new Blinkin(0, Constants.Blinkin.SolidColors.WHITE.value, arm, indexer);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        configureNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser("Nothing");
        SmartDashboard.putData("Auto chooser", autoChooser);

        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    private void configureBindings() {
        driveRightTrigger.whileTrue(indexer.index());
        drivekRightBumper.toggleOnTrue(arm.top());
        drivekLeftBumper.whileTrue(arm.bump());
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Index", indexer.index());
        NamedCommands.registerCommand("Stop Indexer", indexer.stop());
        NamedCommands.registerCommand("Shoot", shooter.shoot());
        NamedCommands.registerCommand("Stop Shooter", shooter.stop());
        NamedCommands.registerCommand("Intake", intake.intake());
        NamedCommands.registerCommand("Discharge", intake.discharge());
        NamedCommands.registerCommand("Stop Intake", intake.stop());
        NamedCommands.registerCommand("Deploy arm", arm.deploy());
        NamedCommands.registerCommand("Bump arm", arm.bump());
        NamedCommands.registerCommand("Top arm", arm.top());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Drivetrain enabled", drivetrain::getIsEnabled, (newVal) -> drivetrain.toggle(newVal));
        builder.addBooleanProperty("Indexer enabled", indexer::getIsEnabled, (newVal) -> indexer.toggle(newVal));
        builder.addBooleanProperty("Shooter enabled", shooter::getIsEnabled, (newVal) -> shooter.toggle(newVal));
        builder.addBooleanProperty("Intake enabled", intake::getIsEnabled, (newVal) -> intake.toggle(newVal));
        builder.addBooleanProperty("Arm enabled", arm::getIsEnabled, (newVal) -> arm.toggle(newVal));
        builder.addBooleanProperty("Blinkin enabled", blinkin::getIsEnabled, (newVal) -> blinkin.toggle(newVal));
    }
}
