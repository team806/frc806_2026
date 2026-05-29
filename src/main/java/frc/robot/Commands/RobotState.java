package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Blinkin;

public class RobotState extends Command {
    private Arm arm;
    private Indexer indexer;
    private Blinkin blinkin;

    public RobotState(Arm arm, Indexer indexer, Blinkin blinkin) {
        this.arm = arm;
        this.indexer = indexer;
        this.blinkin = blinkin;
        addRequirements(blinkin);
    }

    @Override
    public void execute() {
        Constants.Arm.States armState = arm.getState();
        Constants.Indexer.States indexerState = indexer.getState();

        if (armState == Constants.Arm.States.Deployed && indexerState == Constants.Indexer.States.Forwards) {
            blinkin.setColor(Constants.Blinkin.SolidColors.VIOLET.value);
        }
        else if (armState == Constants.Arm.States.Deployed) {
            blinkin.setColor(Constants.Blinkin.SolidColors.BLUE.value);
        }
        else if (indexerState == Constants.Indexer.States.Forwards) {
            blinkin.setColor(Constants.Blinkin.SolidColors.RED.value);
        }
        else {
            blinkin.setColor(Constants.Blinkin.SolidColors.WHITE.value);
        }
    }
}
