package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Blinkin;

public class RobotState extends SubsystemBase {
    private Arm arm;
    private Indexer indexer;
    private Blinkin blinkin;

    public RobotState(Arm arm, Indexer indexer, Blinkin blinkin) {
        this.arm = arm;
        this.indexer = indexer;
        this.blinkin = blinkin;
    }

    public void SetBlinkin() {
        Constants.RobotState.ArmStates armState = arm.getState();
        Constants.RobotState.IndexerStates indexerState = indexer.getState();

        if (armState == Constants.RobotState.ArmStates.Deployed && indexerState == Constants.RobotState.IndexerStates.Forwards) {
            blinkin.setColor(Constants.Blinkin.SolidColors.VIOLET.value);
        }
        else if (armState == Constants.RobotState.ArmStates.Deployed) {
            blinkin.setColor(Constants.Blinkin.Strobe.BLUE.value);
        }
        else if (indexerState == Constants.RobotState.IndexerStates.Forwards) {
            blinkin.setColor(Constants.Blinkin.Strobe.RED.value);
        }
        else {
            blinkin.setColor(Constants.Blinkin.Strobe.WHITE.value);
        }

    }

    @Override
    public void periodic() {
        SetBlinkin();
    }

}
