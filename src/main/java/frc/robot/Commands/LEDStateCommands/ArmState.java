package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Blinkin;

public class ArmState extends Command {
    private Arm arm;
    private Blinkin blinkin;

    public ArmState(Arm arm, Blinkin blinkin) {
        this.arm = arm;
        this.blinkin = blinkin;
        addRequirements(blinkin);
    }

    @Override
    public void execute() {
        Constants.Arm.States armTargetState = arm.getState();
        Constants.Arm.States armActualState = arm.getActualState();

        if (armTargetState != armActualState) {
            blinkin.setColor(Constants.Blinkin.SolidColors.RED.value);
        }
        else if (armTargetState == Constants.Arm.States.Deployed) {
            blinkin.setColor(Constants.Blinkin.SolidColors.GREEN.value);
        }
        else if (armTargetState == Constants.Arm.States.Vertical) {
            blinkin.setColor(Constants.Blinkin.SolidColors.BLUE.value);
        }
        else if (armTargetState == Constants.Arm.States.Back) {
            blinkin.setColor(Constants.Blinkin.SolidColors.ORANGE.value);
        }

    }
}
