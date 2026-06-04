package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Blinkin;

public class IndexerState extends Command {
    private Indexer indexer;
    private Blinkin blinkin;

    public IndexerState(Indexer indexer, Blinkin blinkin) {
        this.indexer = indexer;
        this.blinkin = blinkin;
        addRequirements(blinkin);
    }

    @Override
    public void execute() {
        Constants.Indexer.States indexerState = indexer.getState();

        if (indexerState == Constants.Indexer.States.Stopped) {
            blinkin.setColor(Constants.Blinkin.SolidColors.RED.value);
        }
        else if (indexerState == Constants.Indexer.States.Backwards) {
            blinkin.setColor(Constants.Blinkin.SolidColors.BLUE.value);
        }
        else if (indexerState == Constants.Indexer.States.Forwards) {
            blinkin.setColor(Constants.Blinkin.SolidColors.GREEN.value);
        }
    }
}
