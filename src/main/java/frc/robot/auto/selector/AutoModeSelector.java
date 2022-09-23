package frc.robot.auto.selector;

import frc.robot.auto.selector.AutoModeList;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.modes.*;

public class AutoModeSelector {
    private final SendableChooser<AutoModeList> mModeChooser;

    private final DoNothing DoNothing = new DoNothing();

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("DO NOTHING", AutoModeList.DO_NOTHING);
        //mModeChooser.addOption(name, enum);

        SmartDashboard.putData(mModeChooser);

    }

    public void UpdateAutoModeSelector() {
    }


}
