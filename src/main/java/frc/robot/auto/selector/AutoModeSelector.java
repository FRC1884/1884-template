package frc.robot.auto.selector;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.DoNothing;

public class AutoModeSelector {
  private final SendableChooser<AutoModeList> mModeChooser;

  private final DoNothing doNothing = new DoNothing();

  public AutoModeSelector() {
    mModeChooser = new SendableChooser<>();
    mModeChooser.setDefaultOption("DO NOTHING", AutoModeList.DO_NOTHING);
    // mModeChooser.addOption(name, enum);

    SmartDashboard.putData(mModeChooser);
  }

  public void updateAutoModeSelector() {}
}
