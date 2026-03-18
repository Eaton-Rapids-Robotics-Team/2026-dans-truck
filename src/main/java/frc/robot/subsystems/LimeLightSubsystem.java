package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.Optional;

public class LimeLightSubsystem extends SubsystemBase {
  public double txnc = 0;
  private double tync = 0;
  private double ta = 0;
  private double distToCamera = 0;
  private double distToRobot = 0;
  private double ambiguity = 0;
  private int id = 0;
  private Optional<Alliance> alliance = DriverStation.getAlliance();
  private int[] seekedIDs;

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Limelight");

  // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

  private final int[] redIds = {21, 26, 18};
  private final int[] blueIds = {2, 10, 5};

  public LimeLightSubsystem() {
    if (DriverStation.getAlliance().isPresent() && alliance.get() == Alliance.Red) {
      seekedIDs = redIds;
    } else {
      seekedIDs = blueIds;
    }
    CameraServer.startAutomaticCapture();
  }

  public void flashLight(boolean isOn) {
    if (isOn) {
      LimelightHelpers.setLEDMode_ForceOn("");
    } else {
      LimelightHelpers.setLEDMode_ForceOff("");
    }
  }

  public void updateDashboard() {
    m_table.getEntry("Id").setDouble(id);
    m_table.getEntry("Tx").setDouble(txnc);
    m_table.getEntry("Ty").setDouble(tync);
    m_table.getEntry("Ta").setDouble(ta);
    m_table.getEntry("distToCamera").setDouble(distToCamera);
    m_table.getEntry("distToRobot").setDouble(distToRobot);
    m_table.getEntry("ambiguity").setDouble(ambiguity);
  }

  public Command getUpdateDashboard() {
    return runOnce(() -> updateDashboard());
  }

  public void periodic() {
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    for (RawFiducial fiducial : fiducials) {
      id = fiducial.id; // Tag ID

      if (id == seekedIDs[0] || id == seekedIDs[1] || id == seekedIDs[2]) {
        txnc = fiducial.txnc; // X offset (no crosshair)
        tync = fiducial.tync; // Y offset (no crosshair)
        ta = fiducial.ta; // Target area
        distToCamera = fiducial.distToCamera; // Distance to camera
        distToRobot = fiducial.distToRobot; // Distance to robot
        ambiguity = fiducial.ambiguity; // Tag pose ambiguity
      }
    }
    updateDashboard();
  }
}
