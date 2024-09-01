package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CameraIO {
  @AutoLog
  public static class CameraIOInputs {
    public Rotation2d xOffset = new Rotation2d();
    public Rotation2d yOffset = new Rotation2d();
    public boolean targetAquired = false;
    public int totalTargets = 0;
    public double averageDistance = 0.0;
    public double frameTimestamp = 0.0;
    public Pose3d primaryPose = new Pose3d();
    public Pose3d secondaryPose = new Pose3d();
  }

  public default void updateInputs(CameraIOInputs inputs) {}

  public default Rotation2d getXOffset() {
    return new Rotation2d();
  }

  public default Rotation2d getYOffset() {
    return new Rotation2d();
  }

  public default boolean getTargetAquired() {
    return false;
  }

  public default int getTotalTargets() {
    return 0;
  }

  public default double getAverageDistance() {
    return 0.0;
  }

  public default double getFrameTimestamp() {
    return 0.0;
  }

  public default Pose3d getPrimaryPose() {
    return new Pose3d();
  }

  public default Pose3d getSecondaryPose() {
    return new Pose3d();
  }

  public default long getPipeline() {
    return 0;
  }

  public default String getName() {
    return "";
  }

  public default CameraType getCameraType() {
    return CameraType.DEFAULT;
  }

  public default double getHorizontalFOV() {
    return 0.0;
  }

  public default double getVerticalFOV() {
    return 0.0;
  }

  public default double getPrimaryXYStandardDeviationCoefficient() {
    return 0.0;
  }

  public default double getSecondaryXYStandardDeviationCoefficient() {
    return 0.0;
  }

  public default void setPipeline(int pipeline) {}
}
