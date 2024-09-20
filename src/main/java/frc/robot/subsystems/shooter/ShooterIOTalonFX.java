package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final StatusSignal<Double> topPositionRotations;
  private final StatusSignal<Double> topVelocityRotPerSec;
  private final StatusSignal<Double> topAppliedVolts;
  private final StatusSignal<Double> topCurrentAmps;
  private final StatusSignal<Double> topTemperatureCelsius;
  private final StatusSignal<Double> topVelocitySetpointRotationsPerSec;

  private final StatusSignal<Double> bottomPositionRotations;
  private final StatusSignal<Double> bottomVelocityRotPerSec;
  private final StatusSignal<Double> bottomAppliedVolts;
  private final StatusSignal<Double> bottomCurrentAmps;
  private final StatusSignal<Double> bottomTemperatureCelsius;
  private final StatusSignal<Double> bottomVelocitySetpointRotationsPerSec;

  private final TalonFXConfiguration topConfig;
  private final TalonFXConfiguration bottomConfig;

  private final NeutralOut neutralControl;
  private final VoltageOut voltageControl;
  private final MotionMagicVelocityVoltage topProfiledVelocityControl;
  private final MotionMagicVelocityVoltage bottomProfiledVelocityControl;

  private double topGoalRadiansPerSecond;
  private double bottomGoalRadiansPerSecond;

  private double topSetPointRadiansPerSecond;
  private double bottomSetPointRadiansPerSecond;

  public ShooterIOTalonFX() {

    topMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
    bottomMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);

    topConfig = new TalonFXConfiguration();
    bottomConfig = new TalonFXConfiguration();

    topConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    topConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    topConfig.Slot0.kP = ShooterConstants.KP.get();
    topConfig.Slot0.kD = ShooterConstants.KD.get();
    topConfig.Slot0.kS = ShooterConstants.KS.get();
    topConfig.Slot0.kV = ShooterConstants.KV.get();
    topConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(
            ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get());

    bottomConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    bottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bottomConfig.Slot0.kP = ShooterConstants.KP.get();
    bottomConfig.Slot0.kD = ShooterConstants.KD.get();
    bottomConfig.Slot0.kS = ShooterConstants.KS.get();
    bottomConfig.Slot0.kV = ShooterConstants.KV.get();
    bottomConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(
            ShooterConstants.MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED.get());

    topMotor.getConfigurator().apply(topConfig);
    bottomMotor.getConfigurator().apply(bottomConfig);

    topPositionRotations = topMotor.getPosition();
    topVelocityRotPerSec = topMotor.getVelocity();
    topAppliedVolts = topMotor.getMotorVoltage();
    topCurrentAmps = topMotor.getSupplyCurrent();
    topTemperatureCelsius = topMotor.getDeviceTemp();

    bottomPositionRotations = bottomMotor.getPosition();
    bottomVelocityRotPerSec = bottomMotor.getVelocity();
    bottomAppliedVolts = bottomMotor.getMotorVoltage();
    bottomCurrentAmps = bottomMotor.getSupplyCurrent();
    bottomTemperatureCelsius = bottomMotor.getDeviceTemp();

    topVelocitySetpointRotationsPerSec = topMotor.getClosedLoopReference();
    bottomVelocitySetpointRotationsPerSec = bottomMotor.getClosedLoopReference();

    topGoalRadiansPerSecond = 0.0;
    bottomGoalRadiansPerSecond = 0.0;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topPositionRotations,
        topVelocityRotPerSec,
        topAppliedVolts,
        topCurrentAmps,
        topTemperatureCelsius,
        bottomPositionRotations,
        bottomVelocityRotPerSec,
        bottomAppliedVolts,
        bottomCurrentAmps,
        bottomTemperatureCelsius);

    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();

    neutralControl = new NeutralOut();
    voltageControl = new VoltageOut(0.0);
    topProfiledVelocityControl = new MotionMagicVelocityVoltage(0);
    bottomProfiledVelocityControl = new MotionMagicVelocityVoltage(0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    inputs.topPosition = Rotation2d.fromRotations(topPositionRotations.getValueAsDouble());
    inputs.topVelocityRadPerSec = Units.rotationsToRadians(topVelocityRotPerSec.getValueAsDouble());
    inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
    inputs.topCurrentAmps = topCurrentAmps.getValueAsDouble();
    inputs.topTemperatureCelsius = topTemperatureCelsius.getValueAsDouble();
    inputs.topVelocitySetpointRadiansPerSec =
        Units.rotationsToRadians(
            topVelocitySetpointRotationsPerSec.getValueAsDouble()
                / ShooterConstants.TOP_GEAR_RATIO);
    inputs.topVelocityGoalRadiansPerSec = topGoalRadiansPerSecond;

    inputs.bottomPosition = Rotation2d.fromRotations(bottomPositionRotations.getValueAsDouble());
    inputs.bottomVelocityRadPerSec =
        Units.rotationsToRadians(bottomVelocityRotPerSec.getValueAsDouble());
    inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
    inputs.bottomCurrentAmps = bottomCurrentAmps.getValueAsDouble();
    inputs.bottomTemperatureCelsius = bottomTemperatureCelsius.getValueAsDouble();
    inputs.bottomVelocitySetpointRadiansPerSec =
        Units.rotationsToRadians(
            bottomVelocitySetpointRotationsPerSec.getValueAsDouble()
                / ShooterConstants.BOTTOM_GEAR_RATIO);
    inputs.bottomVelocityGoalRadiansPerSec = bottomGoalRadiansPerSecond;
  }

  /**
   * Sets a target value for the Top Motor's velocity in rotations per second.
   *
   * @param setPointVelocityRadiansPerSecond the target for the velocity, in radians per second, to
   *     reach.
   */
  @Override
  public void setTopVelocitySetPoint(double setPointVelocityRadiansPerSecond) {

    topGoalRadiansPerSecond = setPointVelocityRadiansPerSecond;
    topMotor.setControl(
        topProfiledVelocityControl.withVelocity(
            Units.radiansToRotations(setPointVelocityRadiansPerSecond)));
  }

  /**
   * Sets a target value for the Bottom Motor's velocity in rotations per second.
   *
   * @param setPointVelocityRadiansPerSecond the target for the velocity, in radians per second, to
   *     reach.
   */
  @Override
  public void setBottomVelocitySetPoint(double setPointVelocityRadiansPerSecond) {

    bottomGoalRadiansPerSecond = setPointVelocityRadiansPerSecond;
    bottomMotor.setControl(
        bottomProfiledVelocityControl.withVelocity(
            Units.radiansToRotations(setPointVelocityRadiansPerSecond)));
  }

  /**
   * Applies the S, V, and A gains to the Top Motor Feed Forward.
   *
   * @param kS the voltage gain
   * @param kV the velocity gain
   * @param kA the acceleration gain
   */
  @Override
  public void setTopFeedForward(double kS, double kV, double kA) {

    topConfig.Slot0.kS = kS;
    topConfig.Slot0.kV = kV;
    topConfig.Slot0.kA = kA;
    topMotor.getConfigurator().apply(topConfig, 0.01);
  }

  /**
   * Applies the S, V, and A gains to the Bottom Motor Feed Forward.
   *
   * @param kS the voltage gain
   * @param kV the velocity gain
   * @param kA the acceleration gain
   */
  @Override
  public void setBottomFeedForward(double kS, double kV, double kA) {

    topConfig.Slot0.kS = kS;
    topConfig.Slot0.kV = kV;
    topConfig.Slot0.kA = kA;
    topMotor.getConfigurator().apply(topConfig, 0.01);
  }

  /**
   * Calculates the upper and lower bounds of the top and bottom motor velocities (in radians per
   * second) and then makes sure the current top and bottom motor velocities are within these bounds
   * (i.e. at the target velocity).
   */
  @Override
  public boolean atSetPoint() {

    double topSetPointRadiansPerSecondUpperBound =
        topSetPointRadiansPerSecond
            + (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND / 2);
    double topSetPointRadiansPerSecondLowerBound =
        topSetPointRadiansPerSecond
            - (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND / 2);
    double bottomSetPointRadiansPerSecondUpperBound =
        bottomSetPointRadiansPerSecond
            + (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND / 2);
    double bottomSetPointRadiansPerSecondLowerBound =
        bottomSetPointRadiansPerSecond
            - (ShooterConstants.PROFILE_SPEED_TOLERANCE_RADIANS_PER_SECOND / 2);

    double currentTopVelocityRadiansPerSecond =
        Units.rotationsToRadians(topVelocityRotPerSec.getValueAsDouble());
    double currentBottomVelocityRadiansPerSecond =
        Units.rotationsToRadians(bottomVelocityRotPerSec.getValueAsDouble());

    return currentTopVelocityRadiansPerSecond < topSetPointRadiansPerSecondUpperBound
        && currentTopVelocityRadiansPerSecond > topSetPointRadiansPerSecondLowerBound
        && currentBottomVelocityRadiansPerSecond < bottomSetPointRadiansPerSecondUpperBound
        && currentBottomVelocityRadiansPerSecond > bottomSetPointRadiansPerSecondLowerBound;
  }

  /**
   * Sets the acceleration of the Top Motor in rotations per second squared.
   *
   * @param maxAccelerationRadiansPerSecondSquared the acceleration, in radians per second squared,
   *     that will be applied to the Top Motor.
   */
  @Override
  public void setTopProfile(double maxAccelerationRadiansPerSecondSquared) {

    topConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared);
    topMotor.getConfigurator().apply(topConfig);
  }

  /**
   * Sets the acceleration of the Bottom Motor in rotations per second squared.
   *
   * @param maxAccelerationRadiansPerSecondSquared the acceleration, in radians per second squared,
   *     that will be applied to the Bottom Motor.
   */
  @Override
  public void setBottomProfile(double maxAccelerationRadiansPerSecondSquared) {

    bottomConfig.MotionMagic.MotionMagicAcceleration =
        Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared);
    bottomMotor.getConfigurator().apply(bottomConfig);
  }

  /**
   * Applies the P, I, and D gains to the Top Motor PID.
   *
   * @param kP the proportional gain
   * @param kI the integral gain
   * @param kD the derivative gain
   */
  @Override
  public void setTopPID(double kP, double kI, double kD) {

    topConfig.Slot0.kP = kP;
    topConfig.Slot0.kI = kI;
    topConfig.Slot0.kD = kD;
    topMotor.getConfigurator().apply(topConfig, 0.01);
  }

  @Override
  public void setBottomPID(double kP, double kI, double kD) {

    bottomConfig.Slot0.kP = kP;
    bottomConfig.Slot0.kI = kI;
    bottomConfig.Slot0.kD = kD;
    bottomMotor.getConfigurator().apply(bottomConfig, 0.001);
  }

  /**
   * Takes in a value volts, and sets the voltage of both the top and bottom motors to this value.
   *
   * @param volts the voltage that will be inputted as the input voltage for the top and bottom
   *     motors.
   */
  @Override
  public void setVoltage(double volts) {

    topMotor.setControl(voltageControl.withOutput(volts));
    bottomMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void stop() {

    topMotor.setControl(neutralControl);
    bottomMotor.setControl(neutralControl);
  }
}
