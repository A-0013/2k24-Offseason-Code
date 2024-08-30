package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean hasNote() {
    return inputs.intakeSensor || inputs.middleSensor || inputs.finalSensor;
  }

  public Command intake() {
    return Commands.sequence(
        Commands.parallel(
                Commands.runEnd(() -> io.setTopVoltage(-12.0), () -> io.setTopVoltage(0.0)),
                Commands.runEnd(() -> io.setBottomVoltage(12.0), () -> io.setBottomVoltage(0.0)),
                Commands.runEnd(
                    () -> io.setAcceleratorVoltage(12.0), () -> io.setAcceleratorVoltage(0.0)))
            .until(() -> inputs.middleSensor),
        Commands.parallel(
                Commands.runEnd(() -> io.setTopVoltage(-3.0), () -> io.setTopVoltage(0.0)),
                Commands.runEnd(() -> io.setBottomVoltage(3.0), () -> io.setBottomVoltage(0.0)),
                Commands.runEnd(
                    () -> io.setAcceleratorVoltage(3.0), () -> io.setAcceleratorVoltage(0.0)))
            .until(() -> inputs.finalSensor));
  }

  public Command eject() {
    return Commands.parallel(
        Commands.runEnd(() -> io.setTopVoltage(12.0), () -> io.setTopVoltage(0.0)),
        Commands.runEnd(() -> io.setBottomVoltage(-12.0), () -> io.setBottomVoltage(0.0)));
  }
}
