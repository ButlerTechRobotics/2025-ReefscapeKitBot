package frc.robot.subsystems.roller;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class RollerIOTalonSRX implements RollerIO {
  private final TalonSRX rollerMotor = new TalonSRX(6);

  public RollerIOTalonSRX() {
    var config = new TalonSRXConfiguration();
    config.peakCurrentLimit = 80;
    config.peakCurrentDuration = 250;
    config.continuousCurrentLimit = 60;
    config.voltageCompSaturation = 12.0;
    rollerMotor.configAllSettings(config);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.appliedVolts = rollerMotor.getMotorOutputVoltage();
    inputs.currentAmps = new double[] {rollerMotor.getSupplyCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotor.set(TalonSRXControlMode.PercentOutput, volts * 12.0);
  }
}
