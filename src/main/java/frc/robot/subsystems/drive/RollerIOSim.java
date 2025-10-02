package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class RollerIOSim implements RollerIO {
  private final FlywheelSim motorSimModel;
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  Distance radius = Inches.of(1.5);
  double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);

  public RollerIOSim() {
    super();
    DCMotor motor = DCMotor.getCIM(1);
    LinearSystem<N1, N1, N1> linearSystem = LinearSystemId.createFlywheelSystem(motor, moi, 1.5);
    motorSimModel = new FlywheelSim(linearSystem, motor);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(motorSimModel.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      motorSimModel.setInputVoltage(appliedVolts);
    }

    motorSimModel.update(0.02);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = motorSimModel.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {motorSimModel.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    motorSimModel.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}