package frc.robot;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Subzero robot. AdvantageKit "lite" (D-21): LoggedRobot + WPILOGWriter + NT4Publisher, outputs only,
 * no IO layers, no replay. SignalLogger is NOT started (A-06). Logs land in frc/logs/ in sim and
 * /home/lvuser/logs on the RoboRIO (no USB stick, H-24).
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("ProjectName", "Subzero");
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  /** A-05: agents/frc-mcp auto-enable the sim via SUBZERO_SIM_AUTOENABLE = "teleop" | "auto". */
  @Override
  public void simulationInit() {
    String mode = System.getenv("SUBZERO_SIM_AUTOENABLE");
    if (mode != null) {
      DriverStationSim.setDsAttached(true);
      DriverStationSim.setAutonomous("auto".equals(mode));
      DriverStationSim.setTest(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData(); // required — state is not seen until this is called
    }
  }

  @Override
  public void simulationPeriodic() {}
}
