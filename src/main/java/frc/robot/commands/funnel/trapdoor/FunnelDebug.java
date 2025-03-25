package frc.robot.commands.funnel.trapdoor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.intake.Intake;

public class FunnelDebug extends Command{
  private FunnelSubsystem m_funnel;


  public FunnelDebug(FunnelSubsystem funnel){
    m_funnel = funnel;
      addRequirements(m_funnel);
  }

  @Override
  public void initialize(){
    m_funnel.setVoltage(4);;
  }

  @Override
  public void execute(){

  }

  @Override
  public boolean isFinished(){
      return false;
  }

  @Override
  public void end(boolean isInterrupted){
    m_funnel.setVoltage(0);
  }
}
