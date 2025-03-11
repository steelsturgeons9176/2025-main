package frc.robot.subsystems;

import frc.robot.Constants.EndifactorConstant;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// This is the subsystem for the funnel (or trough as liam calls it)
public class FunnelSubsystem extends SubsystemBase{
    private final SparkMax m_leadNeo;
    private final SparkMax m_fallowNeo;

    private final SparkMaxConfig m_neoConfig;

    public FunnelSubsystem() {
        m_leadNeo = new SparkMax(EndifactorConstant.KLeadNeoId, null);
        m_fallowNeo = new SparkMax(EndifactorConstant.KFallowNeoId, null);

        m_neoConfig = new SparkMaxConfig();

        m_neoConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);

        m_leadNeo.configure(m_neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_neoConfig.inverted(true);

        m_fallowNeo.configure(m_neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setVoltage(double voltage) {
        m_leadNeo.setVoltage(voltage);
        m_fallowNeo.setVoltage(voltage);
    }
}
