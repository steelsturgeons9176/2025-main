package frc.robot.subsystems;

import frc.robot.Constants.FunnelConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class FunnelSubsystem extends SubsystemBase{
    private final SparkMax m_leadNeo;
    private final SparkMax m_fallowNeo;

    private final SparkMaxConfig m_neoConfig;

    public FunnelSubsystem() {
        m_leadNeo = new SparkMax(FunnelConstants.KLeadNeoId, MotorType.kBrushless);
        m_fallowNeo = new SparkMax(FunnelConstants.KFallowNeoId, MotorType.kBrushless);

        m_neoConfig = new SparkMaxConfig();

        m_neoConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15)
            .inverted(true);

        m_leadNeo.configure(m_neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_neoConfig.inverted(false);

        m_fallowNeo.configure(m_neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setVoltage(double voltage) {
        m_leadNeo.setVoltage(voltage);
        m_fallowNeo.setVoltage(voltage);
    }
}
