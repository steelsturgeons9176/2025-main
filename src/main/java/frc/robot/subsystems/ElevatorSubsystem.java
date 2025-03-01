package frc.robot.subsystems;

import java.util.EnumMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorSubsystem extends SubsystemBase {
    public enum elevatorPositions{
        L1, 
        L2, 
        L3,
        L4,
        SOURCE
    }

    private final SparkMax m_armLead;
    private final SparkMax m_armFollow;

    private SparkMaxConfig m_leadConfig;
    private SparkMaxConfig m_followConfig;

    private final RelativeEncoder ElevatorEncoder;

    //private final PIDController m_AbsPidController = new PIDController(1, 0.0, 0.0);

    private final SparkClosedLoopController m_pidController;

    private double currentGoal = 0.125f;

    DoubleSubscriber currentGoalSub;

   
    private Timer m_timer;

    private double kS_tuner = 0;

    private double kP = 1;
    private double kI = 0;
    private double kD = 0.0;

  

    private double kG = .19;
    private double kS = .4;
    private double kV = 0;
    private double kA = 0;

    private double p;
    private double i;
    private double d;

    private boolean isTuning = false;

    private double getP()
    {
        return p;
    }
    private double getI()
    {
        return i;
    }
    private double getD()
    {
        return d;
    }

    private ElevatorFeedforward ff =
    new ElevatorFeedforward(kS, kG ,kV, kA);


    private final TrapezoidProfile profile;

    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    EnumMap<elevatorPositions, Double> mapAbs = new EnumMap<>(elevatorPositions.class);

    double m_speed = 0.0;

    //DoublePublisher kG_gain = NetworkTableInstance.getDefault()
    //.getDoubleTopic("MyStates", ).publish();

    public ElevatorSubsystem () {
        m_timer = new Timer();
        m_timer.start();
        m_timer.reset();

        mapAbs.put(elevatorPositions.L1, ElevatorConstants.L1);
        mapAbs.put(elevatorPositions.L2, ElevatorConstants.L2);
        mapAbs.put(elevatorPositions.L3, ElevatorConstants.L3);
        mapAbs.put(elevatorPositions.L4, ElevatorConstants.L4);
        mapAbs.put(elevatorPositions.SOURCE, ElevatorConstants.SOURCE);
      //  mapAbs.put(armPositions.INTAKE, ArmConstants.INTAKE);
      //  mapAbs.put(armPositions.POOP, ArmConstants.POOP);

        m_armLead = new SparkMax(ElevatorConstants.kLeadMotorID, MotorType.kBrushless);
        m_armFollow = new SparkMax(ElevatorConstants.kFollowMotorID, MotorType.kBrushless);

        m_pidController = m_armLead.getClosedLoopController();

        ElevatorEncoder = m_armLead.getEncoder();

        m_followConfig.inverted(true);
        m_followConfig.follow(m_armLead);

        //m_pidController = m_armLead.getClosedLoopController();

        p = kP;
        i = kI;
        d = kD;

        m_leadConfig.closedLoop.pid(kP, kI, kD);

       // TODO
        m_leadConfig.closedLoop.outputRange(ElevatorConstants.kElevatorMinOutput, ElevatorConstants.kElevatorMaxOutput);

        m_leadConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
        
        m_followConfig.idleMode(ElevatorConstants.kElevatorIdleMode);

        //m_pidController.enableContinuousInput(0, 1);
        profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(8, 4));

        m_leadConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
       // m_followConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      
       // m_followConfig.closedLoop.outputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput);
     //   m_leadConfig.closedLoop.velocityFF(ElevatorConstants.kElevatorFF);
       // m_followConfig.closedLoop.velocityFF(ElevatorConstants.kElevatorFF);
      
      //  m_followConfig.closedLoop.pid(kP, kI, kD);

        //m_leadConfig.closedLoop.setPositionPIDWrappingEnabled(true);
        //m_pidController.setPositionPIDWrappingMinInput(0.0f);
        //m_pidController.setPositionPIDWrappingMaxInput(1.0f);
        
        m_armLead.configure(m_leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_armFollow.configure(m_followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Arm/goal", currentGoal);
        SmartDashboard.putNumber("Arm/KP", kP);
        SmartDashboard.putNumber("Arm/KI", kI);
        SmartDashboard.putNumber("Arm/KD", kD);

        SmartDashboard.putNumber("Arm/KS", kS);
        SmartDashboard.putNumber("Arm/KG", kG);
        SmartDashboard.putNumber("Arm/KV", kV);
        SmartDashboard.putNumber("Arm/KA", kA);

        SmartDashboard.putNumber("Arm/KSTuner", kS_tuner);

        //setDefaultCommand(new ArmToPosition(this, armPositions.STOWED));
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Arm Relative Enc", m_armRight.getEncoder().getPosition());
        SmartDashboard.putNumber("ArmABS Absolute", ElevatorEncoder.getPosition());
        //SmartDashboard.putNumber("Arm oCurrent", m_armRight.getOutputCurrent());
        //SmartDashboard.putNumber("Arm Motor Speed", m_speed);
        //SmartDashboard.putNumber("LeftMotor", m_armRight.getOutputCurrent());
        //SmartDashboard.putNumber("Right Motor", m_armLeft.getOutputCurrent());

        if(isTuning)
        {
            tuneNumbers();
        }

    

        m_speed = m_armLead.getEncoder().getVelocity();
        SmartDashboard.putNumber("setpointState", setpointState.position);
        if(profile.isFinished(m_timer.get() + .06))
        {
            setpointState = new TrapezoidProfile.State(currentGoal, 0);
            updateMotionProfile();
             setpointState =
          profile.calculate(
              m_timer.get(),
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      currentGoal,
                      Constants.ArmConstants.kMinHeightAbs,
                      Constants.ArmConstants.kMaxHeightAbs),
                   0.0));
        }
        else{
        setpointState =
          profile.calculate(
              m_timer.get(),
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      currentGoal,
                      Constants.ArmConstants.kMinHeightAbs,
                      Constants.ArmConstants.kMaxHeightAbs),
                   0.0));
                
        }

        m_pidController.setReference(setpointState.position, ControlType.kPosition,ClosedLoopSlot.kSlot0,
        ff.calculate(setpointState.position * 2 * Math.PI, setpointState.velocity * 2 * Math.PI ));
        //m_AbsPidController.

        //SmartDashboard.putNumber("ff", ff.calculate(setpointState.position, setpointState.velocity));

    }

//        SmartDashboard.putNumber("voltage", m_armRight.getBusVoltage());


    private void updateMotionProfile() {
        m_timer.reset();
      }

    public boolean raiseArmAbs(elevatorPositions position){
        if (((ElevatorEncoder.getPosition() < ArmConstants.kMinHeightAbs) && (position == elevatorPositions.L1)) ||
            ((ElevatorEncoder.getPosition() > ArmConstants.kMaxHeightAbs) && (position == elevatorPositions.L4))) {
            //m_armRight.set(0);
            //return true;
        }
        
        double ref = mapAbs.get(position);
        currentGoal = ref;
        updateMotionProfile();


        //double pidOut = MathUtil.clamp(
        //    m_AbsPidController.calculate(armAbsEncoder.getPosition(),ref),
        //    Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);
        //m_pidController.setReference(setpointState.position, ControlType.kPosition);

        //m_armRight.
            
        //SmartDashboard.putNumber("Arm Abs Target Pos", ref);
 //       m_armRight.set(pidOut);
        
        //if(atPosition(position))
       // {
       //     return true;
       // }
        return false;
    }

    public void updatePID() 
    {
        m_leadConfig.closedLoop.pid(kP, kI, kD);
        
        p = kP;
        i = kI;
        d = kD;
    }

    public boolean atPosition(){
        double currentEncoderPosition = ElevatorEncoder.getPosition();
        return (Math.abs(currentEncoderPosition - currentGoal) < Constants.ArmConstants.kAllowedErrAbs);
    }

    public void noArmPower()
    {
        m_armLead.set(0);
    }

    public void tuneNumbers()
    {
        currentGoal = SmartDashboard.getNumber("Arm/goal", currentGoal);
        kP = SmartDashboard.getNumber("Arm/KP", kP);
        kI = SmartDashboard.getNumber("Arm/KI", kI);
        kD = SmartDashboard.getNumber("Arm/KD", kD);

        kS = SmartDashboard.getNumber("Arm/KS", kS); 
        kG = SmartDashboard.getNumber("Arm/KG", kG);
        kV = SmartDashboard.getNumber("Arm/KV", kV);
        kA = SmartDashboard.getNumber("Arm/KA", kA);

        kS_tuner = SmartDashboard.getNumber("Arm/KSTuner", kS_tuner);

        if( getP() != kP || getI() != kI || getD() != kD)
        {
            updatePID();
        }

        if(ff.getKa() != kA || ff.getKg() != kG || ff.getKs() != kS || ff.getKv() != kV)
        {
            ff = new ElevatorFeedforward(kS, kG, kV, kA);
        }
    }

}