package frc.robot.subsystems;

import java.util.EnumMap;

import com.pathplanner.lib.path.GoalEndState;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystemNeo extends SubsystemBase {
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

    private ElevatorFeedforward ff =
    new ElevatorFeedforward(kS, kG ,kV, kA);


    private final TrapezoidProfile profile;

    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    EnumMap<elevatorPositions, Double> mapAbs = new EnumMap<>(elevatorPositions.class);

    double m_speed = 0.0;

    //DoublePublisher kG_gain = NetworkTableInstance.getDefault()
    //.getDoubleTopic("MyStates", ).publish();

    public ElevatorSubsystemNeo () {
        m_timer = new Timer();
        m_timer.start();
        m_timer.reset();

        mapAbs.put(elevatorPositions.L1, ElevatorConstants.L1_HEIGHT);
        mapAbs.put(elevatorPositions.L2, ElevatorConstants.L2_HEIGHT);
        mapAbs.put(elevatorPositions.L3, ElevatorConstants.L3_HEIGHT);
        mapAbs.put(elevatorPositions.L4, ElevatorConstants.L4_HEIGHT);
        mapAbs.put(elevatorPositions.SOURCE, ElevatorConstants.SOURCE);
      //  mapAbs.put(armPositions.INTAKE, ArmConstants.INTAKE);
      //  mapAbs.put(armPositions.POOP, ArmConstants.POOP);

        m_armLead = new SparkMax(ElevatorConstants.kLeadMotorID, MotorType.kBrushless);
        m_armFollow = new SparkMax(ElevatorConstants.kFollowMotorID, MotorType.kBrushless);

        m_pidController = m_armLead.getClosedLoopController();

        ElevatorEncoder = m_armLead.getEncoder();

        m_leadConfig = new SparkMaxConfig();
        m_followConfig = new SparkMaxConfig();

        m_leadConfig.inverted(true);
        m_followConfig.inverted(false);
        //m_followConfig.follow(m_armLead);
        m_leadConfig.smartCurrentLimit(40);
        m_followConfig.smartCurrentLimit(30);
        //m_pidController = m_armLead.getClosedLoopController();

        p = kP;
        i = kI;
        d = kD;

        m_leadConfig.closedLoop.pid(kP, kI, kD);

       // TODO
        m_leadConfig.closedLoop.outputRange(-.80, .80);

        m_leadConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
        
        m_followConfig.idleMode(ElevatorConstants.kElevatorIdleMode);

        //m_pidController.enableContinuousInput(0, 1);
        profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(1000, 750));

        m_leadConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
       
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
        SmartDashboard.putNumber("Arm/ArmABS Absolute", ElevatorEncoder.getPosition());
        //SmartDashboard.putNumber("Arm/RightSide Encoder" , m_armFollow.getAlternateEncoder().getPosition());
        //SmartDashboard.putNumber("Arm oCurrent", m_armRight.getOutputCurrent());
        //SmartDashboard.putNumber("Arm Motor Speed", m_speed);
        SmartDashboard.putNumber("Arm/LeftMotor", m_armLead.getOutputCurrent());
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
                      Constants.ElevatorConstants.kElevatorMinOutput,
                      Constants.ElevatorConstants.kElevatorMaxOutput),
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
                      Constants.ElevatorConstants.kElevatorMinOutput,
                      Constants.ElevatorConstants.kElevatorMaxOutput),
                   0.0));
                
        }

        m_pidController.setReference(setpointState.position, ControlType.kPosition,ClosedLoopSlot.kSlot0,
        ff.calculate(setpointState.position * 2 * Math.PI, setpointState.velocity * 2 * Math.PI ));
        //m_AbsPidController.

        //SmartDashboard.putNumber("ff", ff.calculate(setpointState.position, setpointState.velocity));

    }




    private void updateMotionProfile() {
        m_timer.reset();
      }
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

    public boolean raiseArmAbs(elevatorPositions position){
        if (((ElevatorEncoder.getPosition() < ArmConstants.kMinHeightAbs) && (position == elevatorPositions.L1)) ||
            ((ElevatorEncoder.getPosition() > ArmConstants.kMaxHeightAbs) && (position == elevatorPositions.L4))) {
            //m_armRight.set(0);
            //return true;
        }
        
        double ref = mapAbs.get(position);
        currentGoal = ref;
        updateMotionProfile();


       
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

 public boolean currentPos(){

if (ElevatorEncoder.getPosition() == currentGoal) {
return true;
} else {
    return false;
}
    

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