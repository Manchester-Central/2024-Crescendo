package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldPose2024;

public class LiftClimbAndPull extends Command {
    private Lift m_lift;
    private liftState state = liftState.visionMove;
    private BaseSwerveDrive m_swerveDrive;
    private Pose2d m_closestChain;
    private Pose2d m_startClimbPose;
    private Vision m_vision;
    private Launcher m_launcher;

    private enum liftState{
        moveIntoPosition, visionMove, moveForwards, pullDown
    }

    public LiftClimbAndPull (Lift lift, BaseSwerveDrive swerveDrive, Vision vision, Launcher launcher){
        m_lift = lift;
        m_swerveDrive = swerveDrive;
        m_vision = vision;
        m_launcher = launcher;
        addRequirements(lift, swerveDrive, launcher);
    };

    @Override
    public void initialize() {
    //   m_lift.moveToHeight(Constants.LiftConstants.StartClimbHeight);
        List<Pose2d> Stages = Arrays.asList(new Pose2d[]{
            FieldPose2024.StageSource.getCurrentAlliancePose(),
            FieldPose2024.StageAmp.getCurrentAlliancePose(),
            FieldPose2024.StageFar.getCurrentAlliancePose()});
        Pose2d closestChain = m_swerveDrive.getPose().nearest(Stages);
        m_closestChain = closestChain;
        double x = Math.cos(closestChain.getRotation().getRadians());
        double y = Math.sin(closestChain.getRotation().getRadians());
        Translation2d pose = closestChain.getTranslation().plus(new Translation2d (x, y));
        m_startClimbPose = new Pose2d(pose, closestChain.getRotation());
        m_swerveDrive.setTarget(new Pose2d(pose, closestChain.getRotation()));
    }

  @Override
  public void execute() {
    System.out.println(state);
    if (state == liftState.moveIntoPosition) {
        if(m_swerveDrive.atTarget(/* 1.5 */) && m_lift.atTargetHeight(LiftConstants.StartClimbHeight)) {
            state = liftState.visionMove;
            m_swerveDrive.stop();
        } else {
            m_swerveDrive.moveToTarget(0.40);
            m_lift.moveToHeight(LiftConstants.StartClimbHeight);
        }
    } else if ( state == liftState.visionMove) {

        Pose2d visionPose = m_vision.getPose();
        Translation2d translationErrorMeters = visionPose.getTranslation().minus(m_startClimbPose.getTranslation());
        Rotation2d rotationError = visionPose.getRotation().minus(m_startClimbPose.getRotation());
        m_lift.moveToHeight(LiftConstants.StartClimbHeight);

        

        if(translationErrorMeters.getNorm() < 0.05 && rotationError.getDegrees() < 2 && m_lift.atTargetHeight(LiftConstants.StartClimbHeight)){
            state = liftState.moveForwards;
            

            double x = -0.6 * Math.cos(m_swerveDrive.getPose().getRotation().getRadians());
            double y = -0.6 * Math.sin(m_swerveDrive.getPose().getRotation().getRadians());
            Translation2d currntPose = m_swerveDrive.getPose().getTranslation();
            currntPose = currntPose.plus(new Translation2d(x, y));

            m_swerveDrive.setTarget(new Pose2d(currntPose, m_swerveDrive.getPose().getRotation()));
            
        } else { 
            var robotPose = m_swerveDrive.getPose();
            m_swerveDrive.setTarget(new Pose2d(translationErrorMeters.plus(robotPose.getTranslation()), rotationError.plus(robotPose.getRotation())));
            m_swerveDrive.moveToTarget(0.15);
        }
    }else if(state == liftState.moveForwards) {

        if (m_swerveDrive.atTarget()) {
            state = liftState.pullDown;
            m_lift.moveToHeight(LiftConstants.MinHeightMeters);
            m_swerveDrive.stop();
        } else {
            m_swerveDrive.moveToTarget(0.15);
        }
    }else if(state == liftState.pullDown){
        // no work(lift do on own )- Jojo :D
    }

  }

  @Override
  public boolean isFinished () {
    return m_lift.atTargetHeight(LiftConstants.MinHeightMeters) && state == liftState.pullDown;
  }
    
  @Override
  public void end(boolean interrupted) {
    state = liftState.visionMove;
  }
}
