package frc.robot.commands;

import org.ejml.sparse.csc.mult.MatrixVectorMultWithSemiRing_FSCC;

import com.chaos131.swerve.BaseSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.FieldPose2024;

public class LiftClimbAndPull extends Command {
    private Lift m_lift;
    private liftState state = liftState.moveIntoPosition;
    private BaseSwerveDrive m_swerveDrive;

    private enum liftState{
        moveIntoPosition, raiseLift, moveForwards, pullDown
    }

    public LiftClimbAndPull (Lift lift, BaseSwerveDrive swerveDrive){
        m_lift = lift;
        m_swerveDrive = swerveDrive;
        addRequirements(lift, swerveDrive);
    };

    @Override
    public void initialize() {
    //   m_lift.moveToHeight(Constants.LiftConstants.StartClimbHeight);
        double x = Math.cos(FieldPose2024.StageSource.getCurrentAlliancePose().getRotation().getRadians());
        double y = Math.sin(FieldPose2024.StageSource.getCurrentAlliancePose().getRotation().getRadians());
        Translation2d pose = FieldPose2024.StageSource.getCurrentAlliancePose().getTranslation().plus(new Translation2d (x, y));
        m_swerveDrive.setTarget(new Pose2d(pose, FieldPose2024.StageSource.getCurrentAlliancePose().getRotation()));
    }

  @Override
  public void execute() {
    System.out.println(state);
    if (state == liftState.moveIntoPosition) {
        if(m_swerveDrive.atTarget()) {
            state = liftState.raiseLift;
            m_swerveDrive.stop();
        } else {
            m_swerveDrive.moveToTarget(0.40);
        }
    } else if ( state == liftState.raiseLift) {

       m_lift.moveToHeight(Constants.LiftConstants.StartClimbHeight);

        if(m_lift.atTargetHeight(Constants.LiftConstants.StartClimbHeight)){
            state = liftState.moveForwards;
            

            double x = -0.6 * Math.cos(m_swerveDrive.getPose().getRotation().getRadians());
            double y = -0.6 * Math.sin(m_swerveDrive.getPose().getRotation().getRadians());
            Translation2d currntPose = m_swerveDrive.getPose().getTranslation();
            currntPose = currntPose.plus(new Translation2d(x, y));

            m_swerveDrive.setTarget(new Pose2d(currntPose, m_swerveDrive.getPose().getRotation()));
            
       }
    }else if(state == liftState.moveForwards) {

        if (m_swerveDrive.atTarget()) {
            state = liftState.pullDown;
            m_lift.moveToHeight(Constants.LiftConstants.MinHeightMeters);
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
    return m_lift.atTargetHeight(Constants.LiftConstants.MinHeightMeters) && state == liftState.pullDown;
  }
    
  @Override
  public void end(boolean interrupted) {
    state = liftState.moveIntoPosition;
  }
}
