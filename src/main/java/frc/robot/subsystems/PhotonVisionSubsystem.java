package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    PhotonCamera camera;


        public PhotonVisionSubsystem () {
            camera = new PhotonCamera("OV9281");
        }

        public boolean hasTarget()
        {
            var result = camera.getLatestResult();

            SmartDashboard.putBoolean("PhotonVisionUser/hasTarget", result.hasTargets());

            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            SmartDashboard.putNumber("PhotonVisionUser/yaw", yaw);
            SmartDashboard.putNumber("PhotonVisionUser/pitch", pitch);

            return result.hasTargets();
        }



        public boolean goToTarget(){

            return false;


        }

        public void testTarget()
        {
            var result = camera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            SmartDashboard.putNumber("PhotonVisionUser/yaw", yaw);
            SmartDashboard.putNumber("PhotonVisionUser/pitch", pitch);
            //Transform2d pose = target.getCameraToTarget();
            //List<TargetCorner> corners = target.getCorners();
        }

}
