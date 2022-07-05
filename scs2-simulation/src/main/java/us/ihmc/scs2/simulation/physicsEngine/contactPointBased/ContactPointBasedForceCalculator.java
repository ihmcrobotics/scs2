package us.ihmc.scs2.simulation.physicsEngine.contactPointBased;

import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHolder;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParametersReadOnly;
import us.ihmc.scs2.simulation.parameters.YoContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.robot.RobotInterface;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ContactPointBasedForceCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoContactPointBasedContactParameters parameters = new YoContactPointBasedContactParameters("ground", registry);

   private final ReferenceFrame inertialFrame;

   private boolean hasPrintedErrorMessageAboutMultipleCollisions = false; 

   public ContactPointBasedForceCalculator(ReferenceFrame inertialFrame, YoRegistry parentRegistry)
   {
      this.inertialFrame = inertialFrame;
      parameters.set(ContactPointBasedContactParameters.defaultParameters());

      parentRegistry.addChild(registry);
   }

   public void setParameters(ContactPointBasedContactParametersReadOnly parameters)
   {
      this.parameters.set(parameters);
   }

   public void resolveContactForces(List<? extends RobotInterface> robots, CollidableHolder staticCollidableHolder)
   {
      for (RobotInterface robot : robots)
      {
         resolveContactForces(robot, staticCollidableHolder);
      }
   }

   private final Point3D closestPointOnSurface = new Point3D();
   private final Vector3D normalAtClosestPoint = new Vector3D();
   private final Vector3D deltaPositionFromTouchdown = new Vector3D();

   public void resolveContactForces(RobotInterface robot, CollidableHolder staticCollidableHolder)
   {
      for (SimJointBasics joint : robot.getJointsToConsider())
      {
         if (joint.getAuxiliaryData().getGroundContactPoints().isEmpty())
            continue;

         for (GroundContactPoint gcp : joint.getAuxiliaryData().getGroundContactPoints())
         {
            YoFramePoseUsingYawPitchRoll gcpPose = gcp.getPose();

            List<Collidable> activeCollidables = staticCollidableHolder.getCollidables().stream().filter(collidable ->
            {
               if (!collidable.getBoundingBox().isInsideInclusive(gcpPose.getPosition()))
                  return false;
               return collidable.getShape().isPointInside(gcpPose.getPosition());
            }).collect(Collectors.toList());

            if (activeCollidables.isEmpty())
            {
               gcp.getInContact().set(false);
               gcp.getIsSlipping().set(false);
               gcp.getWrench().setToZero();
               continue;
            }

            if (activeCollidables.size() > 1)
            {
               if (!hasPrintedErrorMessageAboutMultipleCollisions)
               {
                  LogTools.error("Cannot handle collision to more than one collidable. (Reporting error only once)");
                  hasPrintedErrorMessageAboutMultipleCollisions = true;
               }
            }

            Collidable collidable = activeCollidables.get(0);

            collidable.getShape().evaluatePoint3DCollision(gcpPose.getPosition(), closestPointOnSurface, normalAtClosestPoint);

            YoFramePose3D gcpTouchdownPose = gcp.getTouchdownPose();

            if (!gcp.getInContact().getValue())
            {
               gcp.getInContact().set(true);
               gcpTouchdownPose.set(gcpPose);
               gcp.getContactNormal().set(normalAtClosestPoint);
            }

            resolveContactForceUsingSurfaceNormal(gcp);
            checkIfSlipping(gcp);
         }
      }
   }

   private final Vector3D linearVelocityWorld = new Vector3D();
   private final Vector3D inPlaneVector1 = new Vector3D();
   private final Vector3D inPlaneVector2 = new Vector3D();
   private final Vector3D forceParallel = new Vector3D();
   private final Vector3D forceNormal = new Vector3D();

   private void resolveContactForceUsingSurfaceNormal(GroundContactPoint gcp)
   {
      YoFramePoseUsingYawPitchRoll gcpPose = gcp.getPose();
      YoFramePose3D gcpTouchdownPose = gcp.getTouchdownPose();
      FrameVector3DReadOnly contactNormal = gcp.getContactNormal();

      deltaPositionFromTouchdown.sub(gcpTouchdownPose.getPosition(), gcpPose.getPosition());

      linearVelocityWorld.set(gcp.getTwist().getLinearPart());
      gcp.getFrame().transformFromThisToDesiredFrame(inertialFrame, linearVelocityWorld);

      inPlaneVector1.set(Axis3D.Y);

      if (Math.abs(inPlaneVector1.dot(contactNormal)) == 1.0) // check if they are parallel, in which case UNIT_X will do.
      {
         inPlaneVector1.set(Axis3D.X);
      }

      inPlaneVector1.cross(inPlaneVector1, contactNormal);
      inPlaneVector1.normalize();

      inPlaneVector2.cross(contactNormal, inPlaneVector1);
      inPlaneVector2.normalize();

      // Spring part
      double xPrime = inPlaneVector1.dot(deltaPositionFromTouchdown);
      double yPrime = inPlaneVector2.dot(deltaPositionFromTouchdown);
      double zPrime = contactNormal.dot(deltaPositionFromTouchdown);

      forceParallel.setAndScale(xPrime, inPlaneVector1);
      forceParallel.scaleAdd(yPrime, inPlaneVector2, forceParallel);
      forceParallel.scale(parameters.getKxy());

      forceNormal.set(contactNormal);

      if (parameters.getStiffeningLength() - zPrime > 0.002)
      {
         forceNormal.scale(parameters.getKz() * zPrime / (parameters.getStiffeningLength() - zPrime));
      }
      else
      {
         forceNormal.scale(parameters.getKz() * zPrime / 0.002);
      }

      // Damping part
      xPrime = inPlaneVector1.dot(linearVelocityWorld);
      yPrime = inPlaneVector2.dot(linearVelocityWorld);
      zPrime = contactNormal.dot(linearVelocityWorld);
      forceParallel.scaleAdd(-parameters.getBxy() * xPrime, inPlaneVector1, forceParallel);
      forceParallel.scaleAdd(-parameters.getBxy() * yPrime, inPlaneVector2, forceParallel);
      forceNormal.scaleAdd(-parameters.getBz() * zPrime, contactNormal, forceNormal);

      double magnitudeOfForceNormal = forceNormal.dot(contactNormal);

      if (magnitudeOfForceNormal < 0.0)
      {
         // If both the ground is pulling the point in rather than pushing it out,
         // and the point is higher than the touchdown point, then set not in contact so that
         // the touchdown point can be reset next tick if still below the ground...
         if (zPrime < 0.0)
         {
            forceParallel.setToZero();
            forceNormal.setToZero();
            gcp.getInContact().set(false);
         }
         else
         {
            //            forceParallel.set(0.0, 0.0, 0.0);
            forceNormal.set(0.0, 0.0, 0.0);
         }
      }

      // Sum the total
      gcp.getWrench().getLinearPart().add(forceParallel, forceNormal);
      inertialFrame.transformFromThisToDesiredFrame(gcp.getFrame(), gcp.getWrench().getLinearPart());
   }

   private final Vector3D forceWorld = new Vector3D();

   private void checkIfSlipping(GroundContactPoint gcp)
   {
      if (!parameters.isSlipEnabled())
      {
         gcp.getIsSlipping().set(false);
         return;
      }

      // Stick-Slip code below
      // Compute the horizontal to vertical force ratio:
      forceWorld.set(gcp.getWrench().getLinearPart());
      gcp.getFrame().transformFromThisToDesiredFrame(gcp.getFrame(), forceWorld);
      FrameVector3DReadOnly contactNormal = gcp.getContactNormal();

      forceNormal.set(contactNormal);
      forceNormal.scale(contactNormal.dot(forceWorld));

      forceParallel.sub(forceWorld, forceNormal);

      double parallelSpringForce = forceParallel.norm();
      double normalSpringForce = forceNormal.norm();

      double ratio = parallelSpringForce / normalSpringForce;

      // It's slipping if it already was and forces are above dynamic ratio.  It starts slipping if above static ratio.
      // But don't slip if inside the ground by more than 1 cm since this probably only occurs when in a wedge and keep sliding
      // perpendicular to the normal into the chasm..
      // +++JEP: 140626: Revisit the chasm thing later. For now take the heightAt check out...
      //    if ((gc.getZ() > heightAt - 0.010) && ((ratio > groundAlphaStick.getDoubleValue()) || ((gc.isSlipping()) && (ratio > groundAlphaSlip.getDoubleValue()))))
      if (ratio > parameters.getAlphaStick() || gcp.getIsSlipping().getValue() && ratio > parameters.getAlphaSlip())
      {
         gcp.getIsSlipping().set(true);
         double parallelSlipForce = parameters.getAlphaSlip() * normalSpringForce;

         double parallelScale = parallelSlipForce / parallelSpringForce;
         if (parallelScale < 1.0)
            forceParallel.scale(parallelScale);

         forceWorld.add(forceNormal, forceParallel);
         gcp.getWrench().getLinearPart().setMatchingFrame(inertialFrame, forceWorld);

         // Move touch-down values along the perp direction to follow the slipping.

         double len = forceParallel.norm();
         if (len > 1e-7)
            forceParallel.scale(1.0 / len);

         YoFramePoseUsingYawPitchRoll gcpPose = gcp.getPose();
         YoFramePose3D gcpTouchdownPose = gcp.getTouchdownPose();
         gcpTouchdownPose.getPosition()
                         .scaleAdd(-0.05 * gcpPose.getPosition().distance(gcpTouchdownPose.getPosition()), forceParallel, gcpTouchdownPose.getPosition());

         // Only update the contact normal when slipping.
         gcp.getContactNormal().set(normalAtClosestPoint);
      }
      else
      {
         gcp.getIsSlipping().set(false);
      }
   }
}
