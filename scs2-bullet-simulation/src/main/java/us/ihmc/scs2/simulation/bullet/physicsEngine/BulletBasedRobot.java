package us.ihmc.scs2.simulation.bullet.physicsEngine;

import java.util.List;
import us.ihmc.scs2.simulation.collision.Collidable;
import us.ihmc.scs2.simulation.collision.CollidableHolder;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.RobotExtension;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BulletBasedRobot extends RobotExtension implements CollidableHolder
{
   private final BulletBasedRobotPhysics robotPhysics;
      
   public BulletBasedRobot(Robot robot, YoRegistry physicsRegistry)
   {
      super(robot, physicsRegistry);
      robotPhysics = new BulletBasedRobotPhysics(this);
   }
   
   public void updateCollidableBoundingBoxes()
   {
      robotPhysics.updateCollidableBoundingBoxes();
   }
   
   public void integrateState(double dt)
   {
      robotPhysics.integrateState(dt);
   }
   
   public void updateSensors()
   {
      for (SimJointBasics joint : getRootBody().childrenSubtreeIterable())
      {
         joint.getAuxialiryData().update(robotPhysics.getPhysicsOutput());
      }
   }
   
   @Override
   public List<Collidable> getCollidables()
   {
      // TODO Auto-generated method stub
      return null;
   }

}
