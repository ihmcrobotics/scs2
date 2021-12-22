package us.ihmc.scs2.examples.simulations;

import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class CylinderRobotSimulation extends RobotDefinition
{
   public CylinderRobotSimulation()
   {
      super("Cylinder");

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition(getRootJointName());
      RigidBodyDefinition cylinder = createCylinderRigidBody();
      cylinder.setMass(10.0);
      cylinder.getMomentOfInertia().setToDiagonal(0.1, 0.1, 0.1);

      setRootBodyDefinition(elevator);
      elevator.addChildJoint(floatingJoint);
      floatingJoint.setSuccessor(cylinder);
   }

   public String getRootJointName()
   {
      return "rootJoint";
   }

   private final RigidBodyDefinition createCylinderRigidBody()
   {
      RigidBodyDefinition ball = new RigidBodyDefinition("Cylinder");
      GeometryDefinition geometryDefinition = new Cylinder3DDefinition(0.8, 0.25);
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Red());
      ball.addVisualDefinition(new VisualDefinition(geometryDefinition, materialDefinition));
      ball.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryDefinition));
      return ball;
   }
}
