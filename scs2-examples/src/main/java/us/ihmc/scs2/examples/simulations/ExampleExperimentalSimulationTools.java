package us.ihmc.scs2.examples.simulations;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class ExampleExperimentalSimulationTools
{
   public static RobotDefinition newSphereRobot(String name, double radius, double mass, double radiusOfGyrationPercent, ColorDefinition color, boolean addStripes,
                                         ColorDefinition stripesColor)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);
      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(newSphereRigidBody(name + "RigidBody", radius, mass, radiusOfGyrationPercent, color, addStripes, stripesColor));
      robotDefinition.setRootBodyDefinition(rootBody);
      return robotDefinition;
   }

   public static RigidBodyDefinition newSphereRigidBody(String name, double radius, double mass, double radiusOfGyrationPercent, ColorDefinition color,
                                                        boolean addStripes, ColorDefinition stripesColor)
   {
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name);
      double radiusOfGyration = radiusOfGyrationPercent * radius;
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass, radiusOfGyration, radiusOfGyration, radiusOfGyration));

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addSphere(radius, new MaterialDefinition(color));

      if (addStripes)
      {
         double stripePercent = 0.05;
         factory.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, new MaterialDefinition(stripesColor));
         factory.appendRotation(Math.PI / 2.0, Axis3D.X);
         factory.addArcTorus(0.0, 2.0 * Math.PI, (1.01 - stripePercent) * radius, radius * stripePercent, new MaterialDefinition(stripesColor));
      }

      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      return rigidBody;
   }

   public static RobotDefinition newCylinderRobot(String name, double radius, double height, double mass, double radiusOfGyrationPercent, ColorDefinition color,
                                           boolean addStripes, ColorDefinition stripesColor)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                     radiusOfGyrationPercent * radius,
                                                                                     radiusOfGyrationPercent * radius,
                                                                                     radiusOfGyrationPercent * height));

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addCylinder(height, radius, new MaterialDefinition(color));

      if (addStripes)
      {
         double stripePercent = 0.05;
         factory.appendTranslation(0.0, 0.0, -height * 0.01);
         factory.addBox(2.0 * radius * 1.01, radius * stripePercent, height * 1.02, new MaterialDefinition(stripesColor));
         factory.appendRotation(Math.PI / 2.0, Axis3D.Z);
         factory.addBox(2.0 * radius * 1.01, radius * stripePercent, height * 1.02, new MaterialDefinition(stripesColor));
      }

      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      rootJoint.setSuccessor(rigidBody);
      robotDefinition.setRootBodyDefinition(rootBody);

      return robotDefinition;
   }

   public static RobotDefinition newCapsuleRobot(String name, double radius, double height, double mass, double radiusOfGyrationPercent, ColorDefinition color,
                                          boolean addStripes, ColorDefinition stripesColor)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(name + "RigidBody");
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                     radiusOfGyrationPercent * radius,
                                                                                     radiusOfGyrationPercent * radius,
                                                                                     radiusOfGyrationPercent * height));

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addCapsule(radius, height, new MaterialDefinition(color));

      if (addStripes)
      {
         double stripePercent = 0.05;
         factory.addBox(2.0 * radius * 1.01, radius * stripePercent, height, new MaterialDefinition(stripesColor));
         factory.appendRotation(Math.PI / 2.0, Axis3D.Z);
         factory.addBox(2.0 * radius * 1.01, radius * stripePercent, height, new MaterialDefinition(stripesColor));
      }

      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      rootJoint.setSuccessor(rigidBody);
      robotDefinition.setRootBodyDefinition(rootBody);

      return robotDefinition;
   }

   public static RobotDefinition newBoxRobot(String name, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      return newBoxRobot(name, size.getX(), size.getY(), size.getZ(), mass, radiusOfGyrationPercent, color);
   }

   public static RobotDefinition newBoxRobot(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(newBoxRigidBody(name + "RigidBody", sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, color));
      robotDefinition.setRootBodyDefinition(rootBody);

      return robotDefinition;
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent,
                                                     ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, size, mass, radiusOfGyrationPercent, null, color);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent,
                                                     Vector3DReadOnly offsetFromParent, ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, size.getX(), size.getY(), size.getZ(), mass, radiusOfGyrationPercent, offsetFromParent, color);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, double sizeX, double sizeY, double sizeZ, double mass,
                                                     double radiusOfGyrationPercent, ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, null, color);
   }

   public static RigidBodyDefinition newBoxRigidBody(String rigidBodyName, double sizeX, double sizeY, double sizeZ, double mass,
                                                     double radiusOfGyrationPercent, Vector3DReadOnly offsetFromParentJoint, ColorDefinition color)
   {
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(rigidBodyName);
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                     radiusOfGyrationPercent * sizeX,
                                                                                     radiusOfGyrationPercent * sizeY,
                                                                                     radiusOfGyrationPercent * sizeZ));
      if (offsetFromParentJoint != null)
         rigidBody.setCenterOfMassOffset(offsetFromParentJoint);

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      if (offsetFromParentJoint != null)
         factory.appendTranslation(offsetFromParentJoint);
      factory.addBox(sizeX, sizeY, sizeZ, new MaterialDefinition(color));
      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      return rigidBody;
   }
}
