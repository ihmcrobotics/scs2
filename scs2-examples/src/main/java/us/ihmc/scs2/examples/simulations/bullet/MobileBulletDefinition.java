package us.ihmc.scs2.examples.simulations.bullet;

import java.util.Random;
import java.util.stream.Stream;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.controller.implementations.ControllerCollectionDefinition;
import us.ihmc.scs2.definition.controller.implementations.OneDoFJointDampingControllerDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;


public class MobileBulletDefinition extends RobotDefinition
{
   private static final String MOBILE = "mobile";

   private static final double L1 = 0.3, M1 = 0.1, R1 = 0.01, Ixx1 = 0.01, Iyy1 = 0.01, Izz1 = 0.01;
   private static final double L2 = 0.12, M2 = 0.05, R2 = 0.005, Ixx2 = 0.01, Iyy2 = 0.01, Izz2 = 0.01;
   private static final double L3 = 0.08, M3 = 0.03, R3 = 0.001, Ixx3 = 0.01, Iyy3 = 0.01, Izz3 = 0.01;;
   private static final double TOY_L = 0.02, TOY_W = 0.04, TOY_H = 0.03, TOY_R = 0.02;

   private static final double DAMP1 = 0.06, DAMP2 = 0.006, DAMP3 = 0.003;

   private final OneDoFJointDampingControllerDefinition jointLvl1DampingControllerDefinition = new OneDoFJointDampingControllerDefinition();
   private final OneDoFJointDampingControllerDefinition jointLvl2DampingControllerDefinition = new OneDoFJointDampingControllerDefinition();
   private final OneDoFJointDampingControllerDefinition jointLvl3DampingControllerDefinition = new OneDoFJointDampingControllerDefinition();

   public MobileBulletDefinition()
   {
      super(MOBILE);

      jointLvl1DampingControllerDefinition.setControllerName("DampLevel1").createDampingVariable("damp1", DAMP1);
      jointLvl2DampingControllerDefinition.setControllerName("DampLevel2").createDampingVariable("damp2", DAMP2);
      jointLvl3DampingControllerDefinition.setControllerName("DampLevel3").createDampingVariable("damp3", DAMP3);

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      OneDoFJointDefinition[] jointsLevel1 = createGimbal("jointLvl1", elevator, new Vector3D(0.0, 0.0, 1.0));
      RigidBodyDefinition crossBarLvl1 = createCrossBar("crossBarLvl1", jointsLevel1[2], M1, L1, R1, Ixx1, Iyy1, Izz1);
      jointLvl1DampingControllerDefinition.addJointsToControl(Stream.of(jointsLevel1).map(JointDefinition::getName).toArray(String[]::new));

      
      RigidBodyDefinition crossBarLvl2;

      for (int i = 0; i < 4; i++)
      {
         double xOffset = 0.0;
         double yOffset = 0.0;

         if (i == 0)
            xOffset = L1;
         else if (i == 1)
            xOffset = -L1;
         else if (i == 2)
            yOffset = L1;
         else // i == 3
            yOffset = -L1;
         
         OneDoFJointDefinition jointsLevel2[] = createGimbal("jointLvl2_" + i, crossBarLvl1, new Vector3D(xOffset, yOffset, -L1 / 2.0));
         crossBarLvl2 = createCrossBar("crossBarLvl2_" + i, jointsLevel2[2], M2, L2, R2, Ixx2, Iyy2, Izz2);
         jointLvl2DampingControllerDefinition.addJointsToControl(Stream.of(jointsLevel2).map(JointDefinition::getName).toArray(String[]::new));

         for (int j = 0; j < 4; j++)
         {
            xOffset = 0.0;
            yOffset = 0.0;

            if (j == 0)
               xOffset = L2;
            else if (j == 1)
               xOffset = -L2;
            else if (j == 2)
               yOffset = L2;
            else // j == 3
               yOffset = -L2;

            OneDoFJointDefinition jointsLevel3[] = createGimbal("jointLvl3_" + i + "" + j, crossBarLvl2, new Vector3D(xOffset, yOffset, -L2 / 2.0));
            createRandomShape("toy_" + i + "" + j, jointsLevel3[2]);
            jointLvl3DampingControllerDefinition.addJointsToControl(Stream.of(jointsLevel3).map(JointDefinition::getName).toArray(String[]::new));
         }
      }

      addControllerDefinition(new ControllerCollectionDefinition().setControllerName("mobileController").addControllerOutputReset()
                                                                  .addControllerDefinitions(jointLvl1DampingControllerDefinition,
                                                                                            jointLvl2DampingControllerDefinition,
                                                                                            jointLvl3DampingControllerDefinition));
   }

   private OneDoFJointDefinition[] createGimbal(String name, RigidBodyDefinition predecessor, Tuple3DReadOnly jointOffset)
   {
      RigidBodyDefinition jointXBody = createNullBody(name + "XBody");
      RigidBodyDefinition jointYBody = createNullBody(name + "YBody");

      RevoluteJointDefinition jointX = new RevoluteJointDefinition(name + "X");
      RevoluteJointDefinition jointY = new RevoluteJointDefinition(name + "Y");
      RevoluteJointDefinition jointZ = new RevoluteJointDefinition(name + "Z");

      jointX.getTransformToParent().getTranslation().set(jointOffset);
      jointX.getAxis().set(Axis3D.X);
      jointY.getAxis().set(Axis3D.Y);
      jointZ.getAxis().set(Axis3D.Z);

      predecessor.getChildrenJoints().add(jointX);
      jointX.setSuccessor(jointXBody);
      jointXBody.getChildrenJoints().add(jointY);
      jointY.setSuccessor(jointYBody);
      jointYBody.getChildrenJoints().add(jointZ);

      Random random = new Random();
      jointX.setInitialJointState(new OneDoFJointState(EuclidCoreRandomTools.nextDouble(random, 0.25), EuclidCoreRandomTools.nextDouble(random, 0.5)));
      jointY.setInitialJointState(new OneDoFJointState(EuclidCoreRandomTools.nextDouble(random, 0.25), EuclidCoreRandomTools.nextDouble(random, 0.5)));
      jointZ.setInitialJointState(new OneDoFJointState(EuclidCoreRandomTools.nextDouble(random, Math.PI), EuclidCoreRandomTools.nextDouble(random, 2.0)));

      return new OneDoFJointDefinition[] {jointX, jointY, jointZ};
   }

   private RigidBodyDefinition createNullBody(String name)
   {
      RigidBodyDefinition nullBody = new RigidBodyDefinition(name);
      nullBody.setMass(1.0e-12);
      nullBody.getMomentOfInertia().setToDiagonal(1.0e-12, 1.0e-12, 1.0e-12);
      return nullBody;
   }

   /**
    * Creates a cross bar link from the given parameters.
    */
   private RigidBodyDefinition createCrossBar(String name, JointDefinition parentJoint, double mass, double length, double radius, double Ixx, double Iyy,
                                              double Izz)
   {
      RigidBodyDefinition crossBar = new RigidBodyDefinition(name);
      crossBar.setMass(mass);
      crossBar.getMomentOfInertia().setToDiagonal(Ixx, Iyy, Izz);
      crossBar.getInertiaPose().getTranslation().set(0.0, 0.0, -length / 2.0);
      parentJoint.setSuccessor(crossBar);

      MaterialDefinition redMaterial = new MaterialDefinition(ColorDefinitions.Red());
      MaterialDefinition blackMaterial = new MaterialDefinition(ColorDefinitions.Black());
      GeometryDefinition sphere1 = new Sphere3DDefinition(R1);
      GeometryDefinition sphere2 = new Sphere3DDefinition(radius);
      GeometryDefinition cylinder1 = new Cylinder3DDefinition(0.5 * length, radius);
      GeometryDefinition cylinder2 = new Cylinder3DDefinition(2.0 * length, radius);

      RigidBodyTransform verticalBarPose = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, -0.25 * length));
      RigidBodyTransform crossBarCenter1 = new RigidBodyTransform(new AxisAngle(Axis3D.X, 0.5 * Math.PI), new Vector3D(0.0, 0.0, -0.5 * length));
      RigidBodyTransform crossBarCenter2 = new RigidBodyTransform(new AxisAngle(Axis3D.Y, 0.5 * Math.PI), new Vector3D(0.0, 0.0, -0.5 * length));
      RigidBodyTransform crossBarTip1 = new RigidBodyTransform(new AxisAngle(), new Vector3D(length, 0.0, -0.5 * length));
      RigidBodyTransform crossBarTip2 = new RigidBodyTransform(new AxisAngle(Axis3D.Z, -0.5 * Math.PI), new Vector3D());
      crossBarTip2.multiply(crossBarTip1);
      RigidBodyTransform crossBarTip3 = new RigidBodyTransform(new AxisAngle(Axis3D.Z, -0.5 * Math.PI), new Vector3D());
      crossBarTip3.multiply(crossBarTip2);
      RigidBodyTransform crossBarTip4 = new RigidBodyTransform(new AxisAngle(Axis3D.Z, -0.5 * Math.PI), new Vector3D());
      crossBarTip4.multiply(crossBarTip3);

      crossBar.addVisualDefinition(new VisualDefinition(sphere1, redMaterial));
      crossBar.addVisualDefinition(new VisualDefinition(verticalBarPose, cylinder1, blackMaterial));
      crossBar.addVisualDefinition(new VisualDefinition(crossBarCenter1, cylinder2, blackMaterial));
      crossBar.addVisualDefinition(new VisualDefinition(crossBarCenter2, cylinder2, blackMaterial));
      crossBar.addVisualDefinition(new VisualDefinition(crossBarTip1, sphere2, redMaterial));
      crossBar.addVisualDefinition(new VisualDefinition(crossBarTip2, sphere2, redMaterial));
      crossBar.addVisualDefinition(new VisualDefinition(crossBarTip3, sphere2, redMaterial));
      crossBar.addVisualDefinition(new VisualDefinition(crossBarTip4, sphere2, redMaterial));
      
      CollisionShapeDefinition collisionShapeVericalBarPose = new CollisionShapeDefinition(verticalBarPose, cylinder1);
      crossBar.addCollisionShapeDefinition(collisionShapeVericalBarPose);
      CollisionShapeDefinition collisionShapeCrossBarCenter1 = new CollisionShapeDefinition(crossBarCenter1, cylinder2);
      crossBar.addCollisionShapeDefinition(collisionShapeCrossBarCenter1);
      CollisionShapeDefinition collisionShapeCrossBarCenter2 = new CollisionShapeDefinition(crossBarCenter2, cylinder2);
      crossBar.addCollisionShapeDefinition(collisionShapeCrossBarCenter2);
      CollisionShapeDefinition collisionShapeCrossBarTip1 = new CollisionShapeDefinition(crossBarTip1, sphere2);
      crossBar.addCollisionShapeDefinition(collisionShapeCrossBarTip1);
      CollisionShapeDefinition collisionShapeCrossBarTip2 = new CollisionShapeDefinition(crossBarTip2, sphere2);
      crossBar.addCollisionShapeDefinition(collisionShapeCrossBarTip2);
      CollisionShapeDefinition collisionShapeCrossBarTip3 = new CollisionShapeDefinition(crossBarTip3, sphere2);
      crossBar.addCollisionShapeDefinition(collisionShapeCrossBarTip3);
      CollisionShapeDefinition collisionShapeCrossBarTip4 = new CollisionShapeDefinition(crossBarTip4, sphere2);
      crossBar.addCollisionShapeDefinition(collisionShapeCrossBarTip4);
      return crossBar;
   }

   /**
    * Generates a random link shape with a thin cylinder attached to represent a string. The toys are
    * generated from one of 9 colors and 7 shapes.
    */
   private RigidBodyDefinition createRandomShape(String name, JointDefinition parentJoint)
   {
      double stringLength = L3 * (1.0 + 2.0 * Math.random());
      RigidBodyDefinition toyRigidbody = new RigidBodyDefinition(name);
      toyRigidbody.setMass(M3);
      toyRigidbody.getMomentOfInertia().setToDiagonal(Ixx3, Iyy3, Izz3);
      toyRigidbody.getInertiaPose().getTranslation().set(0.0, 0.0, -stringLength);
      parentJoint.setSuccessor(toyRigidbody);

      RigidBodyTransform barVisualPose = new RigidBodyTransform();
      barVisualPose.getTranslation().setZ(-stringLength / 2.0);
      GeometryDefinition barGeometryDefinition = new Cylinder3DDefinition(stringLength, R3);
      toyRigidbody.addVisualDefinition(new VisualDefinition(barVisualPose, barGeometryDefinition, new MaterialDefinition(ColorDefinitions.Black())));

      int toySelection = (int) (Math.random() * 5.0);
      GeometryDefinition toyGeometryDefinition;

      switch (toySelection)
      {
         case 0:
            toyGeometryDefinition = new Sphere3DDefinition(TOY_R);
            break;
         case 1:
            toyGeometryDefinition = new Cylinder3DDefinition(TOY_H, TOY_R);
            break;
         case 2:
            toyGeometryDefinition = new Box3DDefinition(TOY_L, TOY_W, TOY_H);
            break;
         case 3:
            toyGeometryDefinition = new Cone3DDefinition(TOY_H, TOY_R);
            break;
         default:
            toyGeometryDefinition = new Capsule3DDefinition(TOY_L, TOY_R);
            break;
      }

      RigidBodyTransform toyVisualPose = new RigidBodyTransform();
      toyVisualPose.getTranslation().setZ(-stringLength);
      MaterialDefinition toyMaterialDefinition = new MaterialDefinition(ColorDefinition.rgb(new Random().nextInt()));
      toyRigidbody.addVisualDefinition(new VisualDefinition(toyVisualPose, toyGeometryDefinition, toyMaterialDefinition));
      
      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(toyVisualPose, toyGeometryDefinition);
      toyRigidbody.addCollisionShapeDefinition(collisionShapeDefinition);

      return toyRigidbody;
   }
}
