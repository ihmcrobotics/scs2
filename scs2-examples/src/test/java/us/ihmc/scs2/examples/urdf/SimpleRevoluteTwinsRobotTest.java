package us.ihmc.scs2.examples.urdf;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RevoluteTwinsJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.URDFTools.URDFParserProperties;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

import javax.xml.bind.JAXBException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

public class SimpleRevoluteTwinsRobotTest
{
   @Test
   public void testLoadingURDFTypeA() throws Exception
   {
      URDFModel urdfModel = getURDFModel("urdf/SimpleRevoluteTwinsRobotTypeA.urdf");
      buildRobotDefinitionAndTest(urdfModel);
   }

   @Test
   public void testLoadingURDFTypeB() throws Exception
   {
      URDFModel urdfModel = getURDFModel("urdf/SimpleRevoluteTwinsRobotTypeB.urdf");
      urdfModel.getJoints().stream().filter(j -> j.getName().equals("jointA_jointB")).findFirst().get().setName("revoluteTwins");
      buildRobotDefinitionAndTest(urdfModel);
   }

   @Test
   public void testLoadingURDFTypeC() throws Exception
   {
      URDFModel urdfModel = getURDFModel("urdf/SimpleRevoluteTwinsRobotTypeC.urdf");
      buildRobotDefinitionAndTest(urdfModel);
   }

   private static URDFModel getURDFModel(String urdfFilename) throws JAXBException
   {
      try (InputStream is = SimpleCrossFourBarURDFRobot.class.getClassLoader().getResourceAsStream(urdfFilename))
      {
         return URDFTools.loadURDFModel(is, Collections.emptyList(), SimpleCrossFourBarURDFRobot.class.getClassLoader());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static void buildRobotDefinitionAndTest(URDFModel urdfModel)
   {
      URDFParserProperties parserProperties = new URDFParserProperties();
      parserProperties.setRootJointFactory(null);
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, parserProperties);
      assertAsExpected(robotDefinition);
   }

   /**
    * Asserts that the robot definition is as expected.
    *
    * @param robotDefinition the robot definition to verify.
    */
   private static void assertAsExpected(RobotDefinition robotDefinition)
   {
      RigidBodyDefinition rootBody = robotDefinition.getRootBodyDefinition();
      assertEquals("baseLink", rootBody.getName());
      assertEquals(1, rootBody.getChildrenJoints().size());
      assertEquals(RevoluteTwinsJointDefinition.class, rootBody.getChildrenJoints().get(0).getClass());

      RevoluteTwinsJointDefinition joint = (RevoluteTwinsJointDefinition) rootBody.getChildrenJoints().get(0);
      assertEquals("revoluteTwins", joint.getName());
      assertEquals(0, joint.getActuatedJointIndex());
      EuclidCoreTestTools.assertEquals(Axis3D.Y, joint.getAxis(), 0.0);

      assertEquals("jointA", joint.getJointA().getName());
      assertEquals("jointB", joint.getJointB().getName());

      EuclidCoreTestTools.assertEquals(new YawPitchRollTransformDefinition(new Point3D(0.1, 0, 0.2)), joint.getTransformAToPredecessor(), 0.0);
      EuclidCoreTestTools.assertEquals(new YawPitchRollTransformDefinition(new Point3D(0.0, 0, 0.2)), joint.getTransformBToA(), 0.0);

      RigidBodyDefinition bodyAB = joint.getBodyAB();
      assertEquals("bodyAB", bodyAB.getName());
      assertEquals(0.025, bodyAB.getMass());
      EuclidCoreTestTools.assertEquals(new MomentOfInertiaDefinition(1.0e-8, 1.0e-8, 1.0e-8), bodyAB.getMomentOfInertia(), 0.0);
   }
}
