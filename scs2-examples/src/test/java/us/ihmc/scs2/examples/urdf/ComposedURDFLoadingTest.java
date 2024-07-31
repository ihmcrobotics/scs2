package us.ihmc.scs2.examples.urdf;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

import javax.xml.bind.JAXBException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Class that tests the ability to load an URDF that is split up in multiple files, this may happen because the urdf
 * is rather large or has parts that are often replaced.
 */
public class ComposedURDFLoadingTest
{
   @Test
   public void loadURDFFromOneFiles() throws JAXBException
   {  // Ensure loading a single urdf file works
      String[] urdfFiles = {"urdf/torsoAndLimb.urdf"};
      URDFModel urdfModel = getURDFModel(urdfFiles);

      URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, parserProperties);
      // We expect three because a rootJointDefinition joint is added, and the revolute_twins joint's get combined into one joint, and we don't keep duplicates
      assertEquals(3, robotDefinition.getAllJoints().size());
   }

   @Test
   public void loadURDFFromTwoFilesWithDuplicates() throws JAXBException
   {
      String[] urdfFiles = {"urdf/limbA.urdf", "urdf/torso.urdf"};
      URDFModel urdfModel = getURDFModel(urdfFiles);

      URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, parserProperties);
      // We expect three because a rootJointDefinition joint is added, and the revolute_twins joint's get combined into one joint, and we don't keep duplicates
      assertEquals(3, robotDefinition.getAllJoints().size());
      // The root model is the one that ends up as the final parent
      assertEquals("rootModel", robotDefinition.getName());
   }

   @Test
   public void loadURDFFromThreeFilesWithDuplicates() throws JAXBException
   {
      String[] urdfFiles = {"urdf/limbA.urdf", "urdf/torso.urdf", "urdf/limbB.urdf"};
      URDFModel urdfModel = getURDFModel(urdfFiles);

      URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, parserProperties);
      // We expect three because a rootJointDefinition joint is added, and the revolute_twins joint's get combined into one joint, and we don't keep duplicates
      assertEquals(3, robotDefinition.getAllJoints().size());
      // The root model is the one that ends up as the final parent
      assertEquals("rootModel", robotDefinition.getName());
   }

   @Test
   public void loadURDFWithCorrectName() throws JAXBException
   {
      // A joint here has a child link in the other file, making it the parent
      String[] urdfFiles = {"urdf/torsoParent.urdf", "urdf/torsoChild.urdf"};
      URDFModel urdfModel = getURDFModel(urdfFiles);

      URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, parserProperties);
      // We expect three because a rootJointDefinition joint is added, and the revolute_twins joint's get combined into one joint, and we don't keep duplicates
      assertEquals(3, robotDefinition.getAllJoints().size());
      // The root model is the one that ends up as the final parent
      assertEquals("rootModelParent", robotDefinition.getName());
   }

   private static URDFModel getURDFModel(String[] urdfFilenames) throws JAXBException
   {
      Collection<InputStream> inputStreams = new ArrayList<>();

      for (String urdfFile : urdfFilenames)
      {
         inputStreams.add(ComposedURDFLoadingTest.class.getClassLoader().getResourceAsStream(urdfFile));
      }

      return URDFTools.loadURDFModel(inputStreams, Collections.emptyList(), SimpleCrossFourBarURDFRobot.class.getClassLoader());
   }
}
