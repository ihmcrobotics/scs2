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

public class MultipleURDFTest
{
   @Test
   public void loadURDFFromTwoDifferentFiles() throws JAXBException
   {
      String[] urdfFiles = {"urdf/torso.urdf", "urdf/arm.urdf"};
      URDFModel urdfModel = getURDFModel(urdfFiles);

      URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel, parserProperties);
      assertEquals(3, robotDefinition.getAllJoints().size());
   }

   private static URDFModel getURDFModel(String[] urdfFilenames) throws JAXBException
   {
      Collection<InputStream> inputStreams = new ArrayList<>();

      for (String urdfFile : urdfFilenames)
      {
         inputStreams.add(MultipleURDFTest.class.getClassLoader().getResourceAsStream(urdfFile));
      }

      return URDFTools.loadURDFModel(inputStreams, Collections.emptyList(), SimpleCrossFourBarURDFRobot.class.getClassLoader());
   }
}
