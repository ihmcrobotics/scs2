package us.ihmc.scs2.examples.urdf;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

import javax.xml.bind.JAXBException;
import java.io.InputStream;
import java.util.Collections;

public class SimpleCrossFourBarRobotTest
{
   @Test
   public void testLoadingURDF() throws JAXBException
   {
      InputStream is = getClass().getClassLoader().getResourceAsStream("urdf/SimpleCrossFourBarRobot.urdf");
      URDFModel urdfModel = URDFTools.loadURDFModel(is, Collections.emptyList(), null);
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel);
      robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
   }
}
