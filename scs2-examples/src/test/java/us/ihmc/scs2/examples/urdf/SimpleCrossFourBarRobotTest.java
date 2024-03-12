package us.ihmc.scs2.examples.urdf;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.URDFTools.URDFParserProperties;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

import javax.xml.bind.JAXBException;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.Objects;

import static org.junit.jupiter.api.Assertions.*;

public class SimpleCrossFourBarRobotTest
{
   @Test
   public void testLoadingURDF() throws JAXBException, IOException
   {
      InputStream inputStream = Objects.requireNonNull(getClass().getClassLoader().getResourceAsStream("urdf/SimpleCrossFourBarRobot.urdf"));

      // File is 3.3 KiB
      ByteBuffer inputBytesToCompare = ByteBuffer.allocate(Conversions.kibibytesToBytes(5));
      inputBytesToCompare.put(inputStream.readAllBytes());
      inputBytesToCompare.flip();

      inputStream.close();

      // Open it again
      inputStream = getClass().getClassLoader().getResourceAsStream("urdf/SimpleCrossFourBarRobot.urdf");

      URDFParserProperties parserProperties = new URDFParserProperties();
      parserProperties.setRootJointFactory(null);
      parserProperties.setAutoGenerateCollisionName(false);
      parserProperties.setAutoGenerateVisualName(false);
      parserProperties.setTransformToZUp(false);
      URDFModel originalURDFModel = URDFTools.loadURDFModel(Collections.singletonList(inputStream), Collections.emptyList(), null, parserProperties);
      RobotDefinition originalRobotDefinition = URDFTools.toRobotDefinition(originalURDFModel, parserProperties);
      originalRobotDefinition.newInstance(ReferenceFrame.getWorldFrame());

      ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();

      URDFModel reassembledURDFModel = URDFTools.toURDFModel(originalRobotDefinition);
      URDFTools.saveURDFModel(byteArrayOutputStream, reassembledURDFModel);

      //      System.out.println(new String(byteArrayOutputStream.toByteArray()));

      URDFModel copyURDFModel = URDFTools.loadURDFModel(Collections.singletonList(new ByteArrayInputStream(byteArrayOutputStream.toByteArray())),
                                                        Collections.emptyList(),
                                                        null,
                                                        parserProperties);
      RobotDefinition copyRobotDefinition = URDFTools.toRobotDefinition(copyURDFModel, parserProperties);
      copyRobotDefinition.newInstance(ReferenceFrame.getWorldFrame());

      assertEquals(originalRobotDefinition, copyRobotDefinition);
   }
}
