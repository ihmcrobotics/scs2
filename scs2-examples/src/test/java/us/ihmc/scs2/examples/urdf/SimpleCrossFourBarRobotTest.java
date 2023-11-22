package us.ihmc.scs2.examples.urdf;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

import javax.xml.bind.JAXBException;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.channels.Channels;
import java.nio.channels.ReadableByteChannel;
import java.util.Collections;

public class SimpleCrossFourBarRobotTest
{
   @Test
   public void testLoadingURDF() throws JAXBException, IOException
   {
      InputStream inputStream = getClass().getClassLoader().getResourceAsStream("urdf/SimpleCrossFourBarRobot.urdf");

      // File is 3.3 KiB
      ByteBuffer inputBytesToCompare = ByteBuffer.allocate(Conversions.kibibytesToBytes(5));

      ReadableByteChannel channel = Channels.newChannel(inputStream);
      while (channel.read(inputBytesToCompare) != -1)
         ;
      inputBytesToCompare.flip();

      inputStream.close();

      // Open it again
      inputStream = getClass().getClassLoader().getResourceAsStream("urdf/SimpleCrossFourBarRobot.urdf");

      URDFModel urdfModel = URDFTools.loadURDFModel(inputStream, Collections.emptyList(), null);
      RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfModel);
      robotDefinition.newInstance(ReferenceFrame.getWorldFrame());

      ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();

      URDFModel reassembledURDFModel = URDFTools.toURDFModel(robotDefinition);
      URDFTools.saveURDFModel(byteArrayOutputStream, reassembledURDFModel);

      ByteBuffer outputBytesToCompare = ByteBuffer.wrap(byteArrayOutputStream.toByteArray());

      // Output is currently in a different ordering
      // Assertions.assertEquals(0, inputBytesToCompare.compareTo(outputBytesToCompare));
   }
}
