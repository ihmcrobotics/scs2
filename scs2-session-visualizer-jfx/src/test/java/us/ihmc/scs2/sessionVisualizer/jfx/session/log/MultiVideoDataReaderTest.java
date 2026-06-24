package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import logger_msgs.Camera;
import org.junit.jupiter.api.Test;

public class MultiVideoDataReaderTest
{
   @Test
   void recognizesMagewellCameraTypes()
   {
      Camera camera = new Camera();
      camera.setType("Magewell");
      assertTrue(MultiVideoDataReader.isMagewellCamera(camera));
      assertFalse(MultiVideoDataReader.isBlackMagicCamera(camera));

      camera.setType("CAPTURE_CARD_MAGEWELL");
      assertTrue(MultiVideoDataReader.isMagewellCamera(camera));
   }

   @Test
   void recognizesBlackMagicCameraTypes()
   {
      Camera camera = new Camera();
      camera.setType("Capture Card");
      assertTrue(MultiVideoDataReader.isBlackMagicCamera(camera));
      assertFalse(MultiVideoDataReader.isMagewellCamera(camera));

      camera.setType("CAPTURE_CARD");
      assertTrue(MultiVideoDataReader.isBlackMagicCamera(camera));
   }
}
