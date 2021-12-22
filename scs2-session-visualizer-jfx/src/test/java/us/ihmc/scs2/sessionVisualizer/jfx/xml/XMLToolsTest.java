package us.ihmc.scs2.sessionVisualizer.jfx.xml;

import java.io.IOException;

import javax.xml.bind.JAXBException;

import org.junit.jupiter.api.Test;

public class XMLToolsTest
{

   @Test
   public void test() throws JAXBException, IOException
   {
      XMLTools.loadYoGraphicListDefinition(getClass().getClassLoader().getResourceAsStream("yoGraphic/AtlasDefaultWalking.scs2.yoGraphic"));
   }
}
