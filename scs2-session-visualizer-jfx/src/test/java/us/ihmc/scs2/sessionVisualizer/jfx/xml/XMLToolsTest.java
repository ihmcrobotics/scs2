package us.ihmc.scs2.sessionVisualizer.jfx.xml;

import org.junit.jupiter.api.Test;
import us.ihmc.scs2.definition.DefinitionIOTools;

import jakarta.xml.bind.JAXBException;
import java.io.IOException;
import java.io.InputStream;

public class XMLToolsTest
{

   @Test
   public void test() throws JAXBException, IOException
   {
      InputStream inputStream = getClass().getClassLoader().getResourceAsStream("yoGraphic/AtlasDefaultWalking.scs2.yoGraphic");
      DefinitionIOTools.loadYoGraphicListDefinition(inputStream);
   }
}
