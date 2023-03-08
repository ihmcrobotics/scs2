package us.ihmc.scs2.definition.yoComposite;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;

public class YoColorRGBASingleDefinitionTest
{

   @Test
   public void testToStringAndParse()
   {
      Random random = new Random(23427);
      for (int i = 0; i < 1000; i++)
      {
         YoColorRGBASingleDefinition original = DefinitionRandomTools.nextYoColorRGBASingleDefinition(random);
         YoColorRGBASingleDefinition parsed = YoColorRGBASingleDefinition.parse(original.toString());

         assertEquals(original, parsed);
      }
   }

}
