package us.ihmc.scs2.definition.yoGraphic;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;

public class YoListDefinitionTest
{
   @Test
   public void testToStringAndParse()
   {
      Random random = new Random(3411);

      for (int i = 0; i < 1000; i++)
      {
         YoListDefinition original = DefinitionRandomTools.nextYoListDefinition(random);
         YoListDefinition parsed = YoListDefinition.parse(original.toString());

         assertEquals(original, parsed);
      }
   }
}
