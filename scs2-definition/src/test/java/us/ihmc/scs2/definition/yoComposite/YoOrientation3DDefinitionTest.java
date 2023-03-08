package us.ihmc.scs2.definition.yoComposite;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;

public class YoOrientation3DDefinitionTest
{
   @Test
   public void testToStringAndParse()
   {
      Random random = new Random(3411);

      for (int i = 0; i < 1000; i++)
      {
         YoOrientation3DDefinition original = DefinitionRandomTools.nextYoOrientation3DDefinition(random);
         YoOrientation3DDefinition parsed = YoOrientation3DDefinition.parse(original.toString());

         assertEquals(original, parsed);
      }
   }
}
