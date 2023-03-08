package us.ihmc.scs2.definition.yoComposite;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;

public class YoTuple3DDefinitionTest
{
   @Test
   public void testToStringAndParse()
   {
      Random random = new Random(3411);

      for (int i = 0; i < 1000; i++)
      {
         YoTuple3DDefinition original = DefinitionRandomTools.nextYoTuple3DDefinition(random);
         YoTuple3DDefinition parsed = YoTuple3DDefinition.parse(original.toString());

         assertEquals(original, parsed);
      }
   }
}
