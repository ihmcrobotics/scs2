package us.ihmc.scs2.definition.visual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;

public class PaintDefinitionTest
{
   @Test
   public void testToStringAndParse()
   {
      Random random = new Random(334455);

      for (int i = 0; i < 1000; i++)
      {
         PaintDefinition original = DefinitionRandomTools.nextPaintDefinition(random);
         PaintDefinition parsed = PaintDefinition.parse(original.toString());

         assertEquals(original, parsed);
      }
   }
}
