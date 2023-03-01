package us.ihmc.scs2.definition.yoComposite;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;

public class YoColorRGBADoubleDefinitionTest
{
   @Test
   public void testToStringAndParse()
   {
      Random random = new Random(23427);
      for (int i = 0; i < 1000; i++)
      {
         YoColorRGBADoubleDefinition original = DefinitionRandomTools.nextYoColorRGBADoubleDefinition(random);
         YoColorRGBADoubleDefinition parsed = YoColorRGBADoubleDefinition.parse(original.toString());

         assertEquals(original, parsed);
      }
   }
}
