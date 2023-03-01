package us.ihmc.scs2.definition.yoComposite;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;

public class YoColorRGBAIntDefinitionTest
{
   @Test
   public void testToStringAndParse()
   {
      Random random = new Random(23427);
      for (int i = 0; i < 1000; i++)
      {
         YoColorRGBAIntDefinition original = DefinitionRandomTools.nextYoColorRGBAIntDefinition(random);
         YoColorRGBAIntDefinition parsed = YoColorRGBAIntDefinition.parse(original.toString());

         assertEquals(original, parsed);
      }
   }
}
