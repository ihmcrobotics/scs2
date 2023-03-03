package us.ihmc.scs2.definition.yoGraphic;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition.YoGraphicFieldsSummary;

public class YoGraphicDefinitionTest
{
   @Test
   public void testListToStringAndParse() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < 1000; i++)
      {
         int size = random.nextInt(20);
         List<YoTuple2DDefinition> original = new ArrayList<>();
         while (original.size() < size)
            original.add(DefinitionRandomTools.nextYoTuple2DDefinition(random));

         String elementLabel = "dsf";
         String listStringValue = YoGraphicDefinition.listToString(original, elementLabel, Object::toString);
         List<YoTuple2DDefinition> parsed = YoGraphicDefinition.parseList(listStringValue, elementLabel, YoTuple2DDefinition::parse);

         assertEquals(original, parsed);
      }

      List<YoTuple2DDefinition> original = null;
      String elementLabel = "dsf";
      String listStringValue = YoGraphicDefinition.listToString(original, elementLabel, Object::toString);
      List<YoTuple2DDefinition> parsed = YoGraphicDefinition.parseList(listStringValue, elementLabel, YoTuple2DDefinition::parse);

      assertEquals(original, parsed);
   }

   @Test
   public void testToStringAndParse() throws Exception
   {
      Random random = new Random(345);

      for (int i = 0; i < 1000; i++)
      {
         YoGraphicDefinition original = DefinitionRandomTools.nextYoGraphicDefinition(random);
         YoGraphicDefinition parsed = YoGraphicDefinition.parse(original.toParsableString());

         assertEquals(original.getClass(), parsed.getClass());
         assertEquals(original.getName(), parsed.getName());
      }
   }

   @Test
   public void testToStringMapAndParseMap() throws Exception
   {
      Random random = new Random(5456);

      for (int i = 0; i < 1000; i++)
      {
         YoGraphicGroupDefinition originalRoot = DefinitionRandomTools.nextYoGraphicGroupDefinition(random, 5);
         List<YoGraphicFieldsSummary> subtreeFieldValueStringMaps = YoGraphicDefinition.exportSubtreeYoGraphicFieldsSummaryList(originalRoot);
         List<YoGraphicGroupDefinition> parsed = YoGraphicGroupDefinition.parseTreeYoGraphicFieldsSummary(subtreeFieldValueStringMaps);

         assertEquals(1, parsed.size());
         assertEquals(originalRoot, parsed.get(0));
      }

      for (int i = 0; i < 1000; i++)
      {
         int numberOfRoots = random.nextInt(5);
         List<YoGraphicGroupDefinition> originalRoots = new ArrayList<>();
         for (int j = 0; j < numberOfRoots; j++)
            originalRoots.add(DefinitionRandomTools.nextYoGraphicGroupDefinition(random, 5));
         List<YoGraphicFieldsSummary> subtreeFieldValueStringMaps = YoGraphicDefinition.exportSubtreeYoGraphicFieldsSummaryList(originalRoots);
         List<YoGraphicGroupDefinition> parsed = YoGraphicGroupDefinition.parseTreeYoGraphicFieldsSummary(subtreeFieldValueStringMaps);
         
         assertEquals(numberOfRoots, parsed.size());
         assertEquals(originalRoots, parsed);
      }
   }
}
