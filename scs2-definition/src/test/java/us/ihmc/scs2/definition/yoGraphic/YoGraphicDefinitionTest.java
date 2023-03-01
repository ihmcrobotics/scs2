package us.ihmc.scs2.definition.yoGraphic;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.definition.DefinitionRandomTools;
import us.ihmc.scs2.definition.yoComposite.YoTuple2DDefinition;

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

         String listStringValue = YoGraphicDefinition.listToString(original, Object::toString);
         List<YoTuple2DDefinition> parsed = YoGraphicDefinition.parseList(listStringValue, YoTuple2DDefinition::parse);

         assertEquals(original, parsed);
      }
   }

   @Test
   public void testToStringAndParse() throws Exception
   {
      Random random = new Random(5456);
      YoGraphicGroupDefinition originalRoot = DefinitionRandomTools.nextYoGraphicGroupDefinition(random, 5);
      List<YoGraphicDefinition> allOriginalDefinitions = collectAllDefinitions(originalRoot);

      List<YoGraphicDefinition> allCopyDefinitions = new ArrayList<>();

      for (int i = 0; i < allOriginalDefinitions.size(); i++)
      {
         allCopyDefinitions.add(allOriginalDefinitions.get(i).getClass().getDeclaredConstructor().newInstance());
      }

      List<Map<String, String>> allFieldValueStringMaps = new ArrayList<>();

      for (int i = 0; i < allOriginalDefinitions.size(); i++)
      {
         allFieldValueStringMaps.add(allOriginalDefinitions.get(i).createFieldValueStringMap());
      }

      for (int i = 0; i < allCopyDefinitions.size(); i++)
      {
         allCopyDefinitions.get(i).parseFieldValueStringMap(allFieldValueStringMaps.get(i));
      }

      for (int i = 0; i < allOriginalDefinitions.size(); i++)
      {
         assertEquals(allOriginalDefinitions.get(i), allCopyDefinitions.get(i));
      }
   }

   private static List<YoGraphicDefinition> collectAllDefinitions(YoGraphicDefinition start)
   {
      return collectAllDefinitions(start, new ArrayList<>());
   }

   private static List<YoGraphicDefinition> collectAllDefinitions(YoGraphicDefinition start, List<YoGraphicDefinition> definitionsToPack)
   {
      definitionsToPack.add(start);

      if (start instanceof YoGraphicGroupDefinition group)
      {
         List<YoGraphicDefinition> children = group.getChildren();
         if (children != null)
         {
            for (int i = 0; i < children.size(); i++)
            {
               collectAllDefinitions(children.get(i), definitionsToPack);
            }
         }
      }
      return definitionsToPack;
   }
}
