package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

public class StringToolsTest
{

   @Test
   public void testCommonSubString()
   {
      assertNull(StringTools.commonSubString((String[]) null));
      assertNull(StringTools.commonSubString(new String[0]));

      String actual = StringTools.commonSubString("abcd", "abcd");
      String expected = "abcd";
      assertEquals(expected, actual);

      actual = StringTools.commonSubString("abcd_efgh", "abcd+efgh");
      expected = "abcdefgh";
      assertEquals(expected, actual);
      
      {
         String prefix = "q_d_";
         String midfix = "_tchou_";
         String suffix = "_blah";
         String[] sides = {"Righ", "Left"}; // Removed the 't' of "Right" otherwise it gets picked.
         String[] jointNames = {"HipYaw", "HipRoll", "HipPitch", "KneePitch", "AnklePitch", "AnkleRoll"};

         List<String> input = new ArrayList<>();
         for (String side : sides)
         {
            for (String jointName : jointNames)
            {
               input.add(prefix + side + midfix + jointName + suffix);
            }
         }

         actual = StringTools.commonSubString(input.toArray(new String[0]));
         expected = prefix + midfix + suffix;
         assertEquals(expected, actual);
      }
   }
}
