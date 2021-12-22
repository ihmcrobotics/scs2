package us.ihmc.scs2.sessionVisualizer.jfx.tools;

import java.util.Collection;
import java.util.Iterator;

import org.apache.commons.text.similarity.LongestCommonSubsequence;

public class StringTools
{
   public static String commonSubString(String... strings)
   {
      if (strings == null || strings.length == 0)
         return null;
      if (strings.length == 1)
         return strings[0];

      LongestCommonSubsequence lcs = new LongestCommonSubsequence();
      String result = (String) lcs.longestCommonSubsequence(strings[0], strings[1]);

      for (int i = 2; i < strings.length; i++)
      {
         result = (String) lcs.longestCommonSubsequence(result, strings[i]);
      }

      return result;
   }

   public static String commonSubString(Collection<? extends String> strings)
   {
      if (strings == null || strings.isEmpty())
         return null;

      Iterator<? extends String> iterator = strings.iterator();

      if (strings.size() == 1)
         return iterator.next();

      LongestCommonSubsequence lcs = new LongestCommonSubsequence();
      String result = (String) lcs.longestCommonSubsequence(iterator.next(), iterator.next());

      while (iterator.hasNext())
      {
         result = (String) lcs.longestCommonSubsequence(result, iterator.next());
      }

      return result;
   }
}
