package us.ihmc.scs2.session.mcap;

import org.junit.jupiter.api.Test;

import java.util.Random;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

public class MCAPMessageManagerTest
{

   public static final int ITERATIONS = 1000;

   @Test
   public void testRound()
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long rawValue = random.nextLong();
         if (rawValue < 0)
            rawValue = -rawValue;

         long step = TimeUnit.MILLISECONDS.toNanos(random.nextInt(100));

         long roundedValue = MCAPMessageManager.round(rawValue, step);

         if (step <= 0)
         {
            assertEquals(rawValue, roundedValue, "Iteration: " + i + ", rawValue: " + rawValue + ", step: " + step + ", roundedValue: " + roundedValue);
         }
         else
         {
            long remainder = roundedValue % step;
            assertEquals(0, remainder, "Iteration: " + i + ", rawValue: " + rawValue + ", step: " + step + ", roundedValue: " + roundedValue);
         }
      }
   }
}
