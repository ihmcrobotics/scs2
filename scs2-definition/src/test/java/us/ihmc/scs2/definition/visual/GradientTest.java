package us.ihmc.scs2.definition.visual;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.Assertions;
import us.ihmc.commons.RunnableThatThrows;

import java.awt.*;

import static org.junit.jupiter.api.Assertions.*;

public class GradientTest
{
   @Test // timeout = 30000
   public void testCreateGradient()
   {
      Color[] gradient = Gradient.createGradient(Color.BLUE, Color.YELLOW, 5);

      assertEquals(new Color(0, 0, 255), gradient[0], "Color[" + 0 + "] not correct: " + gradient[0]);
      assertEquals(new Color(51, 51, 204), gradient[1], "Color[" + 1 + "] not correct: " + gradient[1]);
      assertEquals(new Color(102, 102, 153), gradient[2], "Color[" + 2 + "] not correct: " + gradient[2]);
      assertEquals(new Color(153, 153, 102), gradient[3], "Color[" + 3 + "] not correct: " + gradient[3]);
      assertEquals(new Color(204, 204, 51), gradient[4], "Color[" + 4 + "] not correct: " + gradient[4]);
   }

   @Test // timeout = 30000
   public void testCreateMultiGradient()
   {
      Color[] gradient = Gradient.createMultiGradient(new Color[] {Color.BLUE, Color.YELLOW}, 5);

      assertEquals(new Color(0, 0, 255), gradient[0], "Color[" + 0 + "] not correct: " + gradient[0]);
      assertEquals(new Color(51, 51, 204), gradient[1], "Color[" + 1 + "] not correct: " + gradient[1]);
      assertEquals(new Color(102, 102, 153), gradient[2], "Color[" + 2 + "] not correct: " + gradient[2]);
      assertEquals(new Color(153, 153, 102), gradient[3], "Color[" + 3 + "] not correct: " + gradient[3]);
      assertEquals(new Color(204, 204, 51), gradient[4], "Color[" + 4 + "] not correct: " + gradient[4]);

      Assertions.assertExceptionThrown(IllegalArgumentException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            Gradient.createMultiGradient(new Color[] {Color.BLUE}, 5);
         }
      });
   }
}
