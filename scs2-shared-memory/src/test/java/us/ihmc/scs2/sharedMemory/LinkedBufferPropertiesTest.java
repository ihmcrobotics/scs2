package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;

public class LinkedBufferPropertiesTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testSequence()
   {
      Random random = new Random(68935);

      {
         YoBufferProperties sourceProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         LinkedBufferProperties linkedBufferProperties = new LinkedBufferProperties(sourceProperties);

         for (int i = 0; i < ITERATIONS; i++)
         {
            assertNull(linkedBufferProperties.peekCurrentBufferProperties());
            assertNull(linkedBufferProperties.pollCurrentBufferProperties());

            sourceProperties.set(SharedMemoryRandomTools.nextYoBufferProperties(random));
            linkedBufferProperties.prepareForPull();

            assertEquals(sourceProperties, linkedBufferProperties.peekCurrentBufferProperties());
            assertEquals(sourceProperties, linkedBufferProperties.peekCurrentBufferProperties());
            assertEquals(sourceProperties, linkedBufferProperties.pollCurrentBufferProperties());
            assertNull(linkedBufferProperties.peekCurrentBufferProperties());
            assertNull(linkedBufferProperties.pollCurrentBufferProperties());

            sourceProperties.set(SharedMemoryRandomTools.nextYoBufferProperties(random));
            YoBufferPropertiesReadOnly expectedProperties = sourceProperties.copy();
            linkedBufferProperties.prepareForPull();

            assertEquals(expectedProperties, linkedBufferProperties.peekCurrentBufferProperties());
            sourceProperties.set(SharedMemoryRandomTools.nextYoBufferProperties(random));
            assertEquals(expectedProperties, linkedBufferProperties.peekCurrentBufferProperties());
            sourceProperties.set(SharedMemoryRandomTools.nextYoBufferProperties(random));
            assertEquals(expectedProperties, linkedBufferProperties.pollCurrentBufferProperties());
         }
      }
   }
}
