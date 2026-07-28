package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Targets {@link YoVariableBufferList#writeBufferAt(int)}'s type-segregated-buckets + worker-pool path directly.
 * {@code YoRegistryBufferTest.testWriteBuffer} already covers small/random registries, but a bucket-assignment or
 * partition-boundary bug is the kind of thing that only shows up with a large, deliberately mixed-type population -
 * that's what this class builds.
 */
public class YoVariableBufferListTest
{
   private static final int NUMBER_OF_VARIABLES = 5000;

   @Test
   public void testWriteBufferAtWithLargeMixedTypePopulation()
   {
      Random random = new Random(4620);

      YoRegistry rootRegistry = new YoRegistry("root");
      List<YoVariable> allVariables = new ArrayList<>();

      for (int i = 0; i < NUMBER_OF_VARIABLES; i++)
      {
         YoVariable variable = switch (i % 5)
         {
            case 0 -> SharedMemoryRandomTools.nextYoBoolean(random, "var" + i, rootRegistry);
            case 1 -> SharedMemoryRandomTools.nextYoDouble(random, "var" + i, rootRegistry);
            case 2 -> SharedMemoryRandomTools.nextYoInteger(random, "var" + i, rootRegistry);
            case 3 -> SharedMemoryRandomTools.nextYoLong(random, "var" + i, rootRegistry);
            default -> SharedMemoryRandomTools.nextYoEnum(random, "var" + i, rootRegistry);
         };
         allVariables.add(variable);
      }

      YoBufferProperties bufferProperties = new YoBufferProperties(0, 37);
      YoRegistryBuffer registryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

      // The type-segregated internal buckets must not disturb the externally-visible insertion order.
      List<YoVariableBuffer<?>> buffers = registryBuffer.getYoVariableBuffers();
      assertEquals(allVariables.size(), buffers.size());
      for (int i = 0; i < allVariables.size(); i++)
         assertSame(allVariables.get(i), buffers.get(i).getYoVariable());

      for (int index = 0; index < bufferProperties.getSize(); index++)
      {
         allVariables.forEach(v -> SharedMemoryRandomTools.randomizeYoVariable(random, v));
         registryBuffer.writeBufferAt(index);

         for (YoVariable variable : allVariables)
            assertBufferValueEquals(index, variable, registryBuffer.findYoVariableBuffer(variable));
      }
   }

   @Test
   public void testWriteBufferAtWithEmptyList()
   {
      YoVariableBufferList list = new YoVariableBufferList();
      list.writeBufferAt(0); // Must not throw (no workers, no buckets).
      assertEquals(0, list.size());
   }

   /**
    * A population with only one of the five concrete types populates 4 of the 5 buckets with size 0 - the
    * partitioning math ({@code partitionBound}) must handle that without producing an out-of-range slice.
    */
   @Test
   public void testWriteBufferAtWithSingleTypePopulation()
   {
      Random random = new Random(88512);
      YoRegistry rootRegistry = new YoRegistry("root");
      List<YoVariable> allVariables = new ArrayList<>();
      for (int i = 0; i < 500; i++)
         allVariables.add(SharedMemoryRandomTools.nextYoDouble(random, "d" + i, rootRegistry));

      YoBufferProperties bufferProperties = new YoBufferProperties(0, 11);
      YoRegistryBuffer registryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

      for (int index = 0; index < bufferProperties.getSize(); index++)
      {
         allVariables.forEach(v -> SharedMemoryRandomTools.randomizeYoVariable(random, v));
         registryBuffer.writeBufferAt(index);
         for (YoVariable variable : allVariables)
            assertBufferValueEquals(index, variable, registryBuffer.findYoVariableBuffer(variable));
      }
   }

   private static void assertBufferValueEquals(int index, YoVariable expected, YoVariableBuffer<?> buffer)
   {
      if (expected instanceof YoBoolean v)
         assertEquals(v.getValue(), ((YoBooleanBuffer) buffer).getBuffer()[index]);
      else if (expected instanceof YoDouble v)
         assertEquals(v.getValue(), ((YoDoubleBuffer) buffer).getBuffer()[index]);
      else if (expected instanceof YoInteger v)
         assertEquals(v.getValue(), ((YoIntegerBuffer) buffer).getBuffer()[index]);
      else if (expected instanceof YoLong v)
         assertEquals(v.getValue(), ((YoLongBuffer) buffer).getBuffer()[index]);
      else if (expected instanceof YoEnum<?> v)
         assertEquals(v.getOrdinal(), ((YoEnumBuffer<?>) buffer).getBuffer()[index]);
      else
         throw new IllegalStateException("Unhandled variable type: " + expected.getClass());
   }
}
