package us.ihmc.scs2.sharedMemory;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoRegistryBufferTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructor()
   {
      Random random = new Random(62825);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoRegistry[] allRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, 10, 15);
         YoRegistry rootRegistry = allRegistries[0];
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoRegistryBuffer YoRegistryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

         assertTrue(rootRegistry == YoRegistryBuffer.getRootRegistry());

         List<YoVariable> allYoVariables = rootRegistry.collectSubtreeVariables();

         for (YoVariable yoVariable : allYoVariables)
         {
            YoVariableBuffer<?> yoVariableBuffer = YoRegistryBuffer.findYoVariableBuffer(yoVariable);
            assertNotNull(yoVariableBuffer);
            assertTrue(yoVariable == yoVariableBuffer.getYoVariable());
         }
      }
   }

   @Test
   public void testRegisterMissingBuffers()
   {
      Random random = new Random(74586);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoRegistry[] allRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, 5, 5);
         YoRegistry rootRegistry = allRegistries[0];
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoRegistryBuffer yoRegistryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

         for (int j = 0; j < allRegistries.length; j++)
         {
            YoRegistry yoRegistry = allRegistries[j];
            SharedMemoryRandomTools.nextYoRegistryTree(random, yoRegistry, "new" + j, 5, 5);
         }

         List<YoVariable> allYoVariables = rootRegistry.collectSubtreeVariables();

         for (YoVariable yoVariable : allYoVariables)
         {
            YoVariableBuffer<?> yoVariableBuffer = yoRegistryBuffer.findYoVariableBuffer(yoVariable);
            assertNotNull(yoVariableBuffer);
            assertTrue(yoVariable == yoVariableBuffer.getYoVariable());
         }
      }
   }

   @Test
   public void testResizeBuffer()
   {
      Random random = new Random(734259);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoRegistry[] allRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, 2, 2);
         YoRegistry rootRegistry = allRegistries[0];
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoRegistryBuffer YoRegistryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);
         SharedMemoryRandomTools.randomizeYoRegistryBuffer(random, YoRegistryBuffer);

         List<YoVariable> allYoVariables = rootRegistry.collectSubtreeVariables();
         List<YoVariableBuffer<?>> allYoVariableBuffers = new ArrayList<>();
         List<BufferSample<?>> allSamples = new ArrayList<>();

         int from = random.nextInt(bufferProperties.getSize());
         int length = random.nextInt(bufferProperties.getSize() - 1) + 1;

         for (YoVariable yoVariable : allYoVariables)
         {
            YoVariableBuffer<?> yoVariableBuffer = YoRegistryBuffer.findYoVariableBuffer(yoVariable);
            allYoVariableBuffers.add(yoVariableBuffer);
            allSamples.add(new BufferSample<>(0, yoVariableBuffer.copy(from, length, bufferProperties.copy()).getSample(), length, bufferProperties));
         }

         YoRegistryBuffer.resizeBuffer(from, length);

         for (int j = 0; j < allYoVariables.size(); j++)
         {
            BufferSample<?> expected = allYoVariableBuffers.get(j).copy(0, length, bufferProperties.copy());
            expected = new BufferSample<>(expected.getFrom(), expected.getSample(), expected.getSampleLength(), bufferProperties);
            BufferSample<?> actual = allSamples.get(j);
            assertEquals(expected, actual);
         }
      }
   }

   @Test
   public void testWriteBuffer()
   {
      Random random = new Random(734259);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoRegistry[] allRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, 5, 5);
         YoRegistry rootRegistry = allRegistries[0];
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoRegistryBuffer YoRegistryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

         List<YoVariable> allYoVariables = rootRegistry.collectSubtreeVariables();

         for (int j = 0; j < 10; j++)
         {
            allYoVariables.forEach(v -> SharedMemoryRandomTools.randomizeYoVariable(random, v));
            YoRegistryBuffer.writeBuffer();
            for (YoVariable yoVariable : allYoVariables)
            {
               assertBufferCurrentValueEquals(bufferProperties.getCurrentIndex(), yoVariable, YoRegistryBuffer.findYoVariableBuffer(yoVariable));
            }
         }
      }
   }

   @Test
   public void testReadBuffer()
   {
      Random random = new Random(734259);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoRegistry[] allRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, 5, 5);
         YoRegistry rootRegistry = allRegistries[0];
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoRegistryBuffer YoRegistryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

         List<YoVariable> allYoVariables = rootRegistry.collectSubtreeVariables();
         List<YoVariableBuffer<?>> allYoVariableBuffers = allYoVariables.stream().map(YoRegistryBuffer::findYoVariableBuffer).collect(Collectors.toList());

         for (int j = 0; j < 10; j++)
         {
            allYoVariableBuffers.forEach(b -> randomizeBuffer(random, bufferProperties.getCurrentIndex(), b));
            YoRegistryBuffer.readBuffer();
            for (YoVariable yoVariable : allYoVariables)
            {
               assertBufferCurrentValueEquals(bufferProperties.getCurrentIndex(), yoVariable, YoRegistryBuffer.findYoVariableBuffer(yoVariable));
            }
         }
      }
   }

   @Test
   public void testFindOrCreateYoVariableBuffer()
   {
      Random random = new Random(78924);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoRegistry[] allRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, 5, 5);
         YoRegistry rootRegistry = allRegistries[0];
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoRegistryBuffer yoRegistryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

         // Create a mirror of the registries
         YoRegistry rootMirrorRegistry = new YoRegistry(rootRegistry.getName());
         SharedMemoryTools.duplicateMissingYoVariablesInTarget(rootRegistry, rootMirrorRegistry);

         // Add a YoVariable to one of the existing registries
         YoRegistry yoRegistry = rootMirrorRegistry.collectSubtreeRegistries().get(random.nextInt(allRegistries.length - 1));
         YoVariable newYoVariable = SharedMemoryRandomTools.nextYoVariable(random, yoRegistry);
         assertNull(yoRegistryBuffer.findYoVariableBuffer(newYoVariable));
         assertNull(rootRegistry.findVariable(newYoVariable.getFullNameString()));
         YoVariableBuffer<?> newBuffer = yoRegistryBuffer.findOrCreateYoVariableBuffer(newYoVariable);
         assertEquals(newYoVariable.getFullNameString(), newBuffer.getYoVariable().getFullNameString());
         assertNotNull(yoRegistryBuffer.findYoVariableBuffer(newYoVariable));
         assertNotNull(rootRegistry.findVariable(newYoVariable.getFullNameString()));
         assertEquals(newYoVariable.getClass(), rootRegistry.findVariable(newYoVariable.getFullNameString()).getClass());

         // Add a YoVariable with new registries to one of the existing registries.
         List<YoRegistry> newRegistries = new ArrayList<>();

         for (int j = 0; j < random.nextInt(10) + 1; j++)
         {
            YoRegistry newChild = new YoRegistry(SharedMemoryRandomTools.nextAvailableRegistryName(random, 10, 20, yoRegistry));
            yoRegistry.addChild(newChild);
            newRegistries.add(newChild);
            yoRegistry = newChild;
         }

         newYoVariable = SharedMemoryRandomTools.nextYoVariable(random, yoRegistry);
         assertNull(yoRegistryBuffer.findYoVariableBuffer(newYoVariable));
         assertNull(rootRegistry.findVariable(newYoVariable.getFullNameString()));
         newBuffer = yoRegistryBuffer.findOrCreateYoVariableBuffer(newYoVariable);
         assertEquals(newYoVariable.getFullNameString(), newBuffer.getYoVariable().getFullNameString());
         assertNotNull(yoRegistryBuffer.findYoVariableBuffer(newYoVariable));
         assertNotNull(rootRegistry.findVariable(newYoVariable.getFullNameString()));
         assertEquals(newYoVariable.getClass(), rootRegistry.findVariable(newYoVariable.getFullNameString()).getClass());
         newRegistries.forEach(newRegistry -> assertNotNull(rootRegistry.findRegistry(newRegistry.getNamespace())));
      }
   }

   @Test
   public void testNewLinkedYoRegistry()
   {
      Random random = new Random(978345);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YoRegistry[] allRegistries = SharedMemoryRandomTools.nextYoRegistryTree(random, 5, 5);
         YoRegistry rootRegistry = allRegistries[0];
         YoBufferProperties bufferProperties = SharedMemoryRandomTools.nextYoBufferProperties(random);
         YoRegistryBuffer yoRegistryBuffer = new YoRegistryBuffer(rootRegistry, bufferProperties);

         LinkedYoRegistry linkedRootRegistry = yoRegistryBuffer.newLinkedYoRegistry();
         assertEquals(rootRegistry.getName(), linkedRootRegistry.getRootRegistry().getName());

         for (YoRegistry registry : allRegistries)
         {
            YoRegistry linkedRegistry = linkedRootRegistry.getRootRegistry().findRegistry(registry.getNamespace());
            assertNotNull(linkedRegistry);
            assertEquals(linkedRegistry.getChildren().size(), registry.getChildren().size());
            assertEquals(linkedRegistry.getVariables().size(), registry.getVariables().size());
         }

         YoRegistry subTreeRootRegistry = allRegistries[random.nextInt(allRegistries.length)];
         YoRegistry linkedSubTreeRootRegistry = SharedMemoryTools.newEmptyCloneRegistry(subTreeRootRegistry);

         LinkedYoRegistry linkedSubTreeRegistry = yoRegistryBuffer.newLinkedYoRegistry(linkedSubTreeRootRegistry);
         assertTrue(linkedSubTreeRootRegistry == linkedSubTreeRegistry.getRootRegistry());

         for (YoRegistry registry : subTreeRootRegistry.collectSubtreeRegistries())
         {
            YoRegistry linkedRegistry = linkedSubTreeRootRegistry.findRegistry(registry.getNamespace());
            assertNotNull(linkedRegistry);
            assertEquals(linkedRegistry.getChildren().size(), registry.getChildren().size());
            assertEquals(linkedRegistry.getVariables().size(), registry.getVariables().size());
         }
      }
   }

   private static void assertBufferCurrentValueEquals(int currentIndex, YoVariable expectedValue, YoVariableBuffer<?> yoVariableBuffer)
   {
      if (expectedValue instanceof YoBoolean)
         assertEquals(((YoBoolean) expectedValue).getValue(), ((YoBooleanBuffer) yoVariableBuffer).getBuffer()[currentIndex]);
      else if (expectedValue instanceof YoDouble)
         assertEquals(((YoDouble) expectedValue).getValue(), ((YoDoubleBuffer) yoVariableBuffer).getBuffer()[currentIndex]);
      else if (expectedValue instanceof YoInteger)
         assertEquals(((YoInteger) expectedValue).getValue(), ((YoIntegerBuffer) yoVariableBuffer).getBuffer()[currentIndex]);
      else if (expectedValue instanceof YoLong)
         assertEquals(((YoLong) expectedValue).getValue(), ((YoLongBuffer) yoVariableBuffer).getBuffer()[currentIndex]);
      else if (expectedValue instanceof YoEnum)
         assertEquals(((YoEnum<?>) expectedValue).getOrdinal(), ((YoEnumBuffer<?>) yoVariableBuffer).getBuffer()[currentIndex]);
      else
         throw new IllegalStateException("Unhandled variable type.");
   }

   private static void randomizeBuffer(Random random, int index, YoVariableBuffer<?> yoVariableBuffer)
   {
      if (yoVariableBuffer instanceof YoBooleanBuffer)
      {
         ((YoBooleanBuffer) yoVariableBuffer).getBuffer()[index] = random.nextBoolean();
      }
      else if (yoVariableBuffer instanceof YoDoubleBuffer)
      {
         ((YoDoubleBuffer) yoVariableBuffer).getBuffer()[index] = random.nextDouble();
      }
      else if (yoVariableBuffer instanceof YoIntegerBuffer)
      {
         ((YoIntegerBuffer) yoVariableBuffer).getBuffer()[index] = random.nextInt();
      }
      else if (yoVariableBuffer instanceof YoLongBuffer)
      {
         ((YoLongBuffer) yoVariableBuffer).getBuffer()[index] = random.nextLong();
      }
      else if (yoVariableBuffer instanceof YoEnumBuffer)
      {
         YoEnumBuffer<?> yoEnumBuffer = (YoEnumBuffer<?>) yoVariableBuffer;
         if (yoEnumBuffer.getYoVariable().isNullAllowed())
            yoEnumBuffer.getBuffer()[index] = (byte) (random.nextInt(yoEnumBuffer.getYoVariable().getEnumSize() + 1) - 1);
         else
            yoEnumBuffer.getBuffer()[index] = (byte) random.nextInt(yoEnumBuffer.getYoVariable().getEnumSize());
      }
      else
      {
         throw new IllegalStateException("Unhandled buffer type.");
      }
   }
}
