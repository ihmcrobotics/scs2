package us.ihmc.scs2.sharedMemory.tools;

import java.util.Arrays;
import java.util.Objects;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.scs2.sharedMemory.YoBooleanBuffer;
import us.ihmc.scs2.sharedMemory.YoDoubleBuffer;
import us.ihmc.scs2.sharedMemory.YoEnumBuffer;
import us.ihmc.scs2.sharedMemory.YoIntegerBuffer;
import us.ihmc.scs2.sharedMemory.YoLongBuffer;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.YoVariableBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class SharedMemoryTestTools
{
   public static void assertYoSharedBufferEquals(YoSharedBuffer expected, YoSharedBuffer actual, double epsilon)
   {
      assertYoSharedBufferEquals(null, expected, actual, epsilon);
   }

   public static void assertYoSharedBufferEquals(String messagePrefix, YoSharedBuffer expected, YoSharedBuffer actual, double epsilon)
   {
      if (expected == null)
      {
         if (actual != null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);
      }
      else
      {
         if (actual == null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);

         if (expected.getProperties().getActiveBufferLength() != actual.getProperties().getActiveBufferLength())
            throwNotEqualAssertionError(messagePrefix, expected, actual);

         assertYoRegistryEquals(expected.getRootRegistry(), actual.getRootRegistry());

         if (expected.getRegistryBuffer().getYoVariableBuffers().size() != actual.getRegistryBuffer().getYoVariableBuffers().size())
            throwNotEqualAssertionError(messagePrefix, expected, actual);

         for (int i = 0; i < expected.getRegistryBuffer().getYoVariableBuffers().size(); i++)
         {
            assertYoVariableBufferEquals(messagePrefix,
                                         expected.getRegistryBuffer().getYoVariableBuffers().get(i),
                                         actual.getRegistryBuffer().getYoVariableBuffers().get(i),
                                         epsilon);
         }
      }
   }

   public static void assertYoVariableBufferEquals(YoVariableBuffer<?> expected, YoVariableBuffer<?> actual, double epsilon)
   {
      assertYoVariableBufferEquals(null, expected, actual, epsilon);
   }

   public static void assertYoVariableBufferEquals(String messagePrefix, YoVariableBuffer<?> expected, YoVariableBuffer<?> actual, double epsilon)
   {
      if (expected == null)
      {
         if (actual != null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);
      }
      else
      {
         if (actual == null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);

         assertYoVariableEquals(expected.getYoVariable(), actual.getYoVariable());
         if (expected instanceof YoBooleanBuffer)
            assertYoBooleanBufferDataEquals(messagePrefix, (YoBooleanBuffer) expected, (YoBooleanBuffer) actual, epsilon);
         else if (expected instanceof YoDoubleBuffer)
            assertYoDoubleBufferDataEquals(messagePrefix, (YoDoubleBuffer) expected, (YoDoubleBuffer) actual, epsilon);
         else if (expected instanceof YoIntegerBuffer)
            assertYoIntegerBufferDataEquals(messagePrefix, (YoIntegerBuffer) expected, (YoIntegerBuffer) actual, epsilon);
         else if (expected instanceof YoLongBuffer)
            assertYoLongBufferDataEquals(messagePrefix, (YoLongBuffer) expected, (YoLongBuffer) actual, epsilon);
         else if (expected instanceof YoEnumBuffer)
            assertYoEnumBufferDataEquals(messagePrefix, (YoEnumBuffer<?>) expected, (YoEnumBuffer<?>) actual, epsilon);
         else
            throw new IllegalArgumentException("Unsupported buffer type: " + expected);
      }
   }

   private static void assertYoBooleanBufferDataEquals(String messagePrefix, YoBooleanBuffer expected, YoBooleanBuffer actual, double epsilon)
   {
      if (expected.getProperties().getActiveBufferLength() != actual.getProperties().getActiveBufferLength())
         throwNotEqualAssertionError(messagePrefix, expected, actual);

      int expectedReadIndex = expected.getProperties().getInPoint();
      int actualReadIndex = actual.getProperties().getInPoint();

      for (int i = 0; i < expected.getProperties().getActiveBufferLength(); i++)
      {
         if (expected.getBuffer()[expectedReadIndex] != actual.getBuffer()[actualReadIndex])
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         expectedReadIndex = SharedMemoryTools.increment(expectedReadIndex, 1, expected.getProperties().getSize());
         actualReadIndex = SharedMemoryTools.increment(actualReadIndex, 1, actual.getProperties().getSize());
      }
   }

   private static void assertYoDoubleBufferDataEquals(String messagePrefix, YoDoubleBuffer expected, YoDoubleBuffer actual, double epsilon)
   {
      if (expected.getProperties().getActiveBufferLength() != actual.getProperties().getActiveBufferLength())
         throwNotEqualAssertionError(messagePrefix, expected, actual);

      int expectedReadIndex = expected.getProperties().getInPoint();
      int actualReadIndex = actual.getProperties().getInPoint();

      for (int i = 0; i < expected.getProperties().getActiveBufferLength(); i++)
      {
         if (!EuclidCoreTools.epsilonEquals(expected.getBuffer()[expectedReadIndex], actual.getBuffer()[actualReadIndex], epsilon))
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         expectedReadIndex = SharedMemoryTools.increment(expectedReadIndex, 1, expected.getProperties().getSize());
         actualReadIndex = SharedMemoryTools.increment(actualReadIndex, 1, actual.getProperties().getSize());
      }
   }

   private static void assertYoIntegerBufferDataEquals(String messagePrefix, YoIntegerBuffer expected, YoIntegerBuffer actual, double epsilon)
   {
      if (expected.getProperties().getActiveBufferLength() != actual.getProperties().getActiveBufferLength())
         throwNotEqualAssertionError(messagePrefix, expected, actual);

      int expectedReadIndex = expected.getProperties().getInPoint();
      int actualReadIndex = actual.getProperties().getInPoint();

      for (int i = 0; i < expected.getProperties().getActiveBufferLength(); i++)
      {
         if (expected.getBuffer()[expectedReadIndex] != actual.getBuffer()[actualReadIndex])
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         expectedReadIndex = SharedMemoryTools.increment(expectedReadIndex, 1, expected.getProperties().getSize());
         actualReadIndex = SharedMemoryTools.increment(actualReadIndex, 1, actual.getProperties().getSize());
      }
   }

   private static void assertYoLongBufferDataEquals(String messagePrefix, YoLongBuffer expected, YoLongBuffer actual, double epsilon)
   {
      if (expected.getProperties().getActiveBufferLength() != actual.getProperties().getActiveBufferLength())
         throwNotEqualAssertionError(messagePrefix, expected, actual);

      int expectedReadIndex = expected.getProperties().getInPoint();
      int actualReadIndex = actual.getProperties().getInPoint();

      for (int i = 0; i < expected.getProperties().getActiveBufferLength(); i++)
      {
         if (expected.getBuffer()[expectedReadIndex] != actual.getBuffer()[actualReadIndex])
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         expectedReadIndex = SharedMemoryTools.increment(expectedReadIndex, 1, expected.getProperties().getSize());
         actualReadIndex = SharedMemoryTools.increment(actualReadIndex, 1, actual.getProperties().getSize());
      }
   }

   private static void assertYoEnumBufferDataEquals(String messagePrefix, YoEnumBuffer<?> expected, YoEnumBuffer<?> actual, double epsilon)
   {
      if (expected.getProperties().getActiveBufferLength() != actual.getProperties().getActiveBufferLength())
         throwNotEqualAssertionError(messagePrefix, expected, actual);

      int expectedReadIndex = expected.getProperties().getInPoint();
      int actualReadIndex = actual.getProperties().getInPoint();

      for (int i = 0; i < expected.getProperties().getActiveBufferLength(); i++)
      {
         if (expected.getBuffer()[expectedReadIndex] != actual.getBuffer()[actualReadIndex])
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         expectedReadIndex = SharedMemoryTools.increment(expectedReadIndex, 1, expected.getProperties().getSize());
         actualReadIndex = SharedMemoryTools.increment(actualReadIndex, 1, actual.getProperties().getSize());
      }
   }

   public static void assertYoBufferPropertiesEquals(YoBufferPropertiesReadOnly expected, YoBufferPropertiesReadOnly actual)
   {
      assertYoBufferPropertiesEquals(null, expected, actual);
   }

   public static void assertYoBufferPropertiesEquals(String messagePrefix, YoBufferPropertiesReadOnly expected, YoBufferPropertiesReadOnly actual)
   {
      if (!expected.equals(actual))
         EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected.toString(), actual.toString());
   }

   public static void assertYoRegistryEquals(YoRegistry expected, YoRegistry actual)
   {
      assertYoRegistryEquals(null, expected, actual);
   }

   public static void assertYoRegistryEquals(String messagePrefix, YoRegistry expected, YoRegistry actual)
   {
      if (expected == null)
      {
         if (actual != null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);
      }
      else
      {
         if (actual == null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);

         if (!Objects.equals(expected.getName(), actual.getName()))
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         if (expected.getVariables().size() != actual.getVariables().size())
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         for (int i = 0; i < expected.getNumberOfVariables(); i++)
         {
            YoVariable expectedVariable = expected.getVariable(i);
            YoVariable actualVariable = actual.getVariable(expectedVariable.getName());
            assertYoVariableEquals(messagePrefix, expectedVariable, actualVariable);
         }

         if (expected.getChildren().size() != actual.getChildren().size())
            EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expected.toString(), actual.toString());
         for (int i = 0; i < expected.getChildren().size(); i++)
         {
            YoRegistry expectedChild = expected.getChildren().get(i);
            YoRegistry actualChild = actual.getChildren().stream().filter(child -> child.getName().equals(expectedChild.getName())).findFirst().orElse(null);
            assertYoRegistryEquals(messagePrefix, expectedChild, actualChild);
         }
      }
   }

   public static void assertYoVariableEquals(YoVariable expected, YoVariable actual)
   {
      assertYoVariableEquals(null, expected, actual);
   }

   public static void assertYoVariableEquals(String messagePrefix, YoVariable expected, YoVariable actual)
   {
      if (expected == null)
      {
         if (actual != null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);
      }
      else
      {
         if (actual == null)
            throwNotEqualAssertionError(messagePrefix, expected, actual);

         if (!Objects.equals(expected.getType(), actual.getType()))
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         if (!Objects.equals(expected.getName(), actual.getName()))
            throwNotEqualAssertionError(messagePrefix, expected, actual);
         if (!Objects.equals(expected.getDescription(), actual.getDescription()))
            throwNotEqualAssertionError(messagePrefix, expected, actual);
      }
   }

   private static void throwNotEqualAssertionError(String messagePrefix, YoRegistry expected, YoRegistry actual)
   {
      String expectedAsString = getYoRegistryString(expected);
      String actualAsString = getYoRegistryString(actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, YoVariable expected, YoVariable actual)
   {
      String expectedAsString = getYoVariableString(expected);
      String actualAsString = getYoVariableString(actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, YoVariableBuffer<?> expected, YoVariableBuffer<?> actual)
   {
      String expectedAsString = getYoVariableBufferString(expected);
      String actualAsString = getYoVariableBufferString(actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   private static void throwNotEqualAssertionError(String messagePrefix, YoSharedBuffer expected, YoSharedBuffer actual)
   {
      String expectedAsString = getYoSharedBufferString(expected);
      String actualAsString = getYoSharedBufferString(actual);
      EuclidCoreTestTools.throwNotEqualAssertionError(messagePrefix, expectedAsString, actualAsString);
   }

   public static String getYoRegistryString(YoRegistry yoRegistry)
   {
      if (yoRegistry == null)
         return "null";
      else
         return yoRegistry.getClass().getSimpleName() + ", name: " + yoRegistry.getName() + ", variables: "
               + EuclidCoreIOTools.getCollectionString("[", "]", ", ", yoRegistry.getVariables(), YoVariable::getName) + ", children: "
               + EuclidCoreIOTools.getCollectionString("[", "]", ", ", yoRegistry.getChildren(), YoRegistry::getName);
   }

   public static String getYoVariableString(YoVariable yoVariable)
   {
      if (yoVariable == null)
         return "null";
      else
         return yoVariable.getClass().getSimpleName() + ", name: " + yoVariable.getName() + ", description: " + yoVariable.getDescription();
   }

   public static String getYoSharedBufferString(YoSharedBuffer yoSharedBuffer)
   {
      if (yoSharedBuffer == null)
         return "null";
      else
         return "properties: " + yoSharedBuffer.getProperties() + ", root registry: " + getYoRegistryString(yoSharedBuffer.getRootRegistry()) + ", buffers: "
               + EuclidCoreIOTools.getCollectionString("[",
                                                       "]",
                                                       ", ",
                                                       yoSharedBuffer.getRegistryBuffer().getYoVariableBuffers(),
                                                       b -> b.getYoVariable().getName());
   }

   public static String getYoVariableBufferString(YoVariableBuffer<?> yoVariableBuffer)
   {
      if (yoVariableBuffer == null)
         return "null";

      String bufferAsString;
      if (yoVariableBuffer instanceof YoBooleanBuffer)
         bufferAsString = Arrays.toString((boolean[]) yoVariableBuffer.getBuffer());
      else if (yoVariableBuffer instanceof YoDoubleBuffer)
         bufferAsString = Arrays.toString((double[]) yoVariableBuffer.getBuffer());
      else if (yoVariableBuffer instanceof YoIntegerBuffer)
         bufferAsString = Arrays.toString((int[]) yoVariableBuffer.getBuffer());
      else if (yoVariableBuffer instanceof YoLongBuffer)
         bufferAsString = Arrays.toString((long[]) yoVariableBuffer.getBuffer());
      else if (yoVariableBuffer instanceof YoEnumBuffer)
         bufferAsString = Arrays.toString((byte[]) yoVariableBuffer.getBuffer());
      else
         bufferAsString = "Unknow type: " + yoVariableBuffer.getBuffer();

      return yoVariableBuffer.getClass().getSimpleName() + ", variable [" + getYoVariableString(yoVariableBuffer.getYoVariable()) + "], buffer properties: ["
            + yoVariableBuffer.getProperties() + "], buffer: " + bufferAsString;
   }
}
