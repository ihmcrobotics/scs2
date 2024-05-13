package us.ihmc.scs2.sharedMemory.tools;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.scs2.definition.yoVariable.YoVariableDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableType;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * This class provides utility methods for shared memory operations.
 */
public class SharedMemoryTools
{
   /**
    * Calculates the size of a registry in bytes.
    *
    * @param registry the registry to calculate the size of.
    * @return the size of the registry in bytes.
    */
   public static long getRegistryMemorySize(YoRegistry registry)
   {
      long size = getVariablesMemorySize(registry.getVariables());

      for (YoRegistry child : registry.getChildren())
      {
         size += getRegistryMemorySize(child);
      }

      return size;
   }

   /**
    * Calculates the size of a list of variables in bytes.
    *
    * @param yoVariables the list of variables to calculate the size of.
    * @return the size of the list of variables in bytes.
    */
   public static long getVariablesMemorySize(Collection<YoVariable> yoVariables)
   {
      int size = 0;

      for (YoVariable yoVariable : yoVariables)
      {
         size += getVariableMemorySize(yoVariable);
      }

      return size;
   }

   /**
    * Calculates the size of a variable in bytes.
    *
    * @param yoVariable the variable to calculate the size of.
    * @return the size of the variable in bytes.
    */
   public static int getVariableMemorySize(YoVariable yoVariable)
   {
      return switch (yoVariable.getType())
      {
         case DOUBLE -> Double.BYTES;
         case INTEGER -> Integer.BYTES;
         case LONG -> Long.BYTES;
         case BOOLEAN, ENUM -> Byte.BYTES;
      };
   }

   /**
    * Concatenates multiple arrays into a single array.
    *
    * @param <T>    the type of the array elements
    * @param arrays the arrays to concatenate
    * @return the concatenated array
    */
   @SuppressWarnings("unchecked")
   public static <T> T[] concatenate(T[]... arrays)
   {
      if (arrays == null || arrays.length == 0)
         return null;

      int length = 0;
      for (T[] array : arrays)
      {
         length += array.length;
      }

      T[] result = (T[]) Array.newInstance(arrays[0].getClass().getComponentType(), length);
      int index = 0;
      for (T[] array : arrays)
      {
         for (T element : array)
         {
            result[index++] = element;
         }
      }
      return result;
   }

   /**
    * Increments an index by a step size, wrapping around to zero if the end of the array is reached.
    *
    * @param index    the current index
    * @param stepSize the step size
    * @param size     the size of the array
    * @return the incremented index
    */
   public static int increment(int index, int stepSize, int size)
   {
      index += stepSize;
      if (index >= size)
         index -= size;
      return index;
   }

   /**
    * Decrements an index by a step size, wrapping around to the end of the array if the beginning is reached.
    *
    * @param index    the current index
    * @param stepSize the step size
    * @param size     the size of the array
    * @return the decremented index
    */
   public static int decrement(int index, int stepSize, int size)
   {
      index -= stepSize;
      if (index < 0)
         index += size;
      return index;
   }

   /**
    * Calculates the sub-length defined by the interval [{@code from}, {@code to}] in a ring buffer of
    * size {@code length}.
    *
    * @param from   the (inclusive) start of the interval.
    * @param to     the (inclusive) end of the interval.
    * @param length the number of elements in the ring buffer.
    * @return the number of elements contained in the interval [{@code from}, {@code to}].
    * @throws IllegalArgumentException if {@code length} is negative or equal to zero.
    * @throws IndexOutOfBoundsException if {@code from} is not &in; [0, {@code length}[.
    * @throws IndexOutOfBoundsException if {@code to} is not &in; [0, {@code length}[.
    */
   public static int computeSubLength(int from, int to, int length)
   {
      if (length <= 0)
         throw new IllegalArgumentException("length must be greater than zero, was: " + length);
      if (from < 0 || from >= length)
         throw new IndexOutOfBoundsException("from is out-of-bound, should be in [0, " + length + "[, but was: " + from);
      if (to < 0 || to >= length)
         throw new IndexOutOfBoundsException("to is out-of-bound, should be in [0, " + length + "[, but was: " + to);

      int subLength = to - from + 1;
      if (subLength <= 0)
         subLength += length;
      return subLength;
   }

   /**
    * Calculates the start point of an interval defined in a ring buffer.
    *
    * @param to        the (inclusive) end of the interval.
    * @param subLength the number of elements in the interval.
    * @param length    the number of elements of the entire ring buffer.
    * @return the index of the interval's start point.
    * @throws IllegalArgumentException if {@code length} is negative or equal to zero.
    * @throws IllegalArgumentException if {@code subLength} is either negative, equal to zero or
    *       greater than {@code length}.
    * @throws IndexOutOfBoundsException if {@code to} is not &in; [0, {@code length}[.
    */
   public static int computeFromIndex(int to, int subLength, int length)
   {
      if (length <= 0)
         throw new IllegalArgumentException("length must be greater than zero, was: " + length);
      if (subLength <= 0)
         throw new IllegalArgumentException("sub-length must be greater than zero, was: " + subLength);
      if (subLength > length)
         throw new IllegalArgumentException("sub-length must be smaller than length, was: " + subLength + ", length was " + length);
      if (to < 0 || to >= length)
         throw new IndexOutOfBoundsException("to is out-of-bound, should be in [0, " + length + "[, but was: " + to);

      int from = to - subLength + 1;
      if (from < 0)
         from += length;
      return from;
   }

   /**
    * Calculates the end point of an interval defined in a ring buffer.
    *
    * @param from      the (inclusive) start of the interval.
    * @param subLength the number of elements in the interval.
    * @param length    the number of elements of the entire ring buffer.
    * @return the index of the interval's end point.
    * @throws IllegalArgumentException if {@code length} is negative or equal to zero.
    * @throws IllegalArgumentException if {@code subLength} is either negative, equal to zero or
    *       greater than {@code length}.
    * @throws IndexOutOfBoundsException if {@code from} is not &in; [0, {@code length}[.
    */
   public static int computeToIndex(int from, int subLength, int length)
   {
      if (length <= 0)
         throw new IllegalArgumentException("length must be greater than zero, was: " + length);
      if (subLength <= 0)
         throw new IllegalArgumentException("sub-length must be greater than zero, was: " + subLength);
      if (subLength > length)
         throw new IllegalArgumentException("sub-length must be smaller than length, was: " + subLength + ", length was " + length);
      if (from < 0 || from >= length)
         throw new IndexOutOfBoundsException("from is out-of-bound, should be in [0, " + length + "[, but was: " + from);

      int to = from + subLength - 1;
      if (to >= length)
         to -= length;
      return to;
   }

   /**
    * Tests whether the {@code query} is contained within the sub-interval of a buffer defined by
    * [{@code start}, {@code to}].
    *
    * @param query the index to test.
    * @param start the (inclusive) start of the interval.
    * @param end   the (inclusive) end of the interval.
    * @param size  the size of the buffer.
    * @return {@code true} if {@code query} &in; [{@code start}, {@code to}], {@code false} otherwise.
    */
   public static boolean isInsideBounds(int query, int start, int end, int size)
   {
      if (query < 0 || query >= size)
         return false;
      else if (start <= end)
         return query >= start && query <= end;
      else
         return query <= end || query >= start;
   }

   /**
    * Copies a portion of a ring array into a new array.
    *
    * @param ringArray the ring array to copy from
    * @param from      the start index in the ring array
    * @param newLength the length of the new array
    * @return the new array
    */
   public static boolean[] ringArrayCopy(boolean[] ringArray, int from, int newLength)
   {
      int length = Math.min(newLength, ringArray.length);

      boolean[] bufferCopy;
      if (from + length <= ringArray.length)
      {
         bufferCopy = Arrays.copyOfRange(ringArray, from, from + newLength);
      }
      else
      {
         int lengthOfFirstCopy = ringArray.length - from;
         bufferCopy = new boolean[newLength];
         System.arraycopy(ringArray, from, bufferCopy, 0, lengthOfFirstCopy);
         System.arraycopy(ringArray, 0, bufferCopy, lengthOfFirstCopy, length - lengthOfFirstCopy);
      }
      return bufferCopy;
   }

   /**
    * Copies a portion of a ring array into a new array.
    *
    * @param ringArray the ring array to copy from
    * @param from      the start index in the ring array
    * @param newLength the length of the new array
    * @return the new array
    */
   public static double[] ringArrayCopy(double[] ringArray, int from, int newLength)
   {
      int length = Math.min(newLength, ringArray.length);

      double[] bufferCopy;
      if (from + length <= ringArray.length)
      {
         bufferCopy = Arrays.copyOfRange(ringArray, from, from + newLength);
      }
      else
      {
         int lengthOfFirstCopy = ringArray.length - from;
         bufferCopy = new double[newLength];
         System.arraycopy(ringArray, from, bufferCopy, 0, lengthOfFirstCopy);
         System.arraycopy(ringArray, 0, bufferCopy, lengthOfFirstCopy, length - lengthOfFirstCopy);
      }
      return bufferCopy;
   }

   /**
    * Copies a portion of a ring array into a new array.
    *
    * @param ringArray the ring array to copy from
    * @param from      the start index in the ring array
    * @param newLength the length of the new array
    * @return the new array
    */
   public static int[] ringArrayCopy(int[] ringArray, int from, int newLength)
   {
      int length = Math.min(newLength, ringArray.length);

      int[] bufferCopy;
      if (from + length <= ringArray.length)
      {
         bufferCopy = Arrays.copyOfRange(ringArray, from, from + newLength);
      }
      else
      {
         int lengthOfFirstCopy = ringArray.length - from;
         bufferCopy = new int[newLength];
         System.arraycopy(ringArray, from, bufferCopy, 0, lengthOfFirstCopy);
         System.arraycopy(ringArray, 0, bufferCopy, lengthOfFirstCopy, length - lengthOfFirstCopy);
      }
      return bufferCopy;
   }

   /**
    * Copies a portion of a ring array into a new array.
    *
    * @param ringArray the ring array to copy from
    * @param from      the start index in the ring array
    * @param newLength the length of the new array
    * @return the new array
    */
   public static long[] ringArrayCopy(long[] ringArray, int from, int newLength)
   {
      int length = Math.min(newLength, ringArray.length);

      long[] bufferCopy;
      if (from + length <= ringArray.length)
      {
         bufferCopy = Arrays.copyOfRange(ringArray, from, from + newLength);
      }
      else
      {
         int lengthOfFirstCopy = ringArray.length - from;
         bufferCopy = new long[newLength];
         System.arraycopy(ringArray, from, bufferCopy, 0, lengthOfFirstCopy);
         System.arraycopy(ringArray, 0, bufferCopy, lengthOfFirstCopy, length - lengthOfFirstCopy);
      }
      return bufferCopy;
   }

   /**
    * Copies a portion of a ring array into a new array.
    *
    * @param ringArray the ring array to copy from
    * @param from      the start index in the ring array
    * @param newLength the length of the new array
    * @return the new array
    */
   public static byte[] ringArrayCopy(byte[] ringArray, int from, int newLength)
   {
      int length = Math.min(newLength, ringArray.length);

      byte[] bufferCopy;
      if (from + length <= ringArray.length)
      {
         bufferCopy = Arrays.copyOfRange(ringArray, from, from + newLength);
      }
      else
      {
         int lengthOfFirstCopy = ringArray.length - from;
         bufferCopy = new byte[newLength];
         System.arraycopy(ringArray, from, bufferCopy, 0, lengthOfFirstCopy);
         System.arraycopy(ringArray, 0, bufferCopy, lengthOfFirstCopy, length - lengthOfFirstCopy);
      }
      return bufferCopy;
   }

   /**
    * Fills a portion of a ring array with a specified value.
    *
    * @param ringArray the ring array to fill
    * @param fillValue the value to fill the array with
    * @param from      the start index in the ring array
    * @param length    the number of elements to fill
    */
   public static void ringArrayFill(boolean[] ringArray, boolean fillValue, int from, int length)
   {
      length = Math.min(length, ringArray.length);

      if (from + length <= ringArray.length)
      {
         Arrays.fill(ringArray, from, from + length, fillValue);
      }
      else
      {
         Arrays.fill(ringArray, from, ringArray.length, fillValue);
         Arrays.fill(ringArray, 0, from + length - ringArray.length, fillValue);
      }
   }

   /**
    * Fills a portion of a ring array with a specified value.
    *
    * @param ringArray the ring array to fill
    * @param fillValue the value to fill the array with
    * @param from      the start index in the ring array
    * @param length    the number of elements to fill
    */
   public static void ringArrayFill(double[] ringArray, double fillValue, int from, int length)
   {
      length = Math.min(length, ringArray.length);

      if (from + length <= ringArray.length)
      {
         Arrays.fill(ringArray, from, from + length, fillValue);
      }
      else
      {
         Arrays.fill(ringArray, from, ringArray.length, fillValue);
         Arrays.fill(ringArray, 0, from + length - ringArray.length, fillValue);
      }
   }

   /**
    * Fills a portion of a ring array with a specified value.
    *
    * @param ringArray the ring array to fill
    * @param fillValue the value to fill the array with
    * @param from      the start index in the ring array
    * @param length    the number of elements to fill
    */
   public static void ringArrayFill(int[] ringArray, int fillValue, int from, int length)
   {
      length = Math.min(length, ringArray.length);

      if (from + length <= ringArray.length)
      {
         Arrays.fill(ringArray, from, from + length, fillValue);
      }
      else
      {
         Arrays.fill(ringArray, from, ringArray.length, fillValue);
         Arrays.fill(ringArray, 0, from + length - ringArray.length, fillValue);
      }
   }

   /**
    * Fills a portion of a ring array with a specified value.
    *
    * @param ringArray the ring array to fill
    * @param fillValue the value to fill the array with
    * @param from      the start index in the ring array
    * @param length    the number of elements to fill
    */
   public static void ringArrayFill(long[] ringArray, long fillValue, int from, int length)
   {
      length = Math.min(length, ringArray.length);

      if (from + length <= ringArray.length)
      {
         Arrays.fill(ringArray, from, from + length, fillValue);
      }
      else
      {
         Arrays.fill(ringArray, from, ringArray.length, fillValue);
         Arrays.fill(ringArray, 0, from + length - ringArray.length, fillValue);
      }
   }

   /**
    * Fills a portion of a ring array with a specified value.
    *
    * @param ringArray the ring array to fill
    * @param fillValue the value to fill the array with
    * @param from      the start index in the ring array
    * @param length    the number of elements to fill
    */
   public static void ringArrayFill(byte[] ringArray, byte fillValue, int from, int length)
   {
      length = Math.min(length, ringArray.length);

      if (from + length <= ringArray.length)
      {
         Arrays.fill(ringArray, from, from + length, fillValue);
      }
      else
      {
         Arrays.fill(ringArray, from, ringArray.length, fillValue);
         Arrays.fill(ringArray, 0, from + length - ringArray.length, fillValue);
      }
   }

   /**
    * Converts a boolean array into a double array, where {@code true} is mapped to {@code 1.0} and {@code false} is mapped to {@code 0.0}.
    *
    * @param array the boolean array to convert
    * @return the double array
    */
   public static double[] toDoubleArray(boolean[] array)
   {
      if (array == null)
         return null;

      double[] doubleArray = new double[array.length];

      for (int i = 0; i < array.length; i++)
         doubleArray[i] = array[i] ? 1.0 : 0.0;
      return doubleArray;
   }

   /**
    * Converts a int array into a double array.
    *
    * @param array the int array to convert
    * @return the double array
    */
   public static double[] toDoubleArray(int[] array)
   {
      if (array == null)
         return null;

      double[] doubleArray = new double[array.length];

      for (int i = 0; i < array.length; i++)
         doubleArray[i] = array[i];
      return doubleArray;
   }

   /**
    * Converts a long array into a double array.
    *
    * @param array the long array to convert
    * @return the double array
    */
   public static double[] toDoubleArray(long[] array)
   {
      if (array == null)
         return null;

      double[] doubleArray = new double[array.length];

      for (int i = 0; i < array.length; i++)
         doubleArray[i] = array[i];
      return doubleArray;
   }

   /**
    * Converts a byte array into a double array.
    *
    * @param array the byte array to convert
    * @return the double array
    */
   public static double[] toDoubleArray(byte[] array)
   {
      if (array == null)
         return null;

      double[] doubleArray = new double[array.length];

      for (int i = 0; i < array.length; i++)
         doubleArray[i] = array[i];
      return doubleArray;
   }

   /**
    * Creates a new empty clone of the given registry.
    * The clone will have the same name and parent hierarchy as the original registry,
    * but it will not contain any variables or children registries.
    *
    * @param original the registry to clone
    * @return the new empty clone registry
    */
   public static YoRegistry newEmptyCloneRegistry(YoRegistry original)
   {
      // Create a new registry with the same name as the original
      YoRegistry clone = new YoRegistry(original.getName());

      // Get the parent of the original registry
      YoRegistry originalParent = original.getParent();
      YoRegistry currentClone = clone;

      // Loop through the parent hierarchy of the original registry
      while (originalParent != null)
      {
         // For each parent in the hierarchy, create a new registry with the same name
         YoRegistry parentClone = new YoRegistry(originalParent.getName());
         // Add the current clone as a child of the new parent clone
         parentClone.addChild(currentClone);
         // Set the current clone to the new parent clone
         currentClone = parentClone;
         // Move up to the next parent in the original registry's hierarchy
         originalParent = originalParent.getParent();
      }

      // Return the topmost clone in the hierarchy
      return clone;
   }

   /**
    * Creates a new registry from a given namespace.
    * The namespace is provided as a varargs of Strings, which are then converted into a YoNamespace object.
    * This method delegates the creation of the new registry to the overloaded method newRegistryFromNamespace(YoNamespace).
    *
    * @param namespace the namespace for the new registry, provided as a varargs of Strings
    * @return the new registry created from the given namespace
    */
   public static YoRegistry newRegistryFromNamespace(String... namespace)
   {
      return newRegistryFromNamespace(new YoNamespace(Arrays.asList(namespace)));
   }

   /**
    * Creates a new registry from a given namespace.
    *
    * @param namespace the namespace for the new registry
    * @return the new registry created from the given namespace
    */
   public static YoRegistry newRegistryFromNamespace(YoNamespace namespace)
   {
      YoRegistry registry = null;

      for (String subName : namespace.getSubNames())
      {
         YoRegistry child = new YoRegistry(subName);
         if (registry != null)
            registry.addChild(child);
         registry = child;
      }

      return registry;
   }

   /**
    * Duplicates missing YoVariables from the original registry to the target registry.
    * This method delegates the creation of the new registry to the overloaded method newRegistryFromNamespace(YoNamespace).
    *
    * @param original the original registry
    * @param target   the target registry
    * @return the number of YoVariables created
    */
   public static int duplicateMissingYoVariablesInTarget(YoRegistry original, YoRegistry target)
   {
      return duplicateMissingYoVariablesInTarget(original, target, yoVariable ->
      {
      });
   }

   /**
    * Duplicates missing YoVariables from the original registry to the target registry.
    * This method also accepts a Consumer for handling new YoVariables.
    *
    * @param original              the original registry
    * @param target                the target registry
    * @param newYoVariableConsumer a Consumer for handling new YoVariables
    * @return the number of YoVariables created
    */
   public static int duplicateMissingYoVariablesInTarget(YoRegistry original, YoRegistry target, Consumer<YoVariable> newYoVariableConsumer)
   {
      int numberOfYoVariablesCreated = 0;

      // Check for missing variables
      Set<String> targetVariableNames = target.getVariables().stream().map(YoVariable::getName).collect(Collectors.toSet());

      for (YoVariable originalVariable : original.getVariables())
      {
         if (!targetVariableNames.contains(originalVariable.getName()))
         { // FIXME YoEnum.duplicate needs to handle this case.
            YoVariable newYoVariable;
            if (originalVariable instanceof YoEnum<?> originalEnum && !((YoEnum<?>) originalVariable).isBackedByEnum())
            {
               newYoVariable = new YoEnum<>(originalEnum.getName(),
                                            originalEnum.getDescription(),
                                            target,
                                            originalEnum.isNullAllowed(),
                                            originalEnum.getEnumValuesAsString());
            }
            else
            {
               newYoVariable = originalVariable.duplicate(target);
            }
            newYoVariableConsumer.accept(newYoVariable);
            numberOfYoVariablesCreated++;
         }
      }

      // Check for missing registries
      Map<String, YoRegistry> targetChildren = target.getChildren().stream().collect(Collectors.toMap(reg -> reg.getName(), Function.identity()));

      for (YoRegistry originalChild : original.getChildren())
      {
         String childName = originalChild.getName();
         YoRegistry targetChild = targetChildren.get(childName);

         if (targetChild == null)
         {
            targetChild = new YoRegistry(childName);
            target.addChild(targetChild);
         }

         // Recurse down the tree
         numberOfYoVariablesCreated += duplicateMissingYoVariablesInTarget(originalChild, targetChild, newYoVariableConsumer);
      }

      return numberOfYoVariablesCreated;
   }

   /**
    * Ensures that a path exists in the given registry.
    * If the registry is null, a new registry is created with the root name of the namespace.
    * If the namespace does not start with the namespace of the root registry, null is returned.
    * If the namespace equals the namespace of the root registry, the root registry is returned.
    *
    * @param rootRegistry      the root registry
    * @param registryNamespace the namespace for the new registry
    * @return the registry with the ensured path
    */
   public static YoRegistry ensurePathExists(YoRegistry rootRegistry, YoNamespace registryNamespace)
   {
      if (rootRegistry == null)
      {
         rootRegistry = new YoRegistry(registryNamespace.getRootName());
      }
      else if (!registryNamespace.startsWith(rootRegistry.getNamespace()))
      {
         return null;
      }
      else if (registryNamespace.equals(rootRegistry.getNamespace()))
      {
         return rootRegistry;
      }

      List<String> subNames = registryNamespace.removeStart(rootRegistry.getNamespace()).getSubNames();
      YoRegistry currentRegistry = rootRegistry;

      for (String subName : subNames)
      {
         YoRegistry childRegistry = currentRegistry.getChildren().stream().filter(r -> r.getName().equals(subName)).findFirst().orElse(null);
         if (childRegistry == null)
         {
            childRegistry = new YoRegistry(subName);
            currentRegistry.addChild(childRegistry);
         }

         currentRegistry = childRegistry;
      }

      return currentRegistry;
   }

   public static YoVariable ensureYoVariableExists(YoRegistry rootRegistry, String variableFullname, Class<? extends YoVariable> type)
   {
      YoVariable variable = rootRegistry.findVariable(variableFullname);
      if (variable != null)
         return variable;
      YoRegistry parentRegistry = ensurePathExists(rootRegistry, new YoNamespace(variableFullname).getParent());

      if (type == YoDouble.class)
         return new YoDouble(variableFullname, parentRegistry);
      if (type == YoInteger.class)
         return new YoInteger(variableFullname, parentRegistry);
      if (type == YoLong.class)
         new YoLong(variableFullname, parentRegistry);
      if (type == YoBoolean.class)
         new YoBoolean(variableFullname, parentRegistry);
      return null;
   }

   public static YoVariable ensureYoVariableExists(YoRegistry rootRegistry, String variableFullname, YoVariableType type)
   {
      YoVariable variable = rootRegistry.findVariable(variableFullname);
      if (variable != null)
         return variable;
      YoRegistry parentRegistry = ensurePathExists(rootRegistry, new YoNamespace(variableFullname).getParent());

      return switch (type)
      {
         case DOUBLE -> new YoDouble(variableFullname, parentRegistry);
         case INTEGER -> new YoInteger(variableFullname, parentRegistry);
         case LONG -> new YoLong(variableFullname, parentRegistry);
         case BOOLEAN -> new YoBoolean(variableFullname, parentRegistry);
         default -> throw new UnsupportedOperationException("Unhandled type: " + type);
      };
   }

   public static YoVariable ensureYoVariableExists(YoRegistry rootRegistry, YoVariableDefinition definition)
   {
      YoVariable variable = rootRegistry.findVariable(definition.getNamespace(), definition.getName());
      if (variable != null)
         return variable;
      YoRegistry parentRegistry = ensurePathExists(rootRegistry, new YoNamespace(definition.getNamespace()));

      YoVariable yoVariable = SharedMemoryIOTools.toYoVariable(definition);
      yoVariable.setRegistry(parentRegistry);
      return yoVariable;
   }

   /**
    * Duplicates a YoFramePoint2D to a new registry and reference frame.
    *
    * @param original          the original YoFramePoint2D
    * @param newRegistry       the new registry
    * @param newReferenceFrame the new reference frame
    * @return the duplicated YoFramePoint2D
    */
   public static YoFramePoint2D duplicate(YoFramePoint2D original, YoRegistry newRegistry, ReferenceFrame newReferenceFrame)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(original.getYoX().getFullNameString());
      YoDouble y = (YoDouble) newRegistry.findVariable(original.getYoY().getFullNameString());
      return new YoFramePoint2D(x, y, newReferenceFrame);
   }

   /**
    * Duplicates a YoFrameVector2D to a new registry and reference frame.
    *
    * @param original          the original YoFrameVector2D
    * @param newRegistry       the new registry
    * @param newReferenceFrame the new reference frame
    * @return the duplicated YoFrameVector2D
    */
   public static YoFrameVector2D duplicate(YoFrameVector2D original, YoRegistry newRegistry, ReferenceFrame newReferenceFrame)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(original.getYoX().getFullNameString());
      YoDouble y = (YoDouble) newRegistry.findVariable(original.getYoY().getFullNameString());
      return new YoFrameVector2D(x, y, newReferenceFrame);
   }

   /**
    * Duplicates a YoFramePoint3D to a new registry and reference frame.
    *
    * @param original          the original YoFramePoint3D
    * @param newRegistry       the new registry
    * @param newReferenceFrame the new reference frame
    * @return the duplicated YoFramePoint3D
    */
   public static YoFramePoint3D duplicate(YoFramePoint3D original, YoRegistry newRegistry, ReferenceFrame newReferenceFrame)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(original.getYoX().getFullNameString());
      YoDouble y = (YoDouble) newRegistry.findVariable(original.getYoY().getFullNameString());
      YoDouble z = (YoDouble) newRegistry.findVariable(original.getYoZ().getFullNameString());
      return new YoFramePoint3D(x, y, z, newReferenceFrame);
   }

   /**
    * Duplicates a YoFrameVector3D to a new registry and reference frame.
    *
    * @param original          the original YoFrameVector3D
    * @param newRegistry       the new registry
    * @param newReferenceFrame the new reference frame
    * @return the duplicated YoFrameVector3D
    */
   public static YoFrameVector3D duplicate(YoFrameVector3D original, YoRegistry newRegistry, ReferenceFrame newReferenceFrame)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(original.getYoX().getFullNameString());
      YoDouble y = (YoDouble) newRegistry.findVariable(original.getYoY().getFullNameString());
      YoDouble z = (YoDouble) newRegistry.findVariable(original.getYoZ().getFullNameString());
      return new YoFrameVector3D(x, y, z, newReferenceFrame);
   }

   /**
    * Duplicates a YoFrameYawPitchRoll to a new registry and reference frame.
    *
    * @param original          the original YoFrameYawPitchRoll
    * @param newRegistry       the new registry
    * @param newReferenceFrame the new reference frame
    * @return the duplicated YoFrameYawPitchRoll
    */
   public static YoFrameYawPitchRoll duplicate(YoFrameYawPitchRoll original, YoRegistry newRegistry, ReferenceFrame newReferenceFrame)
   {
      YoDouble y = (YoDouble) newRegistry.findVariable(original.getYoYaw().getFullNameString());
      YoDouble p = (YoDouble) newRegistry.findVariable(original.getYoPitch().getFullNameString());
      YoDouble r = (YoDouble) newRegistry.findVariable(original.getYoRoll().getFullNameString());
      return new YoFrameYawPitchRoll(y, p, r, newReferenceFrame);
   }

   /**
    * Duplicates a YoFramePoseUsingYawPitchRoll to a new registry and reference frame.
    *
    * @param original          the original YoFramePoseUsingYawPitchRoll
    * @param newRegistry       the new registry
    * @param newReferenceFrame the new reference frame
    * @return the duplicated YoFramePoseUsingYawPitchRoll
    */
   public static YoFramePoseUsingYawPitchRoll duplicate(YoFramePoseUsingYawPitchRoll original, YoRegistry newRegistry, ReferenceFrame newReferenceFrame)
   {
      return new YoFramePoseUsingYawPitchRoll(duplicate(original.getPosition(), newRegistry, newReferenceFrame),
                                              duplicate(original.getYawPitchRoll(), newRegistry, newReferenceFrame));
   }
}
