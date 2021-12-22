package us.ihmc.scs2.sharedMemory.tools;

import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.apache.commons.lang3.RandomStringUtils;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.scs2.sharedMemory.YoBooleanBuffer;
import us.ihmc.scs2.sharedMemory.YoBufferProperties;
import us.ihmc.scs2.sharedMemory.YoDoubleBuffer;
import us.ihmc.scs2.sharedMemory.YoEnumBuffer;
import us.ihmc.scs2.sharedMemory.YoIntegerBuffer;
import us.ihmc.scs2.sharedMemory.YoLongBuffer;
import us.ihmc.scs2.sharedMemory.YoRegistryBuffer;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.YoVariableBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class SharedMemoryRandomTools
{
   public enum EnumTypeA
   {
      A1, A2, A4, A5
   }

   public enum EnumTypeB
   {
      B1, B2, B4
   }

   public enum EnumTypeC
   {
      C1, C2
   }

   public enum EnumTypeD
   {
      D1
   }

   public static YoBufferProperties nextYoBufferProperties(Random random)
   {
      return nextYoBufferProperties(random, random.nextInt(1000) + 2);
   }

   public static YoBufferProperties nextYoBufferProperties(Random random, int size)
   {
      YoBufferProperties next = new YoBufferProperties(random.nextInt(size), size);
      next.setInPointIndex(random.nextInt(size));
      next.setOutPointIndex(random.nextInt(size));
      return next;
   }

   public static YoVariableBuffer<?> nextYoVariableBuffer(Random random, YoVariable yoVariable)
   {
      YoBufferProperties properties = nextYoBufferProperties(random);
      YoVariableBuffer<?> next = YoVariableBuffer.newYoVariableBuffer(yoVariable, properties);
      randomizeYoVariableBuffer(random, next);

      return next;
   }

   public static YoBooleanBuffer nextYoBooleanBuffer(Random random, YoRegistry registry)
   {
      YoBoolean yoBoolean = nextYoBoolean(random, registry);
      YoBufferProperties properties = nextYoBufferProperties(random);
      YoBooleanBuffer next = new YoBooleanBuffer(yoBoolean, properties);
      randomizeYoVariableBuffer(random, next);

      return next;
   }

   public static YoDoubleBuffer nextYoDoubleBuffer(Random random, YoRegistry registry)
   {
      YoDouble yoDouble = nextYoDouble(random, registry);
      YoBufferProperties properties = nextYoBufferProperties(random);
      YoDoubleBuffer next = new YoDoubleBuffer(yoDouble, properties);
      randomizeYoVariableBuffer(random, next);

      return next;
   }

   public static YoIntegerBuffer nextYoIntegerBuffer(Random random, YoRegistry registry)
   {
      YoInteger yoInteger = nextYoInteger(random, registry);
      YoBufferProperties properties = nextYoBufferProperties(random);
      YoIntegerBuffer next = new YoIntegerBuffer(yoInteger, properties);
      randomizeYoVariableBuffer(random, next);

      return next;
   }

   public static YoLongBuffer nextYoLongBuffer(Random random, YoRegistry registry)
   {
      YoLong yoLong = nextYoLong(random, registry);
      YoBufferProperties properties = nextYoBufferProperties(random);
      YoLongBuffer next = new YoLongBuffer(yoLong, properties);
      randomizeYoVariableBuffer(random, next);

      return next;
   }

   public static YoEnumBuffer<?> nextYoEnumBuffer(Random random, YoRegistry registry)
   {
      YoEnum<?> yoEnum = nextYoEnum(random, registry);
      YoBufferProperties properties = nextYoBufferProperties(random);
      YoEnumBuffer<?> next = new YoEnumBuffer<>(yoEnum, properties);
      randomizeYoVariableBuffer(random, next);

      return next;
   }

   public static YoSharedBuffer nextYoSharedBuffer(Random random, int maxNumberOfVariablesPerRegistry, int numberOfRegistries)
   {
      return nextYoSharedBuffer(random, nextYoRegistryTree(random, maxNumberOfVariablesPerRegistry, numberOfRegistries)[0]);
   }

   public static YoSharedBuffer nextYoSharedBuffer(Random random, int maxNumberOfVariablesPerRegistry, int numberOfRegistries, int maxRegistryDepth)
   {
      return nextYoSharedBuffer(random, nextYoRegistryTree(random, maxNumberOfVariablesPerRegistry, numberOfRegistries)[0]);
   }

   public static YoSharedBuffer nextYoSharedBuffer(Random random, YoRegistry rootRegistry)
   {
      YoSharedBuffer next = new YoSharedBuffer(rootRegistry, random.nextInt(500) + 2);
      randomizeYoSharedBuffer(random, next);
      return next;
   }

   public static void randomizeYoSharedBuffer(Random random, YoSharedBuffer yoSharedBuffer)
   {
      randomizeYoRegistryBuffer(random, yoSharedBuffer.getRegistryBuffer());
      yoSharedBuffer.setCurrentIndex(random.nextInt(yoSharedBuffer.getProperties().getSize()));
      yoSharedBuffer.setInPoint(random.nextInt(yoSharedBuffer.getProperties().getSize()));
      yoSharedBuffer.setOutPoint(random.nextInt(yoSharedBuffer.getProperties().getSize()));
   }

   public static void randomizeYoRegistryBuffer(Random random, YoRegistryBuffer YoRegistryBuffer)
   {
      List<YoVariable> allVariables = YoRegistryBuffer.getRootRegistry().collectSubtreeVariables();

      for (int i = 0; i < YoRegistryBuffer.getProperties().getSize(); i++)
      {
         allVariables.forEach(v -> randomizeYoVariable(random, v));
         YoRegistryBuffer.writeBufferAt(i);
      }
   }

   public static void randomizeYoVariableBuffer(Random random, YoVariableBuffer<?> yoVariableBuffer)
   {
      for (int i = 0; i < yoVariableBuffer.getProperties().getSize(); i++)
      {
         randomizeYoVariable(random, yoVariableBuffer.getYoVariable());
         yoVariableBuffer.writeBufferAt(i);
      }
   }

   public static Class<? extends Enum<?>> nextEnumType(Random random)
   {
      switch (random.nextInt(4))
      {
         case 0:
            return EnumTypeA.class;
         case 1:
            return EnumTypeB.class;
         case 2:
            return EnumTypeC.class;
         case 3:
            return EnumTypeD.class;
         default:
            throw new IllegalStateException("Should not reached this state.");
      }
   }

   public static String nextString(Random random, int minLengthInclusive, int maxLengthInclusive)
   {
      int length = RandomNumbers.nextInt(random, minLengthInclusive, maxLengthInclusive);
      return RandomStringUtils.random(length, 32, 127, false, false, null, random);
   }

   public static String nextAlphanumericString(Random random, int minLengthInclusive, int maxLengthInclusive)
   {
      int length = RandomNumbers.nextInt(random, minLengthInclusive, maxLengthInclusive);
      return RandomStringUtils.random(length, 0, 0, true, true, null, random);
   }

   public static String nextAvailableVariableName(Random random, int minLengthInclusive, int maxLengthInclusive, YoRegistry registry)
   {
      String next = nextAlphanumericString(random, minLengthInclusive, maxLengthInclusive);
      while (registry.findVariable(next) != null)
         next = nextAlphanumericString(random, minLengthInclusive, maxLengthInclusive);
      return next;
   }

   public static String nextAvailableRegistryName(Random random, int minLengthInclusive, int maxLengthInclusive, YoRegistry registry)
   {
      return nextAvailableRegistryName(random, null, minLengthInclusive, maxLengthInclusive, registry);
   }

   public static String nextAvailableRegistryName(Random random, String namePrefix, int minLengthInclusive, int maxLengthInclusive, YoRegistry registry)
   {
      Set<String> unavailableNames = registry.getChildren().stream().map(reg -> reg.getName().toLowerCase()).collect(Collectors.toSet());

      {
         YoRegistry parent = registry;
         while (!parent.isRoot())
         {
            unavailableNames.add(parent.getName().toLowerCase());
            parent = parent.getParent();
         }
         unavailableNames.add(parent.getName().toLowerCase());
      }

      String next = addPrefixToName(namePrefix, nextAlphanumericString(random, minLengthInclusive, maxLengthInclusive));
      while (unavailableNames.contains(next.toLowerCase()))
         next = addPrefixToName(namePrefix, nextAlphanumericString(random, minLengthInclusive, maxLengthInclusive));
      return next;
   }

   public static YoVariable nextYoVariable(Random random, YoRegistry registry)
   {
      switch (random.nextInt(5))
      {
         case 0:
            return nextYoBoolean(random, registry);
         case 1:
            return nextYoDouble(random, registry);
         case 2:
            return nextYoInteger(random, registry);
         case 3:
            return nextYoLong(random, registry);
         case 4:
            return nextYoEnum(random, registry);
         default:
            throw new IllegalStateException("Should not reached this state.");
      }
   }

   public static YoVariable nextYoVariable(Random random, String name, YoRegistry registry)
   {
      switch (random.nextInt(5))
      {
         case 0:
            return nextYoBoolean(random, name, registry);
         case 1:
            return nextYoDouble(random, name, registry);
         case 2:
            return nextYoInteger(random, name, registry);
         case 3:
            return nextYoLong(random, name, registry);
         case 4:
            return nextYoEnum(random, name, registry);
         default:
            throw new IllegalStateException("Should not reached this state.");
      }
   }

   public static YoVariable[] nextYoVariables(Random random, int numberOfVariables, YoRegistry registry)
   {
      YoVariable[] next = new YoVariable[numberOfVariables];
      for (int i = 0; i < numberOfVariables; i++)
         next[i] = nextYoVariable(random, registry);
      return next;
   }

   public static YoVariable[] nextYoVariables(Random random, String namePrefix, int numberOfVariables, YoRegistry registry)
   {
      YoVariable[] next = new YoVariable[numberOfVariables];
      for (int i = 0; i < numberOfVariables; i++)
         next[i] = nextYoVariable(random, namePrefix + i, registry);
      return next;
   }

   public static YoBoolean nextYoBoolean(Random random, YoRegistry registry)
   {
      return nextYoBoolean(random, nextAvailableVariableName(random, 1, 50, registry), registry);
   }

   public static YoBoolean nextYoBoolean(Random random, String name, YoRegistry registry)
   {
      String description = random.nextBoolean() ? null : nextString(random, 0, 50);
      YoBoolean next = new YoBoolean(name, description, registry);
      randomizeYoBoolean(random, next);
      return next;
   }

   public static YoDouble nextYoDouble(Random random, YoRegistry registry)
   {
      return nextYoDouble(random, nextAvailableVariableName(random, 1, 50, registry), registry);
   }

   public static YoDouble nextYoDouble(Random random, String name, YoRegistry registry)
   {
      String description = random.nextBoolean() ? null : nextString(random, 0, 50);
      YoDouble next = new YoDouble(name, description, registry);
      randomizeYoDouble(random, next);
      return next;
   }

   public static YoInteger nextYoInteger(Random random, YoRegistry registry)
   {
      return nextYoInteger(random, nextAvailableVariableName(random, 1, 50, registry), registry);
   }

   public static YoInteger nextYoInteger(Random random, String name, YoRegistry registry)
   {
      String description = random.nextBoolean() ? null : nextString(random, 0, 50);
      YoInteger next = new YoInteger(name, description, registry);
      randomizeYoInteger(random, next);
      return next;
   }

   public static YoLong nextYoLong(Random random, YoRegistry registry)
   {
      return nextYoLong(random, nextAvailableVariableName(random, 1, 50, registry), registry);
   }

   public static YoLong nextYoLong(Random random, String name, YoRegistry registry)
   {
      String description = random.nextBoolean() ? null : nextString(random, 0, 50);
      YoLong next = new YoLong(name, description, registry);
      randomizeYoLong(random, next);
      return next;
   }

   public static <E extends Enum<E>> YoEnum<E> nextYoEnum(Random random, YoRegistry registry)
   {
      return nextYoEnum(random, nextAvailableVariableName(random, 1, 50, registry), registry);
   }

   public static <E extends Enum<E>> YoEnum<E> nextYoEnum(Random random, Class<E> enumType, YoRegistry registry)
   {
      return nextYoEnum(random, nextAvailableVariableName(random, 1, 50, registry), enumType, registry);
   }

   @SuppressWarnings("unchecked")
   public static <E extends Enum<E>> YoEnum<E> nextYoEnum(Random random, String name, YoRegistry registry)
   {
      return nextYoEnum(random, name, (Class<E>) nextEnumType(random), registry);
   }

   public static <E extends Enum<E>> YoEnum<E> nextYoEnum(Random random, String name, Class<E> enumType, YoRegistry registry)
   {
      String description = random.nextBoolean() ? null : nextString(random, 0, 50);
      YoEnum<E> next = new YoEnum<>(name, description, registry, enumType, random.nextBoolean());
      randomizeYoEnum(random, next);
      return next;
   }

   public static void randomizeYoVariable(Random random, YoVariable yoVariable)
   {
      if (yoVariable instanceof YoBoolean)
         randomizeYoBoolean(random, (YoBoolean) yoVariable);
      else if (yoVariable instanceof YoDouble)
         randomizeYoDouble(random, (YoDouble) yoVariable);
      else if (yoVariable instanceof YoInteger)
         randomizeYoInteger(random, (YoInteger) yoVariable);
      else if (yoVariable instanceof YoLong)
         randomizeYoLong(random, (YoLong) yoVariable);
      else if (yoVariable instanceof YoEnum)
         randomizeYoEnum(random, (YoEnum<?>) yoVariable);
   }

   public static void randomizeYoBoolean(Random random, YoBoolean yoBoolean)
   {
      yoBoolean.set(random.nextBoolean());
   }

   public static void randomizeYoDouble(Random random, YoDouble yoDouble)
   {
      yoDouble.set(EuclidCoreRandomTools.nextDouble(random, 1000.0));
   }

   public static void randomizeYoInteger(Random random, YoInteger yoInteger)
   {
      yoInteger.set(RandomNumbers.nextInt(random, -100000, 100000));
   }

   public static void randomizeYoLong(Random random, YoLong yoLong)
   {
      yoLong.set(RandomNumbers.nextInt(random, -100000, 100000));
   }

   public static void randomizeYoEnum(Random random, YoEnum<?> yoEnum)
   {
      yoEnum.set(nextOrdinal(random, yoEnum));
   }

   public static void randomizeYoVariables(Random random, YoVariable... yoVariables)
   {
      for (YoVariable yoVariable : yoVariables)
      {
         randomizeYoVariable(random, yoVariable);
      }
   }

   public static void randomizeYoVariables(Random random, Collection<? extends YoVariable> yoVariables)
   {
      for (YoVariable yoVariable : yoVariables)
      {
         randomizeYoVariable(random, yoVariable);
      }
   }

   public static int nextOrdinal(Random random, YoEnum<?> yoEnum)
   {
      if (yoEnum.isNullAllowed())
         return random.nextInt(yoEnum.getEnumSize() + 1) - 1;
      else
         return random.nextInt(yoEnum.getEnumSize());
   }

   public static YoRegistry nextYoRegistry(Random random, int numberOfVariables)
   {
      return nextYoRegistry(random, nextAlphanumericString(random, 1, 50), numberOfVariables);
   }

   public static YoRegistry nextYoRegistry(Random random, String name, int numberOfVariables)
   {
      YoRegistry next = new YoRegistry(name);
      nextYoVariables(random, name + "_var", numberOfVariables, next);
      return next;
   }

   public static YoRegistry[] nextYoRegistryChain(Random random, int maxNumberOfVariablesPerRegistry, int numberOfRegistries)
   {
      return nextYoRegistryChain(random, null, maxNumberOfVariablesPerRegistry, numberOfRegistries);
   }

   public static YoRegistry[] nextYoRegistryChain(Random random, String namePrefix, int maxNumberOfVariablesPerRegistry, int numberOfRegistries)
   {
      YoRegistry root = nextYoRegistry(random,
                                       addPrefixToName(namePrefix, nextAlphanumericString(random, 1, 50)),
                                       random.nextInt(maxNumberOfVariablesPerRegistry + 1));
      YoRegistry[] registries = new YoRegistry[numberOfRegistries];
      registries[0] = root;

      for (int i = 1; i < numberOfRegistries; i++)
      {
         YoRegistry parentRegistry = registries[i - 1];
         YoRegistry next = nextYoRegistry(random,
                                          nextAvailableRegistryName(random, namePrefix, 1, 50, parentRegistry),
                                          random.nextInt(maxNumberOfVariablesPerRegistry + 1));
         registries[i] = next;
         parentRegistry.addChild(next);
      }

      return registries;
   }

   public static YoRegistry[] nextYoRegistryTree(Random random, int maxNumberOfVariablesPerRegistry, int numberOfRegistries)
   {
      return nextYoRegistryTree(random, (String) null, maxNumberOfVariablesPerRegistry, numberOfRegistries);
   }

   public static YoRegistry[] nextYoRegistryTree(Random random, String namePrefix, int maxNumberOfVariablesPerRegistry, int numberOfRegistries)
   {
      return nextYoRegistryTree(random,
                                nextYoRegistry(random,
                                               addPrefixToName(namePrefix, nextAlphanumericString(random, 1, 50)),
                                               random.nextInt(maxNumberOfVariablesPerRegistry + 1)),
                                maxNumberOfVariablesPerRegistry,
                                numberOfRegistries);
   }

   public static YoRegistry[] nextYoRegistryTree(Random random, YoRegistry rootRegistry, int maxNumberOfVariablesPerRegistry, int numberOfRegistries)
   {
      return nextYoRegistryTree(random, rootRegistry, null, maxNumberOfVariablesPerRegistry, numberOfRegistries);
   }

   public static YoRegistry[] nextYoRegistryTree(Random random,
                                                 YoRegistry rootRegistry,
                                                 String namePrefix,
                                                 int maxNumberOfVariablesPerRegistry,
                                                 int numberOfRegistries)
   {
      YoRegistry[] registries = new YoRegistry[numberOfRegistries];
      registries[0] = rootRegistry;

      for (int i = 1; i < numberOfRegistries; i++)
      {
         YoRegistry parentRegistry = registries[random.nextInt(i)];
         YoRegistry next = nextYoRegistry(random,
                                          nextAvailableRegistryName(random, namePrefix, 1, 50, parentRegistry),
                                          random.nextInt(maxNumberOfVariablesPerRegistry + 1));
         registries[i] = next;
         parentRegistry.addChild(next);
      }

      return registries;
   }

   public static String addPrefixToName(String namePrefix, String name)
   {
      if (namePrefix != null && !namePrefix.isEmpty())
         return namePrefix + name;
      else
         return name;
   }
}
