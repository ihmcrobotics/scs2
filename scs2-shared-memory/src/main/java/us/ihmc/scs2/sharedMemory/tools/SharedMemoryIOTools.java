package us.ihmc.scs2.sharedMemory.tools;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.IntConsumer;
import java.util.function.ObjIntConsumer;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;

import org.apache.commons.lang3.mutable.MutableInt;

import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.format.Mat5File;
import us.hebi.matlab.mat.types.Array;
import us.hebi.matlab.mat.types.MatFile;
import us.hebi.matlab.mat.types.MatFile.Entry;
import us.hebi.matlab.mat.types.Matrix;
import us.hebi.matlab.mat.types.Struct;
import us.ihmc.scs2.definition.yoVariable.YoBooleanDefinition;
import us.ihmc.scs2.definition.yoVariable.YoDoubleDefinition;
import us.ihmc.scs2.definition.yoVariable.YoEnumDefinition;
import us.ihmc.scs2.definition.yoVariable.YoIntegerDefinition;
import us.ihmc.scs2.definition.yoVariable.YoLongDefinition;
import us.ihmc.scs2.definition.yoVariable.YoRegistryDefinition;
import us.ihmc.scs2.definition.yoVariable.YoVariableDefinition;
import us.ihmc.scs2.sharedMemory.YoBooleanBuffer;
import us.ihmc.scs2.sharedMemory.YoDoubleBuffer;
import us.ihmc.scs2.sharedMemory.YoEnumBuffer;
import us.ihmc.scs2.sharedMemory.YoIntegerBuffer;
import us.ihmc.scs2.sharedMemory.YoLongBuffer;
import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.scs2.sharedMemory.YoVariableBuffer;
import us.ihmc.scs2.sharedMemory.interfaces.YoBufferPropertiesReadOnly;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class SharedMemoryIOTools
{
   public static final int MATLAB_VARNAME_MAX_LENGTH = 63;
   public static final int MATLAB_VARNAME_OVERFLOW_SUFFIX_CHARS = 3;

   public enum DataFormat
   {
      ASCII(".scs2.ascii"), CSV(".scs2.csv"), MATLAB(".scs2.mat");

      private final String fileExtension;

      DataFormat(String fileExtension)
      {
         this.fileExtension = fileExtension;
      }

      public String getFileExtension()
      {
         return fileExtension;
      }

      public static DataFormat fromFilename(String filename)
      {
         if (filename == null)
            return null;
         for (DataFormat dataFormat : values())
         {
            if (filename.endsWith(dataFormat.getFileExtension()))
               return dataFormat;
         }
         return null;
      }
   }

   private static JAXBContext yoRegistryContext = null;

   public static JAXBContext getYoRegistryContext()
   {
      if (yoRegistryContext == null)
      {
         try
         {
            yoRegistryContext = JAXBContext.newInstance(YoRegistryDefinition.class,
                                                        YoVariableDefinition.class,
                                                        YoBooleanDefinition.class,
                                                        YoDoubleDefinition.class,
                                                        YoIntegerDefinition.class,
                                                        YoLongDefinition.class,
                                                        YoEnumDefinition.class);
         }
         catch (JAXBException e)
         {
            throw new RuntimeException("Problem creating the JAXBContext.", e);
         }
      }

      return yoRegistryContext;
   }

   public static YoRegistryDefinition loadYoRegistryDefinition(InputStream inputStream) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = getYoRegistryContext();
         Unmarshaller unmarshaller = context.createUnmarshaller();
         return (YoRegistryDefinition) unmarshaller.unmarshal(inputStream);
      }
      finally
      {
         inputStream.close();
      }
   }

   public static void saveYoRegistryDefinition(OutputStream outputStream, YoRegistryDefinition definition) throws JAXBException, IOException
   {
      try
      {
         JAXBContext context = getYoRegistryContext();
         Marshaller marshaller = context.createMarshaller();
         marshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, Boolean.TRUE);
         marshaller.marshal(definition, outputStream);
      }
      finally
      {
         outputStream.close();
      }
   }

   public static YoRegistryDefinition toYoRegistryDefinition(YoRegistry yoRegistry)
   {
      return toYoRegistryDefinition(yoRegistry, null, null);
   }

   public static YoRegistryDefinition toYoRegistryDefinition(YoRegistry yoRegistry, Predicate<YoVariable> variableFilter, Predicate<YoRegistry> registryFilter)
   {
      YoRegistryDefinition definition = new YoRegistryDefinition();
      definition.setName(yoRegistry.getName());

      Stream<YoVariable> variablesStream = yoRegistry.getVariables().stream();
      if (variableFilter != null)
         variablesStream = variablesStream.filter(variableFilter);
      definition.setYoVariables(variablesStream.map(SharedMemoryIOTools::toYoVariableDefinition).collect(Collectors.toList()));

      Stream<YoRegistry> childrenStream = yoRegistry.getChildren().stream();
      if (registryFilter != null)
         childrenStream = childrenStream.filter(registryFilter);
      definition.setYoRegistries(childrenStream.map(childRegistry -> toYoRegistryDefinition(childRegistry, variableFilter, registryFilter)).collect(Collectors.toList()));

      return definition;
   }

   public static YoVariableDefinition toYoVariableDefinition(YoVariable yoVariable)
   {
      YoVariableDefinition definition = null;

      if (yoVariable instanceof YoBoolean)
      {
         definition = new YoBooleanDefinition();
      }
      else if (yoVariable instanceof YoDouble)
      {
         definition = new YoDoubleDefinition();
      }
      else if (yoVariable instanceof YoInteger)
      {
         definition = new YoIntegerDefinition();
      }
      else if (yoVariable instanceof YoLong)
      {
         definition = new YoLongDefinition();
      }
      else if (yoVariable instanceof YoEnum<?>)
      {
         YoEnumDefinition enumDefinition = new YoEnumDefinition();
         enumDefinition.setAllowNullValue(((YoEnum<?>) yoVariable).isNullAllowed());
         enumDefinition.setEnumValuesAsString(((YoEnum<?>) yoVariable).getEnumValuesAsString());
         definition = enumDefinition;
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported yoVariable type: " + yoVariable);
      }

      definition.setName(yoVariable.getName());
      definition.setDescription(yoVariable.getDescription());
      definition.setLowerBound(yoVariable.getLowerBound());
      definition.setUpperBound(yoVariable.getUpperBound());
      return definition;
   }

   public static YoRegistry toYoRegistry(YoRegistryDefinition definition)
   {
      YoRegistry yoRegistry = new YoRegistry(definition.getName());
      if (definition.getYoVariables() != null)
         definition.getYoVariables().forEach(varDefinition -> yoRegistry.addVariable(toYoVariable(varDefinition)));
      if (definition.getYoRegistries() != null)
         definition.getYoRegistries().forEach(regDefinition -> yoRegistry.addChild(toYoRegistry(regDefinition)));
      return yoRegistry;
   }

   public static YoVariable toYoVariable(YoVariableDefinition definition)
   {
      YoVariable yoVariable = null;

      String name = definition.getName();
      String description = definition.getDescription();

      if (definition instanceof YoBooleanDefinition)
      {
         yoVariable = new YoBoolean(name, description, null);
      }
      else if (definition instanceof YoDoubleDefinition)
      {
         yoVariable = new YoDouble(name, description, null);
      }
      else if (definition instanceof YoIntegerDefinition)
      {
         yoVariable = new YoInteger(name, description, null);
      }
      else if (definition instanceof YoLongDefinition)
      {
         yoVariable = new YoLong(name, description, null);
      }
      else if (definition instanceof YoEnumDefinition)
      {
         yoVariable = new YoEnum<>(name,
                                   description,
                                   null,
                                   ((YoEnumDefinition) definition).isAllowNullValue(),
                                   ((YoEnumDefinition) definition).getEnumValuesAsString());
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported definition type: " + definition);
      }

      yoVariable.setVariableBounds(definition.getLowerBound(), definition.getUpperBound());
      return yoVariable;
   }

   public static void exportRegistry(YoRegistry rootRegistry, OutputStream outputStream) throws JAXBException, IOException
   {
      exportRegistry(rootRegistry, outputStream, null, null);
   }

   public static void exportRegistry(YoRegistry rootRegistry,
                                     OutputStream outputStream,
                                     Predicate<YoVariable> variableFilter,
                                     Predicate<YoRegistry> registryFilter)
         throws JAXBException, IOException
   {
      saveYoRegistryDefinition(outputStream, toYoRegistryDefinition(rootRegistry, variableFilter, registryFilter));
   }

   public static void exportDataASCII(YoSharedBuffer buffer, OutputStream dataOutputStream)
   {
      exportDataASCII(buffer, dataOutputStream, null, null);
   }

   public static void exportDataASCII(YoSharedBuffer buffer,
                                      OutputStream outputStream,
                                      Predicate<YoVariable> variableFilter,
                                      Predicate<YoRegistry> registryFilter)
   {
      Stream<YoVariableBuffer<?>> yoVariableBufferStream;

      if (registryFilter != null)
      {
         List<YoRegistry> filteredRegistries = YoSearchTools.filterRegistries(registryFilter, buffer.getRootRegistry());
         yoVariableBufferStream = filteredRegistries.stream()
                                                    .flatMap(registry -> registry.getVariables().stream())
                                                    .map(yoVariable -> buffer.getRegistryBuffer().findYoVariableBuffer(yoVariable));
      }
      else
      {
         yoVariableBufferStream = buffer.getRegistryBuffer().getYoVariableBuffers().stream();
      }

      if (variableFilter != null)
      {
         yoVariableBufferStream = yoVariableBufferStream.filter(yoVariableBuffer -> variableFilter.test(yoVariableBuffer.getYoVariable()));
      }

      PrintStream printStream = new PrintStream(outputStream);
      YoBufferPropertiesReadOnly properties = buffer.getProperties();
      int inPoint = properties.getInPoint();
      int activeBufferLength = properties.getActiveBufferLength();

      yoVariableBufferStream.forEach(yoVariableBuffer ->
      {
         String variableName = yoVariableBuffer.getYoVariable().getFullNameString();
         printStream.append(variableName).append(": ");
         printStream.println(arrayToString(yoVariableBuffer.copy(inPoint, activeBufferLength, properties).getSample()));
      });

      printStream.close();
   }

   public static void exportDataCSV(YoSharedBuffer buffer, OutputStream dataOutputStream)
   {
      exportDataCSV(buffer, dataOutputStream, null, null);
   }

   public static void exportDataCSV(YoSharedBuffer buffer,
                                    OutputStream outputStream,
                                    Predicate<YoVariable> variableFilter,
                                    Predicate<YoRegistry> registryFilter)
   {
      Stream<YoVariableBuffer<?>> yoVariableBufferStream;

      if (registryFilter != null)
      {
         List<YoRegistry> filteredRegistries = YoSearchTools.filterRegistries(registryFilter, buffer.getRootRegistry());
         yoVariableBufferStream = filteredRegistries.stream()
                                                    .flatMap(registry -> registry.getVariables().stream())
                                                    .map(yoVariable -> buffer.getRegistryBuffer().findYoVariableBuffer(yoVariable));
      }
      else
      {
         yoVariableBufferStream = buffer.getRegistryBuffer().getYoVariableBuffers().stream();
      }

      if (variableFilter != null)
      {
         yoVariableBufferStream = yoVariableBufferStream.filter(yoVariableBuffer -> variableFilter.test(yoVariableBuffer.getYoVariable()));
      }

      PrintStream printStream = new PrintStream(outputStream);

      YoVariableBuffer<?>[] yoVariableBuffersToExport = yoVariableBufferStream.toArray(YoVariableBuffer[]::new);

      for (int bufferIndex = 0; bufferIndex < yoVariableBuffersToExport.length; bufferIndex++)
      {
         printStream.print(yoVariableBuffersToExport[bufferIndex].getYoVariable().getFullNameString());
         if (bufferIndex < yoVariableBuffersToExport.length - 1)
            printStream.print(", ");
         else
            printStream.println();
      }

      YoBufferPropertiesReadOnly properties = buffer.getProperties();
      int readPosition = properties.getInPoint();

      IntConsumer[] bufferValueWriters = Stream.of(yoVariableBuffersToExport).map(yoVariableBuffer ->
      {
         Object internalBuffer = yoVariableBuffer.getBuffer();

         if (yoVariableBuffer instanceof YoBooleanBuffer)
         {
            boolean[] booleanBuffer = (boolean[]) internalBuffer;
            return (IntConsumer) position -> printStream.print(booleanBuffer[position]);
         }
         if (yoVariableBuffer instanceof YoDoubleBuffer)
         {
            double[] doubleBuffer = (double[]) internalBuffer;
            return (IntConsumer) position -> printStream.print(doubleBuffer[position]);
         }
         if (yoVariableBuffer instanceof YoIntegerBuffer)
         {
            int[] intBuffer = (int[]) internalBuffer;
            return (IntConsumer) position -> printStream.print(intBuffer[position]);
         }
         if (yoVariableBuffer instanceof YoLongBuffer)
         {
            long[] longBuffer = (long[]) internalBuffer;
            return (IntConsumer) position -> printStream.print(longBuffer[position]);
         }
         if (yoVariableBuffer instanceof YoEnumBuffer)
         {
            byte[] byteBuffer = (byte[]) internalBuffer;
            String[] enumConstants = ((YoEnum<?>) yoVariableBuffer.getYoVariable()).getEnumValuesAsString();
            return (IntConsumer) position -> printStream.print(byteBuffer[position] == YoEnum.NULL_VALUE ? YoEnum.NULL_VALUE_STRING
                                                                                                         : enumConstants[byteBuffer[position]]);
         }

         throw new IllegalArgumentException("Unhandled buffer type: " + yoVariableBuffer);
      }).toArray(IntConsumer[]::new);

      for (int i = 0; i < properties.getActiveBufferLength(); i++)
      {
         for (int bufferIndex = 0; bufferIndex < yoVariableBuffersToExport.length; bufferIndex++)
         {
            bufferValueWriters[bufferIndex].accept(readPosition);
            if (bufferIndex < yoVariableBuffersToExport.length - 1)
               printStream.print(", ");
            else
               printStream.println();
         }

         readPosition = SharedMemoryTools.increment(readPosition, 1, properties.getSize());
      }

      printStream.close();
   }

   public static void exportDataMatlab(YoSharedBuffer buffer, File outputFile) throws IOException
   {
      exportDataMatlab(buffer, outputFile, null, null);
   }

   @SuppressWarnings("resource")
   public static void exportDataMatlab(YoSharedBuffer buffer, File outputFile, Predicate<YoVariable> variableFilter, Predicate<YoRegistry> registryFilter)
         throws IOException
   {
      Stream<YoVariableBuffer<?>> yoVariableBufferStream;
      YoRegistry rootRegistry = buffer.getRootRegistry();

      if (registryFilter != null)
      {
         List<YoRegistry> filteredRegistries = YoSearchTools.filterRegistries(registryFilter, rootRegistry);
         yoVariableBufferStream = filteredRegistries.stream()
                                                    .flatMap(registry -> registry.getVariables().stream())
                                                    .map(yoVariable -> buffer.getRegistryBuffer().findYoVariableBuffer(yoVariable));
      }
      else
      {
         yoVariableBufferStream = buffer.getRegistryBuffer().getYoVariableBuffers().stream();
      }

      if (variableFilter != null)
      {
         yoVariableBufferStream = yoVariableBufferStream.filter(yoVariableBuffer -> variableFilter.test(yoVariableBuffer.getYoVariable()));
      }

      MatFile matFile = Mat5.newMatFile();
      Struct rootStruct = Mat5.newStruct();
      matFile.addArray(rootRegistry.getName(), rootStruct);

      Struct nameHelperStruct = Mat5.newStruct();
      matFile.addArray("NameOverflow", nameHelperStruct);
      Map<String, MutableInt> nameOverflowCounter = new HashMap<>();

      yoVariableBufferStream.forEach(yoVariableBuffer ->
      {
         YoBufferPropertiesReadOnly properties = yoVariableBuffer.getProperties();

         Struct parentStruct = rootStruct;

         YoVariable yoVariable = yoVariableBuffer.getYoVariable();

         if (yoVariable.getRegistry() != rootRegistry)
         {
            YoNamespace parentNamespace = yoVariable.getNamespace();

            for (int i = 1; i < parentNamespace.size(); i++)
            {
               String subName = parentNamespace.getSubNames().get(i);
               Struct childStruct;

               try
               {
                  childStruct = parentStruct.getStruct(subName);
               }
               catch (IllegalArgumentException e)
               {
                  childStruct = Mat5.newStruct();
                  String registryStructName = checkAndRegisterLongName(subName, nameOverflowCounter, nameHelperStruct);
                  parentStruct.set(registryStructName, childStruct);
               }

               parentStruct = childStruct;
            }
         }

         String variableStructName = checkAndRegisterLongName(yoVariableBuffer.getYoVariable().getName(), nameOverflowCounter, nameHelperStruct);
         Matrix matMatrix = Mat5.newMatrix(properties.getActiveBufferLength(), 1);
         writeBuffer(yoVariableBuffer.getBuffer(), matMatrix, properties.getInPoint(), properties.getActiveBufferLength());
         parentStruct.set(variableStructName, matMatrix);
      });

      Mat5.writeToFile(matFile, outputFile);
      rootStruct.close();
   }

   private static String checkAndRegisterLongName(String name, Map<String, MutableInt> nameOverflowCounter, Struct nameHelperStruct)
   {
      if (name.length() <= MATLAB_VARNAME_MAX_LENGTH)
         return name;

      String shortenedName = name.substring(0, MATLAB_VARNAME_MAX_LENGTH - MATLAB_VARNAME_OVERFLOW_SUFFIX_CHARS);
      int idSuffix = nameOverflowCounter.computeIfAbsent(name, n -> new MutableInt(0)).getAndIncrement();
      String matlabFieldName = shortenedName + idSuffix;

      if (matlabFieldName.length() > MATLAB_VARNAME_MAX_LENGTH)
         throw new RuntimeException("Too many instances of shortened variable name " + shortenedName);

      Matrix nameMatrix = Mat5.newMatrix(name.length(), 1);
      for (int i = 0; i < name.length(); i++)
         nameMatrix.setInt(i, name.charAt(i));
      nameHelperStruct.set(matlabFieldName, nameMatrix);
      return matlabFieldName;
   }

   private static void writeBuffer(Object buffer, Matrix matrix, int start, int length)
   {
      int readingPosition = start;

      if (buffer instanceof boolean[])
      {
         boolean[] booleanBuffer = (boolean[]) buffer;
         for (int i = 0; i < length; i++)
         {
            matrix.setBoolean(i, booleanBuffer[readingPosition]);
            readingPosition = SharedMemoryTools.increment(readingPosition, 1, booleanBuffer.length);
         }
      }
      else if (buffer instanceof double[])
      {
         double[] doubleBuffer = (double[]) buffer;
         for (int i = 0; i < length; i++)
         {
            matrix.setDouble(i, doubleBuffer[readingPosition]);
            readingPosition = SharedMemoryTools.increment(readingPosition, 1, doubleBuffer.length);
         }
      }
      else if (buffer instanceof int[])
      {
         int[] intBuffer = (int[]) buffer;
         for (int i = 0; i < length; i++)
         {
            matrix.setInt(i, intBuffer[readingPosition]);
            readingPosition = SharedMemoryTools.increment(readingPosition, 1, intBuffer.length);
         }
      }
      else if (buffer instanceof long[])
      {
         long[] longBuffer = (long[]) buffer;
         for (int i = 0; i < length; i++)
         {
            matrix.setLong(i, longBuffer[readingPosition]);
            readingPosition = SharedMemoryTools.increment(readingPosition, 1, longBuffer.length);
         }
      }
      else if (buffer instanceof byte[])
      {
         byte[] byteBuffer = (byte[]) buffer;
         for (int i = 0; i < length; i++)
         {
            matrix.setByte(i, byteBuffer[readingPosition]);
            readingPosition = SharedMemoryTools.increment(readingPosition, 1, byteBuffer.length);
         }
      }
      else
      {
         throw new IllegalArgumentException("Unsupported buffer type: " + buffer);
      }
   }

   private static String arrayToString(Object array)
   {
      if (array instanceof boolean[])
         return Arrays.toString((boolean[]) array);
      if (array instanceof double[])
         return Arrays.toString((double[]) array);
      if (array instanceof int[])
         return Arrays.toString((int[]) array);
      if (array instanceof long[])
         return Arrays.toString((long[]) array);
      if (array instanceof byte[])
         return Arrays.toString((byte[]) array);
      throw new IllegalArgumentException("Unsupported type: " + array);
   }

   public static YoRegistry importRegistry(InputStream inputStream) throws JAXBException, IOException
   {
      return toYoRegistry(loadYoRegistryDefinition(inputStream));
   }

   public static YoSharedBuffer importDataASCII(InputStream inputStream, YoRegistry root) throws IOException
   {
      return importDataASCII(inputStream, new YoSharedBuffer(root, 1));
   }

   public static YoSharedBuffer importDataASCII(InputStream inputStream, YoSharedBuffer buffer) throws IOException
   {
      YoRegistry root = buffer.getRootRegistry();
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream));

      bufferedReader.lines().forEach(line ->
      {
         int seperatorIndex = line.indexOf(":");
         String variableName = line.substring(0, seperatorIndex).trim();

         String data = line.substring(seperatorIndex, line.length());
         YoVariable yoVariable = root.findVariable(variableName);
         YoVariableBuffer<?> yoVariableBuffer = buffer.getRegistryBuffer().findYoVariableBuffer(yoVariable);

         data = data.substring(data.indexOf("[") + 1, data.lastIndexOf("]"));
         String[] values = data.split(", ");
         setActiveBuffer(buffer, values.length);

         if (yoVariable instanceof YoBoolean)
         {
            for (int i = 0; i < values.length; i++)
               ((boolean[]) yoVariableBuffer.getBuffer())[i] = Boolean.parseBoolean(values[i]);
         }
         else if (yoVariable instanceof YoDouble)
         {
            for (int i = 0; i < values.length; i++)
               ((double[]) yoVariableBuffer.getBuffer())[i] = Double.parseDouble(values[i]);
         }
         else if (yoVariable instanceof YoInteger)
         {
            for (int i = 0; i < values.length; i++)
               ((int[]) yoVariableBuffer.getBuffer())[i] = Integer.parseInt(values[i]);
         }
         else if (yoVariable instanceof YoLong)
         {
            for (int i = 0; i < values.length; i++)
               ((long[]) yoVariableBuffer.getBuffer())[i] = Long.parseLong(values[i]);
         }
         else if (yoVariable instanceof YoEnum<?>)
         {
            for (int i = 0; i < values.length; i++)
               ((byte[]) yoVariableBuffer.getBuffer())[i] = Byte.parseByte(values[i]);
         }
         else
         {
            throw new IllegalArgumentException("Unexpected value: " + yoVariable);
         }
      });

      bufferedReader.close();

      return buffer;
   }

   public static YoSharedBuffer importDataCSV(InputStream inputStream, YoRegistry root) throws IOException
   {
      return importDataCSV(inputStream, new YoSharedBuffer(root, 1));
   }

   public static YoSharedBuffer importDataCSV(InputStream inputStream, YoSharedBuffer buffer) throws IOException
   {
      YoRegistry root = buffer.getRootRegistry();
      BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream));

      String currentLine = bufferedReader.readLine();

      int currentPosition = 0;
      String[] variableNames = currentLine.split(", ");

      @SuppressWarnings("unchecked")
      ObjIntConsumer<String>[] bufferValueReaders = Stream.of(variableNames).map(variableName ->
      {
         YoVariable yoVariable = root.findVariable(variableName);
         YoVariableBuffer<?> yoVariableBuffer = buffer.getRegistryBuffer().findYoVariableBuffer(yoVariable);

         if (yoVariableBuffer instanceof YoBooleanBuffer)
         {
            return (ObjIntConsumer<String>) (value, position) -> ((boolean[]) yoVariableBuffer.getBuffer())[position] = Boolean.parseBoolean(value);
         }
         if (yoVariableBuffer instanceof YoDoubleBuffer)
         {
            return (ObjIntConsumer<String>) (value, position) -> ((double[]) yoVariableBuffer.getBuffer())[position] = Double.parseDouble(value);
         }
         if (yoVariableBuffer instanceof YoIntegerBuffer)
         {
            return (ObjIntConsumer<String>) (value, position) -> ((int[]) yoVariableBuffer.getBuffer())[position] = Integer.parseInt(value);
         }
         if (yoVariableBuffer instanceof YoLongBuffer)
         {
            return (ObjIntConsumer<String>) (value, position) -> ((long[]) yoVariableBuffer.getBuffer())[position] = Long.parseLong(value);
         }
         if (yoVariableBuffer instanceof YoEnumBuffer)
         {
            List<String> enumConstants = Arrays.asList(((YoEnum<?>) yoVariable).getEnumValuesAsString());
            return (ObjIntConsumer<String>) (value, position) ->
            {
               if (Objects.equals(value, YoEnum.NULL_VALUE_STRING))
                  ((byte[]) yoVariableBuffer.getBuffer())[position] = YoEnum.NULL_VALUE;
               else
                  ((byte[]) yoVariableBuffer.getBuffer())[position] = (byte) enumConstants.indexOf(value);
            };
         }

         throw new IllegalArgumentException("Unhandled buffer type: " + yoVariableBuffer);

      }).toArray(ObjIntConsumer[]::new);

      if (buffer.getProperties().getSize() < 64)
         buffer.resizeBuffer(64);

      while ((currentLine = bufferedReader.readLine()) != null)
      {
         if (currentPosition >= buffer.getProperties().getSize())
            buffer.resizeBuffer(2 * buffer.getProperties().getSize());

         String[] currentValues = currentLine.split(", ");
         for (int bufferIndex = 0; bufferIndex < bufferValueReaders.length; bufferIndex++)
         {
            bufferValueReaders[bufferIndex].accept(currentValues[bufferIndex], currentPosition);
         }

         currentPosition++;
      }

      buffer.setInPoint(0);
      buffer.setOutPoint(currentPosition - 1);
      bufferedReader.close();

      return buffer;
   }

   public static YoSharedBuffer importDataMatlab(File inputFile, YoRegistry root) throws IOException
   {
      return importDataMatlab(inputFile, new YoSharedBuffer(root, 1));
   }

   public static YoSharedBuffer importDataMatlab(File inputFile, YoSharedBuffer buffer) throws IOException
   {
      YoRegistry root = buffer.getRootRegistry();
      Mat5File mat5File = Mat5.readFromFile(inputFile);

      List<Entry> entries = new ArrayList<>();

      for (Entry entry : mat5File.getEntries())
         entries.add(entry);

      if (entries.size() == 0)
         throw new IllegalArgumentException("Empty data structure");
      if (entries.size() > 2)
         throw new IllegalArgumentException("Unexpected number of structs");

      Entry entry = entries.get(0);
      if (!entry.getName().equals(root.getName()))
         throw new IllegalArgumentException("Registry name mismatch");
      Struct rootStruct = (Struct) entry.getValue();

      Struct nameHelperStruct = entries.size() == 2 ? (Struct) entries.get(1).getValue() : null;
      importMatlabStruct(rootStruct, root, buffer, nameHelperStruct);

      return buffer;
   }

   private static void importMatlabStruct(Struct matlabStruct, YoRegistry parentRegistry, YoSharedBuffer buffer, Struct nameHelperStruct)
   {
      for (String rawFieldName : matlabStruct.getFieldNames())
      {
         String fieldName = checkAndRetrieveLongName(rawFieldName, nameHelperStruct);
         Array field = matlabStruct.get(rawFieldName);

         if (field instanceof Matrix)
         {
            YoVariable variable = parentRegistry.getVariable(fieldName);
            if (variable == null)
               throw new IllegalArgumentException("Could not find the variable " + fieldName + " in " + parentRegistry);
            importMatlabMatrix(variable, (Matrix) field, buffer);
         }
         else if (field instanceof Struct)
         {
            // TODO Should be replaced with YoRegistry.getChild(String) when available.
            YoRegistry registry = parentRegistry.findRegistry(new YoNamespace(fieldName));
            importMatlabStruct((Struct) field, registry, buffer, nameHelperStruct);
         }
      }
   }

   private static String checkAndRetrieveLongName(String fieldName, Struct nameHelperStruct)
   {
      if (nameHelperStruct == null)
         return fieldName;

      try
      {
         Matrix matrix = nameHelperStruct.getMatrix(fieldName);
         StringBuilder fullName = new StringBuilder();
         for (int i = 0; i < matrix.getNumRows(); i++)
            fullName.append((char) matrix.getInt(i));
         return fullName.toString();
      }
      catch (IllegalArgumentException e)
      {
         return fieldName;
      }
   }

   private static void importMatlabMatrix(YoVariable variable, Matrix matlabMatrix, YoSharedBuffer buffer)
   {
      int size = matlabMatrix.getNumRows();
      setActiveBuffer(buffer, size);

      if (variable instanceof YoBoolean)
      {
         YoBooleanBuffer variableBuffer = (YoBooleanBuffer) buffer.getRegistryBuffer().findYoVariableBuffer(variable);

         for (int i = 0; i < size; i++)
            variableBuffer.getBuffer()[i] = matlabMatrix.getBoolean(i);
      }
      else if (variable instanceof YoDouble)
      {
         YoDoubleBuffer variableBuffer = (YoDoubleBuffer) buffer.getRegistryBuffer().findYoVariableBuffer(variable);

         for (int i = 0; i < size; i++)
            variableBuffer.getBuffer()[i] = matlabMatrix.getDouble(i);
      }
      else if (variable instanceof YoInteger)
      {
         YoIntegerBuffer variableBuffer = (YoIntegerBuffer) buffer.getRegistryBuffer().findYoVariableBuffer(variable);

         for (int i = 0; i < size; i++)
            variableBuffer.getBuffer()[i] = matlabMatrix.getInt(i);
      }
      else if (variable instanceof YoLong)
      {
         YoLongBuffer variableBuffer = (YoLongBuffer) buffer.getRegistryBuffer().findYoVariableBuffer(variable);

         for (int i = 0; i < size; i++)
            variableBuffer.getBuffer()[i] = matlabMatrix.getLong(i);
      }
      else if (variable instanceof YoEnum<?>)
      {
         YoEnumBuffer<?> variableBuffer = (YoEnumBuffer<?>) buffer.getRegistryBuffer().findYoVariableBuffer(variable);

         for (int i = 0; i < size; i++)
            variableBuffer.getBuffer()[i] = matlabMatrix.getByte(i);
      }
      else
      {
         throw new IllegalStateException("Unexpected variable type.");
      }
   }

   private static void setActiveBuffer(YoSharedBuffer buffer, int newSize)
   {
      if (newSize > buffer.getProperties().getSize())
      {
         buffer.resizeBuffer(newSize);
      }

      buffer.setInPoint(0);
      buffer.setOutPoint(newSize - 1);
   }

   public static void main(String[] args)
   {
      String a = "3289";
      String b = "thisIsAWord";

      Integer.parseInt(a);
      Integer.parseInt(b);
   }
}
