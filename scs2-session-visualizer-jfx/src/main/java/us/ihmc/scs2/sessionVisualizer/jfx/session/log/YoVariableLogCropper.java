package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;
import java.util.function.LongConsumer;
import java.util.function.Predicate;

import gnu.trove.list.array.TIntArrayList;
import us.hebi.matlab.mat.format.Mat5;
import us.hebi.matlab.mat.types.MatFile;
import us.hebi.matlab.mat.types.Matrix;
import us.hebi.matlab.mat.types.Struct;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;
import us.ihmc.scs2.session.log.ProgressConsumer;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableLogCropper extends YoVariableLogReader
{
   private final MultiVideoDataReader multiVideoDataReader;

   public YoVariableLogCropper(MultiVideoDataReader multiVideoDataReader, File logDirectory, LogProperties logProperties)
   {
      super(logDirectory, logProperties);
      this.multiVideoDataReader = multiVideoDataReader;
   }

   public void crop(File destination, int from, int to, ProgressConsumer progressConsumer)
   {
      progressConsumer.started("Cropping data file");
      progressConsumer.info("Initializing cropper");
      progressConsumer.progress(0.0);

      if (!initialize())
      {
         return;
      }

      try
      {
         progressConsumer.info("Creating directories");
         progressConsumer.progress(0.03);

         if (destination.exists())
         {
            if (!destination.isDirectory())
            {
               progressConsumer.error("Destination " + destination.getAbsolutePath() + " already exists.");
               progressConsumer.done();
               return;
            }
            else if (destination.list().length > 0)
            {
               progressConsumer.error("Destination " + destination.getAbsolutePath() + " is not empty.");
               progressConsumer.done();
               return;
            }
         }
         else if (!destination.mkdir())
         {
            progressConsumer.error("Cannot make directory " + destination.getAbsolutePath());
            progressConsumer.done();
            return;
         }

         progressConsumer.info("Copying description files");
         progressConsumer.progress(0.04);
         copyMetaData(destination);

         progressConsumer.info("Seeking variable data");
         progressConsumer.progress(0.10);

         File outputFile = new File(destination, logProperties.getVariables().getDataAsString());
         FileOutputStream fileOutputStream = new FileOutputStream(outputFile);
         FileChannel outputChannel = fileOutputStream.getChannel();

         File indexFile = new File(destination, logProperties.getVariables().getIndexAsString());
         FileOutputStream indexStream = new FileOutputStream(indexFile);
         FileChannel indexChannel = indexStream.getChannel();

         progressConsumer.info("Writing variable data %d/%d".formatted(0, to - from));

         ProgressConsumer dataCopyingProgress;
         if (multiVideoDataReader == null || multiVideoDataReader.getNumberOfVideos() == 0)
            dataCopyingProgress = progressConsumer.subProgress(0.10, 1.00);
         else
            dataCopyingProgress = progressConsumer.subProgress(0.10, 0.50);

         ByteBuffer indexBuffer = ByteBuffer.allocateDirect(16);
         for (int i = from; i <= to; i++)
         {
            progressConsumer.info("Writing variable data %d/%d".formatted(i - from, to - from));
            dataCopyingProgress.progress((double) (i - from) / (double) (to - from));
            ByteBuffer compressedData = readCompressedData(i);

            indexBuffer.clear();
            indexBuffer.putLong(getTimestamp(i));
            indexBuffer.putLong(outputChannel.position());
            indexBuffer.flip();
            indexChannel.write(indexBuffer);

            outputChannel.write(compressedData);
         }

         outputChannel.close();
         fileOutputStream.close();

         indexChannel.close();
         indexStream.close();

         progressConsumer.info("Cropping video files");

         if (multiVideoDataReader != null && multiVideoDataReader.getNumberOfVideos() > 0)
            multiVideoDataReader.crop(destination, getTimestamp(from), getTimestamp(to), progressConsumer.subProgress(0.50, 1.0));

         progressConsumer.done();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void cropMATLAB(File destination, List<YoVariable> logVariables, int from, int to, ProgressConsumer progressConsumer)
   {
      cropMATLAB(destination, logVariables, null, null, from, to, progressConsumer);
   }

   @SuppressWarnings("resource")
   public void cropMATLAB(File destination,
                          List<YoVariable> logVariables,
                          Predicate<YoVariable> variableFilter,
                          Predicate<YoRegistry> registryFilter,
                          int from,
                          int to,
                          ProgressConsumer progressConsumer)
   {
      progressConsumer.started("Cropping data file");
      progressConsumer.info("Initializing cropper");
      progressConsumer.progress(0.0);

      if (!initialize())
      {
         return;
      }

      TIntArrayList logVariableIndices = null;
      List<YoVariable> logVariablesFiltered = null;

      if (variableFilter != null || registryFilter != null)
      {
         logVariableIndices = new TIntArrayList();
         logVariablesFiltered = new ArrayList<>();

         for (int varIndex = 0; varIndex < logVariables.size(); varIndex++)
         {
            boolean keepVariable = true;
            YoVariable logVariable = logVariables.get(varIndex);

            if (variableFilter != null && !variableFilter.test(logVariable))
               keepVariable = false;

            if (keepVariable)
            {
               if (registryFilter != null && !registryFilter.test(logVariable.getRegistry()))
                  keepVariable = false;
            }

            if (keepVariable)
            {
               logVariableIndices.add(varIndex);
               logVariablesFiltered.add(logVariable);
            }
         }
      }

      try
      {
         progressConsumer.info("Creating directories");
         progressConsumer.progress(0.03);

         if (destination.exists())
         {
            progressConsumer.error("Destination " + destination.getAbsolutePath() + " already exists.");
            progressConsumer.done();
            return;
         }
         else
         {
            File parent = destination.getCanonicalFile().getParentFile();

            if (parent != null && !parent.mkdirs() && !parent.isDirectory())
            {
               progressConsumer.error("Cannot make parent directory for " + destination.getAbsolutePath());
               progressConsumer.done();
               return;
            }
         }

         progressConsumer.info("Seeking variable data");
         progressConsumer.progress(0.04);

         MatFile matFile = Mat5.newMatFile();
         Struct rootStruct = Mat5.newStruct();
         YoRegistry rootRegistry = logVariables.get(0).getRegistry().getRoot();
         matFile.addArray(rootRegistry.getName(), rootStruct);
         int numberOfYoVariables = logVariablesFiltered != null ? logVariablesFiltered.size() : logVariables.size();

         int dataLength = to - from + 1;
         List<MatlabEntryWriter> matlabEntryWriters = new ArrayList<>(numberOfYoVariables);

         progressConsumer.info("Creating data structure");

         ProgressConsumer structSetupProgress = progressConsumer.subProgress(0.04, 0.20);
         // Setup the structure
         for (int varIndex = 0; varIndex < numberOfYoVariables; varIndex++)
         {
            structSetupProgress.progress((double) (varIndex) / (double) (numberOfYoVariables - 1.0));

            Struct parentStruct = rootStruct;

            YoVariable yoVariable = logVariablesFiltered != null ? logVariablesFiltered.get(varIndex) : logVariables.get(varIndex);

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
                     parentStruct.set(subName, childStruct);
                  }

                  parentStruct = childStruct;
               }
            }

            Matrix matMatrix = Mat5.newMatrix(dataLength, 1);
            matlabEntryWriters.add(createMatlabEntryWriter(yoVariable, matMatrix));
            parentStruct.set(yoVariable.getName(), matMatrix);
         }

         progressConsumer.info("Writing variable data");
         ProgressConsumer dataCopyingProgress = progressConsumer.subProgress(0.20, 1.00);

         if (logVariablesFiltered != null)
         {
            for (int i = from; i <= to; i++)
            {
               dataCopyingProgress.progress((double) (i - from) / (double) (to - from));
               LongBuffer data = readData(i).asLongBuffer();

               for (int varIndex = 0; varIndex < numberOfYoVariables; varIndex++)
               {
                  matlabEntryWriters.get(varIndex).accept(i - from, data.get(logVariableIndices.get(varIndex) + 1));
               }
            }
         }
         else
         {
            for (int i = from; i <= to; i++)
            {
               dataCopyingProgress.progress((double) (i - from) / (double) (to - from));
               LongBuffer data = readData(i).asLongBuffer();

               data.get(); // Time entry

               for (int varIndex = 0; varIndex < numberOfYoVariables; varIndex++)
               {
                  matlabEntryWriters.get(varIndex).accept(i - from, data.get());
               }
            }
         }

         Mat5.writeToFile(matFile, destination);
         rootStruct.close();

         progressConsumer.done();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private static MatlabEntryWriter createMatlabEntryWriter(YoVariable variable, Matrix matrix)
   {
      if (variable instanceof YoBoolean)
         return (index, longValue) -> matrix.setBoolean(index, longValue == 1);
      if (variable instanceof YoDouble)
         return (index, longValue) -> matrix.setDouble(index, Double.longBitsToDouble(longValue));
      if (variable instanceof YoInteger)
         return (index, longValue) -> matrix.setInt(index, (int) longValue);
      if (variable instanceof YoLong)
         return (index, longValue) -> matrix.setLong(index, longValue);
      if (variable instanceof YoEnum<?>)
         return (index, longValue) -> matrix.setByte(index, (byte) longValue);
      throw new IllegalArgumentException("Unsupported variable type: " + variable);
   }

   private interface MatlabEntryWriter
   {
      void accept(int index, long longValue);
   }

   public void cropCSV(File destination,
                       List<YoVariable> logVariables,
                       Predicate<YoVariable> variableFilter,
                       Predicate<YoRegistry> registryFilter,
                       int from,
                       int to,
                       ProgressConsumer progressConsumer)
   {
      progressConsumer.started("Cropping data file");
      progressConsumer.info("Initializing cropper");
      progressConsumer.progress(0.0);

      if (!initialize())
      {
         return;
      }

      TIntArrayList logVariableIndices = null;
      List<YoVariable> logVariablesFiltered = null;

      if (variableFilter != null || registryFilter != null)
      {
         logVariableIndices = new TIntArrayList();
         logVariablesFiltered = new ArrayList<>();

         for (int varIndex = 0; varIndex < logVariables.size(); varIndex++)
         {
            boolean keepVariable = true;
            YoVariable logVariable = logVariables.get(varIndex);

            if (variableFilter != null && !variableFilter.test(logVariable))
               keepVariable = false;

            if (keepVariable)
            {
               if (registryFilter != null && !registryFilter.test(logVariable.getRegistry()))
                  keepVariable = false;
            }

            if (keepVariable)
            {
               logVariableIndices.add(varIndex);
               logVariablesFiltered.add(logVariable);
            }
         }
      }

      try
      {
         progressConsumer.info("Creating directories");
         progressConsumer.progress(0.03);

         if (destination.exists())
         {
            progressConsumer.error("Destination " + destination.getAbsolutePath() + " already exists.");
            progressConsumer.done();
            return;
         }
         else
         {
            File parent = destination.getCanonicalFile().getParentFile();

            if (parent != null && !parent.mkdirs() && !parent.isDirectory())
            {
               progressConsumer.error("Cannot make parent directory for " + destination.getAbsolutePath());
               progressConsumer.done();
               return;
            }
         }

         progressConsumer.info("Seeking variable data");
         progressConsumer.progress(0.04);

         int numberOfYoVariables = logVariablesFiltered != null ? logVariablesFiltered.size() : logVariables.size();
         PrintStream printStream = new PrintStream(new FileOutputStream(destination));
         List<LongConsumer> valueWriter = new ArrayList<>();

         for (int varIndex = 0; varIndex < numberOfYoVariables; varIndex++)
         {
            YoVariable yoVariable = logVariablesFiltered != null ? logVariablesFiltered.get(varIndex) : logVariables.get(varIndex);

            printStream.print(yoVariable.getFullNameString());
            if (varIndex < numberOfYoVariables - 1)
               printStream.print(", ");
            else
               printStream.println();

            if (yoVariable instanceof YoBoolean)
               valueWriter.add(longValue -> printStream.print(longValue == 1));
            else if (yoVariable instanceof YoDouble)
               valueWriter.add(longValue -> printStream.print(Double.longBitsToDouble(longValue)));
            else if (yoVariable instanceof YoInteger)
               valueWriter.add(longValue -> printStream.print((int) longValue));
            else if (yoVariable instanceof YoLong)
               valueWriter.add(longValue -> printStream.print(longValue));
            else if (yoVariable instanceof YoEnum yoEnum)
               valueWriter.add(longValue ->
               {
                  int ordinal = (int) longValue;
                  printStream.print(ordinal == YoEnum.NULL_VALUE ? YoEnum.NULL_VALUE_STRING : yoEnum.getEnumValuesAsString()[ordinal]);
               });
         }

         progressConsumer.info("Writing variable data");
         ProgressConsumer dataCopyingProgress = progressConsumer.subProgress(0.04, 1.00);

         if (logVariablesFiltered != null)
         {
            for (int i = from; i <= to; i++)
            {
               progressConsumer.info("Writing variable data %d/%d".formatted(i - from, to - from));
               dataCopyingProgress.progress((double) (i - from) / (double) (to - from));
               LongBuffer data = readData(i).asLongBuffer();

               for (int varIndex = 0; varIndex < numberOfYoVariables; varIndex++)
               {
                  valueWriter.get(varIndex).accept(data.get(logVariableIndices.get(varIndex) + 1));
                  if (varIndex < numberOfYoVariables - 1)
                     printStream.print(", ");
                  else
                     printStream.println();
               }
            }
         }
         else
         {
            for (int i = from; i <= to; i++)
            {
               progressConsumer.info("Writing variable data %d/%d".formatted(i - from, to - from));
               dataCopyingProgress.progress((double) (i - from) / (double) (to - from));
               LongBuffer data = readData(i).asLongBuffer();

               data.get(); // Time entry

               for (int varIndex = 0; varIndex < numberOfYoVariables; varIndex++)
               {
                  valueWriter.get(varIndex).accept(data.get());
                  if (varIndex < numberOfYoVariables - 1)
                     printStream.print(", ");
                  else
                     printStream.println();
               }
            }
         }

         printStream.close();
         progressConsumer.done();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
