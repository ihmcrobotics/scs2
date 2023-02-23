package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

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

         progressConsumer.info("Writing variable data");

         ProgressConsumer dataCopyingProgress;
         if (multiVideoDataReader == null || multiVideoDataReader.getNumberOfVideos() == 0)
            dataCopyingProgress = progressConsumer.subProgress(0.10, 1.00);
         else
            dataCopyingProgress = progressConsumer.subProgress(0.10, 0.50);

         ByteBuffer indexBuffer = ByteBuffer.allocateDirect(16);
         for (int i = from; i <= to; i++)
         {
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

      if (variableFilter != null)
         logVariables = logVariables.stream().filter(variableFilter).toList();
      if (registryFilter != null)
         logVariables = logVariables.stream().filter(var -> registryFilter.test(var.getRegistry())).toList();

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

         int dataLength = to - from + 1;
         List<MatlabEntryWriter> matlabEntryWriters = new ArrayList<>(logVariables.size());

         progressConsumer.info("Creating data structure");

         ProgressConsumer structSetupProgress = progressConsumer.subProgress(0.04, 0.20);
         // Setup the structure
         for (int varIndex = 0; varIndex < logVariables.size(); varIndex++)
         {
            structSetupProgress.progress((double) (varIndex) / (double) (logVariables.size() - 1.0));

            Struct parentStruct = rootStruct;

            YoVariable yoVariable = logVariables.get(varIndex);

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

         for (int i = from; i <= to; i++)
         {
            dataCopyingProgress.progress((double) (i - from) / (double) (to - from));
            LongBuffer data = readData(i).asLongBuffer();

            data.get(); // Time entry

            for (int varIndex = 0; varIndex < logVariables.size(); varIndex++)
            {
               matlabEntryWriters.get(varIndex).accpet(i, data.get());
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
      void accpet(int index, long longValue);
   }
}
