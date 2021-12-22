package us.ihmc.scs2.sessionVisualizer.jfx.session.log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;

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
            dataCopyingProgress.progress((double) (i - from)  / (double) (to - from));
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
}
