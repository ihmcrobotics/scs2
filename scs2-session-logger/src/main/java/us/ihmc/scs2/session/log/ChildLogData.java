package us.ihmc.scs2.session.log;

import java.io.File;
import java.io.IOException;

public class ChildLogData
{
   private final LogDataReaderInterface parentLogDataReader;
   private final LogDataReader childLogDataReader;
   private final ChildLogSynchronization synchronization = new ChildLogSynchronization();
   private boolean inBounds;
   private final String path;

   public ChildLogData(LogDataReaderInterface parentLogDataReader, File logDirectory, ProgressConsumer progressConsumer, long mainDT) throws IOException
   {
      this.parentLogDataReader = parentLogDataReader;
      this.path = logDirectory.getAbsolutePath();
      childLogDataReader = new LogDataReader(logDirectory, progressConsumer);
      long localDT = childLogDataReader.getTimestamp(1) - childLogDataReader.getInitialTimestamp();
      synchronization.setJogRate((int) Math.round(((double) localDT) / ((double) mainDT)));
   }

   public void seek(int mainPosition)
   {
      long relativePosition = synchronization.computeRelativePosition(mainPosition);
      if (relativePosition < 0 || relativePosition >= childLogDataReader.getNumberOfEntries())
      {
         inBounds = false;
      }
      else
      {
         inBounds = true;
         childLogDataReader.seek((int) relativePosition);
      }
   }

   public void read()
   {
      // need to perform a seek, as the indexing is different between this and the main log reader
      seek(parentLogDataReader.getCurrentLogPosition());
      if (inBounds)
         childLogDataReader.read();
      else
         childLogDataReader.setToNaN();
   }

   public ChildLogSynchronization getSynchronization()
   {
      return synchronization;
   }

   public LogDataReaderInterface getChildLogDataReader()
   {
      return childLogDataReader;
   }

   public String getPath()
   {
      return path;
   }
}
