package us.ihmc.scs2.session.log;

import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class MultiLogDataReader implements LogDataReaderInterface
{
   private final LogDataReader mainLogReader;
   private final long mainDT;

   private final List<AddedLogData> addedLogs = new ArrayList<>();

   private final YoRegistry sharedRegistry = new YoRegistry("sharedRegistry");

   public MultiLogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      mainLogReader = new LogDataReader(logDirectory, progressConsumer);
      sharedRegistry.addChild(mainLogReader.getYoRegistry());
      long initialTimestamp = mainLogReader.getInitialTimestamp();
      mainDT = mainLogReader.getTimestamp(1) - initialTimestamp;
   }

   public void addLog(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      AddedLogData addedLogData = new AddedLogData(logDirectory, progressConsumer, mainLogReader.getInitialTimestamp(), mainDT);
      addedLogData.seek(mainLogReader.getCurrentLogPosition());
      sharedRegistry.addChild(mainLogReader.getYoRegistry());
      // TODO augment log properties.
      // TODO augment parser

      addedLogs.add(addedLogData);
   }

   @Override
   public void seek(int position)
   {
      mainLogReader.seek(position);

      for (AddedLogData addedLog : addedLogs)
         addedLog.seek(position);
   }

   @Override
   public boolean read()
   {
      // This has to be called before the added log to get the index of the main log correct.
      boolean ended = mainLogReader.read();
      for (AddedLogData addedLog : addedLogs)
         addedLog.read();

      return ended;
   }

   @Override
   public double getCurrentRobotTime()
   {
      return mainLogReader.getCurrentRobotTime();
   }

   @Override
   public int getCurrentLogPosition()
   {
      return mainLogReader.getCurrentLogPosition();
   }

   @Override
   public LogPropertiesReader getLogProperties()
   {
      return mainLogReader.getLogProperties();
   }

   @Override
   public YoLong getTimestamp()
   {
      return mainLogReader.getTimestamp();
   }

   @Override
   public double getDt()
   {
      return mainLogReader.getParser().getDt();
   }

   public YoVariableHandshakeParser getParser()
   {
      return mainLogReader.getParser();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return sharedRegistry;
   }

   @Override
   public int getNumberOfEntries()
   {
      return mainLogReader.getNumberOfEntries();
   }

   private class AddedLogData
   {
      public final LogDataReader logDataReader;
      public long relativeStart;
      public long seekMultiplier;
      private boolean inBounds;

      public AddedLogData(File logDirectory, ProgressConsumer progressConsumer, long mainInitialTimestamp, long mainDT) throws IOException
      {
         logDataReader = new LogDataReader(logDirectory, progressConsumer);
         long localDT = logDataReader.getTimestamp(1) - logDataReader.getInitialTimestamp();
         seekMultiplier = localDT / mainDT;
         relativeStart = logDataReader.getInitialTimestamp() - mainInitialTimestamp;
      }

      public void seek(int mainPosition)
      {
         int relativePosition = computeRelativePosition(mainPosition);
         if (relativePosition < 0 || relativePosition >= logDataReader.getNumberOfEntries())
            inBounds = false;
         else
         {
            inBounds = true;
            logDataReader.seek(relativePosition);
         }
      }

      private int computeRelativePosition(int mainPosition)
      {
         throw new RuntimeException("This needs to be implemented still.");
      }

      public void read()
      {
         // need to perform a seek, as the indexing is different between this and the main log reader
         seek(mainLogReader.getCurrentLogPosition());
         if (inBounds)
            logDataReader.read();
         else
            logDataReader.setToNaN();
      }
   }
}
