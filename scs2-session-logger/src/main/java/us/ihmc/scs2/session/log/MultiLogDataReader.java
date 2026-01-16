package us.ihmc.scs2.session.log;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class MultiLogDataReader implements LogDataReaderInterface
{
   private static final Logger log = LoggerFactory.getLogger(MultiLogDataReader.class);
   private final LogDataReader mainLogReader;
   private final long mainDT;
   private final String path;

   private final List<AddedLogData> addedLogs = new ArrayList<>();

   private final YoRegistry sharedRegistry = new YoRegistry("sharedRegistry");
   private final List<YoVariable> yoVariables = new ArrayList<>();

   private final YoGraphicGroupDefinition scs2Graphics = new YoGraphicGroupDefinition();
   private final YoGraphicsListRegistry scs1Graphics = new YoGraphicsListRegistry();

   public MultiLogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.path = logDirectory.getAbsolutePath();
      mainLogReader = new LogDataReader(logDirectory, progressConsumer);
      sharedRegistry.addChild(mainLogReader.getLocalYoRegistry());
      long initialTimestamp = mainLogReader.getInitialTimestamp();
      mainDT = mainLogReader.getTimestamp(1) - initialTimestamp;
      yoVariables.addAll(mainLogReader.getParser().getYoVariablesList());

      mainLogReader.getLogSCS1YoGraphics().getYoGraphicsLists().forEach(scs1Graphics::registerYoGraphicsList);
      List<ArtifactList> artifactLists = new ArrayList<>();
      mainLogReader.getLogSCS1YoGraphics().getRegisteredArtifactLists(artifactLists);
      artifactLists.forEach(scs1Graphics::registerArtifactList);
      mainLogReader.getLogSCS2YoGraphics().forEach(scs2Graphics::addChild);
   }

   public LogDataReaderInterface addLog(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      if (isLogLoaded(logDirectory))
      {
         LogTools.warn(logDirectory.getAbsolutePath() + " is already loaded.");
         return null;
      }
      AddedLogData addedLogData = new AddedLogData(logDirectory, progressConsumer, mainLogReader.getInitialTimestamp(), mainDT);
      addedLogData.seek(mainLogReader.getCurrentLogPosition());
//      sharedRegistry.addChild(addedLogData.logDataReader.getYoRegistry());
      yoVariables.addAll(addedLogData.logDataReader.getYoVariablesList());
      // TODO augment log properties.
      // TODO augment parser

      addedLogs.add(addedLogData);

      return addedLogData.logDataReader;
   }

   private boolean isLogLoaded(File logDirectory)
   {
      if (path.equals(logDirectory.getAbsolutePath()))
         return true;
      else
         return addedLogs.stream().anyMatch(addedLog -> addedLog.path.equals(logDirectory.getAbsolutePath()));
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
   public File getLogDirectory()
   {
      return mainLogReader.getLogDirectory();
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
   public YoVariableHandshakeParser getParser()
   {
      return mainLogReader.getParser();
   }

   @Override
   public double getDt()
   {
      return mainLogReader.getParser().getDt();
   }

   @Override
   public YoRegistry getLocalYoRegistry()
   {
      return sharedRegistry;
   }

   @Override
   public YoRegistry getLogRootRegistry()
   {
      return mainLogReader.getParser().getRootRegistry();
   }

   @Override
   public YoGraphicsListRegistry getLogSCS1YoGraphics()
   {
      return scs1Graphics;
   }

   @Override
   public List<YoGraphicGroupDefinition> getLogSCS2YoGraphics()
   {
      List<YoGraphicGroupDefinition> result = new ArrayList<>();
      result.add(scs2Graphics);
      return result;
   }

   @Override
   public int getNumberOfEntries()
   {
      return mainLogReader.getNumberOfEntries();
   }

   @Override
   public long getRelativeTimestamp(int position)
   {
      return mainLogReader.getRelativeTimestamp(position);
   }

   @Override
   public List<YoVariable> getYoVariablesList()
   {
      return yoVariables;
   }

   @Override
   public List<JointState> getJointStates()
   {
      return mainLogReader.getJointStates();
   }

   @Override
   public List<LogDataReaderInterface> getAddedLogDataReaders()
   {
      List<LogDataReaderInterface> result = new ArrayList<>();
      for (AddedLogData addedLog : addedLogs)
         result.add(addedLog.logDataReader);
      return result;
   }

   private class AddedLogData
   {
      public final LogDataReader logDataReader;
      public long relativeStart;
      public int timeRateMultiplier;
      private boolean inBounds;
      public final String path;

      public AddedLogData(File logDirectory, ProgressConsumer progressConsumer, long mainInitialTimestamp, long mainDT) throws IOException
      {
         this.path = logDirectory.getAbsolutePath();
         logDataReader = new LogDataReader(logDirectory, progressConsumer);
         long localDT = logDataReader.getTimestamp(1) - logDataReader.getInitialTimestamp();
         timeRateMultiplier = (int) Math.round(((double) localDT) / ((double) mainDT));
         relativeStart = -((logDataReader.getInitialTimestamp() - mainInitialTimestamp) * timeRateMultiplier);
         LogTools.info("Added log {} with localDT {}, mainDT {}.", path, localDT, mainDT);
         LogTools.info("seek multiplier {} and relative start {}", timeRateMultiplier, relativeStart);
      }

      public void seek(int mainPosition)
      {
         long relativePosition = computeRelativePosition(mainPosition);
         if (relativePosition < 0 || relativePosition >= logDataReader.getNumberOfEntries())
         {
            LogTools.info("Main position {} is outside of bounds with relative position {}", mainPosition, relativePosition);
            inBounds = false;
         }
         else
         {
            LogTools.info("Main position {} is inside of bounds with relative position {}", mainPosition, relativePosition);
            inBounds = true;
            logDataReader.seek((int) relativePosition);
         }
      }

      private long computeRelativePosition(int mainPosition)
      {
         return (long) mainPosition * timeRateMultiplier + relativeStart;
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
