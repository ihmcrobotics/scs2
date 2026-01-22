package us.ihmc.scs2.session.log;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.ChildLog;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.Synchronization;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class CompositeLogDataReader implements LogDataReaderInterface
{
   private static final Logger log = LoggerFactory.getLogger(CompositeLogDataReader.class);
   private final LogDataReader mainLogReader;
   private final long mainDT;
   private final String path;

   private final List<ChildLogData> childLogs = new ArrayList<>();
   private final HashMap<String, ChildLogData> childLogNames = new HashMap<>();

   private final YoRegistry sharedRegistry = new YoRegistry("sharedRegistry");
   private final List<YoVariable> yoVariables = new ArrayList<>();

   private final YoGraphicGroupDefinition scs2Graphics = new YoGraphicGroupDefinition();
   private final YoGraphicsListRegistry scs1Graphics = new YoGraphicsListRegistry();

   private final LogProperties logProperties;

   public CompositeLogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.path = logDirectory.getAbsolutePath();
      mainLogReader = new LogDataReader(logDirectory, progressConsumer);
      sharedRegistry.addChild(mainLogReader.getLocalYoRegistry());
      long initialTimestamp = mainLogReader.getInitialTimestamp();
      mainDT = mainLogReader.getTimestamp(1) - initialTimestamp;

      // Load all the variables into a local copy.
      yoVariables.addAll(mainLogReader.getYoVariablesList());

      // Load all the graphics into the local copy.
      mainLogReader.getLogSCS1YoGraphics().getYoGraphicsLists().forEach(scs1Graphics::registerYoGraphicsList);
      List<ArtifactList> artifactLists = new ArrayList<>();
      mainLogReader.getLogSCS1YoGraphics().getRegisteredArtifactLists(artifactLists);
      artifactLists.forEach(scs1Graphics::registerArtifactList);
      mainLogReader.getLogSCS2YoGraphics().forEach(scs2Graphics::addChild);

      logProperties = mainLogReader.getLogProperties();

      for (ChildLog childLog : logProperties.getChildLogs())
      {
         loadChildLog(logDirectory, childLog, progressConsumer);
      }
   }

   private void loadChildLog(File logDirectory, ChildLog childLog, ProgressConsumer progressConsumer) throws IOException
   {
      ChildLogData logData = addLogInternal(logDirectory, progressConsumer, childLog.getChildName().toString());
      if (logData == null)
      {
         LogTools.warn("Failed to load child log: " + childLog.getChildName() + ".");
         return;
      }

      logData.synchronization.set(childLog.getSynchronization());
      logData.logDataReader.seek(mainLogReader.getCurrentLogPosition());
   }

   public int getLogNumber(String logName)
   {
      return childLogs.indexOf(childLogNames.get(logName));
   }

   public LogDataReaderInterface addLog(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      ChildLogData addedLog = addLogInternal(logDirectory, progressConsumer, null);
      if (addedLog == null)
         return null;

      ChildLog childLog = logProperties.getChildLogs().add();
      childLog.setChildName(addedLog.logDataReader.getLogProperties().getNameAsString());

      return addedLog.logDataReader;
   }

   private ChildLogData addLogInternal(File logDirectory, ProgressConsumer progressConsumer, String childName) throws IOException
   {
      if (childName != null)
         // Actually loading a child log
         logDirectory = new File(logDirectory, childName);
      if (isLogLoaded(logDirectory))
      {
         LogTools.warn(logDirectory.getAbsolutePath() + " is already loaded.");
         return null;
      }
      ChildLogData addedLogData = new ChildLogData(logDirectory, progressConsumer, mainDT);
      addedLogData.seek(mainLogReader.getCurrentLogPosition());

      childLogs.add(addedLogData);
      childLogNames.put(addedLogData.logDataReader.getLogDirectory().getAbsolutePath(), addedLogData);

      return addedLogData;
   }


   public Synchronization bindSynchronization(String logToSynchronize, String mainLogVarName, String logToSyncVarName)
   {
      ChildLogData addedLogData = childLogNames.get(logToSynchronize);
      YoVariable mainLogVar = mainLogReader.getYoVariablesList()
                   .stream()
                   .filter(var -> var.getName().contains(mainLogVarName))
                   .findFirst()
                   .orElse(null);
      YoVariable addedLogVar = addedLogData.logDataReader.getYoVariablesList()
                                           .stream()
                                           .filter(var -> var.getName().contains(logToSyncVarName))
                                           .findFirst()
                                           .orElse(null);
      int currentMainPosition = mainLogReader.getCurrentLogPosition();
      int currentAddedPosition = addedLogData.logDataReader.getCurrentLogPosition();
      mainLogReader.read();
      addedLogData.logDataReader.read();
      double currentMainValue = mainLogVar.getValueAsDouble();
      double currentAddedValue = addedLogVar.getValueAsDouble();
      mainLogReader.read();
      addedLogData.logDataReader.read();
      double nextMainValue = mainLogVar.getValueAsDouble();
      double nextAddedValue = addedLogVar.getValueAsDouble();
      double mainDT = nextMainValue - currentMainValue;
      double addedDT = nextAddedValue - currentAddedValue;

      double timeRateMultiplier = addedDT / mainDT;
      addedLogData.synchronization.setOffset((int) ((currentAddedPosition - currentMainPosition) * timeRateMultiplier));

      addedLogData.seek(mainLogReader.getCurrentLogPosition());


      return addedLogData.synchronization.toPacket();
   }

   private boolean isLogLoaded(File logDirectory)
   {
      if (path.equals(logDirectory.getAbsolutePath()))
         return true;
      else
         return childLogs.stream().anyMatch(addedLog -> addedLog.path.equals(logDirectory.getAbsolutePath()));
   }

   @Override
   public void seek(int position)
   {
      mainLogReader.seek(position);

      for (ChildLogData addedLog : childLogs)
         addedLog.seek(position);
   }

   @Override
   public boolean read()
   {
      // This has to be called before the added log to get the index of the main log correct.
      boolean ended = mainLogReader.read();
      for (ChildLogData addedLog : childLogs)
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
   public LogProperties getLogProperties()
   {
      return logProperties;
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
      for (ChildLogData addedLog : childLogs)
         result.add(addedLog.logDataReader);
      return result;
   }

   private class ChildLogData
   {
      public final LogDataReader logDataReader;
      public final ChildLogSynchronization synchronization = new ChildLogSynchronization();
      private boolean inBounds;
      public final String path;

      public ChildLogData(File logDirectory, ProgressConsumer progressConsumer, long mainDT) throws IOException
      {
         this.path = logDirectory.getAbsolutePath();
         logDataReader = new LogDataReader(logDirectory, progressConsumer);
         long localDT = logDataReader.getTimestamp(1) - logDataReader.getInitialTimestamp();
         synchronization.jogRate = (int) Math.round(((double) localDT) / ((double) mainDT));
      }

      public void seek(int mainPosition)
      {
         long relativePosition = synchronization.computeRelativePosition(mainPosition);
         if (relativePosition < 0 || relativePosition >= logDataReader.getNumberOfEntries())
         {
            inBounds = false;
         }
         else
         {
            inBounds = true;
            logDataReader.seek((int) relativePosition);
         }
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

   private static class ChildLogSynchronization
   {
      private int offset = -1;
      private int jogRate = -1;

      public ChildLogSynchronization()
      {
      }

      public ChildLogSynchronization(int startOffset, int timeJogRate)
      {
         setOffset(startOffset);
         setJogRate(timeJogRate);
      }

      public void set(Synchronization synchronization)
      {
         setOffset(synchronization.getOffset());
         setJogRate(synchronization.getJogRate());
      }

      public void setOffset(int offest)
      {
         this.offset = offest;
      }

      public void setJogRate(int jogRate)
      {
         this.jogRate = jogRate;
      }

      public Synchronization toPacket()
      {
         Synchronization synchronization = new Synchronization();
         synchronization.setJogRate(jogRate);
         synchronization.setOffset(offset);

         return synchronization;
      }


      private long computeRelativePosition(int mainPosition)
      {
         return (long) mainPosition * jogRate + offset;
      }
   }
}
