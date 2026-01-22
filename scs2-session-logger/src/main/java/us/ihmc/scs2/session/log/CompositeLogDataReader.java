package us.ihmc.scs2.session.log;

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
   private final LogDataReader parentLogReader;
   private final long mainDT;
   private final String path;

   private final List<ChildLogData> childLogs = new ArrayList<>();
   private final HashMap<String, ChildLogData> childLogNames = new HashMap<>();

   private final YoRegistry sharedRegistry = new YoRegistry("sharedRegistry");
   private final List<YoVariable> yoVariables = new ArrayList<>();

   private final YoGraphicGroupDefinition scs2Graphics = new YoGraphicGroupDefinition();
   private final YoGraphicsListRegistry scs1Graphics = new YoGraphicsListRegistry();


   public CompositeLogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.path = logDirectory.getAbsolutePath();
      parentLogReader = new LogDataReader(logDirectory, progressConsumer);
      sharedRegistry.addChild(parentLogReader.getLocalYoRegistry());
      long initialTimestamp = parentLogReader.getInitialTimestamp();
      mainDT = parentLogReader.getTimestamp(1) - initialTimestamp;

      // Load all the variables into a local copy.
      yoVariables.addAll(parentLogReader.getYoVariablesList());

      // Load all the graphics into the local copy.
      parentLogReader.getLogSCS1YoGraphics().getYoGraphicsLists().forEach(scs1Graphics::registerYoGraphicsList);
      List<ArtifactList> artifactLists = new ArrayList<>();
      parentLogReader.getLogSCS1YoGraphics().getRegisteredArtifactLists(artifactLists);
      artifactLists.forEach(scs1Graphics::registerArtifactList);
      parentLogReader.getLogSCS2YoGraphics().forEach(scs2Graphics::addChild);


      for (ChildLog childLog : parentLogReader.getLogProperties().getChildLogs())
      {
         loadChildLog(logDirectory, childLog, progressConsumer);
      }
   }

   private void loadChildLog(File logDirectory, ChildLog childLog, ProgressConsumer progressConsumer) throws IOException
   {
      ChildLogData logData = addChildLogInternal(logDirectory, progressConsumer, childLog.getChildName().toString());
      if (logData == null)
      {
         LogTools.warn("Failed to load child log: " + childLog.getChildName() + ".");
         return;
      }

      logData.getSynchronization().set(childLog.getSynchronization());
      logData.getChildLogDataReader().seek(parentLogReader.getCurrentLogPosition());
   }

   public int getLogNumber(String logName)
   {
      return childLogs.indexOf(childLogNames.get(logName));
   }

   public ChildLogData addChildLog(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      ChildLogData childLogData = addChildLogInternal(logDirectory, progressConsumer, null);
      if (childLogData == null)
         return null;

      ChildLog childLog = parentLogReader.getLogProperties().getChildLogs().add();
      childLog.setChildName(childLogData.getChildLogDataReader().getLogProperties().getNameAsString());

      return childLogData;
   }

   private ChildLogData addChildLogInternal(File logDirectory, ProgressConsumer progressConsumer, String childName) throws IOException
   {
      if (childName != null)
         // Actually loading a child log
         logDirectory = new File(logDirectory, childName);
      if (isLogLoaded(logDirectory))
      {
         LogTools.warn(logDirectory.getAbsolutePath() + " is already loaded.");
         return null;
      }
      ChildLogData childLogData = new ChildLogData(this, logDirectory, progressConsumer, mainDT);
      childLogData.seek(parentLogReader.getCurrentLogPosition());

      childLogs.add(childLogData);
      childLogNames.put(childLogData.getChildLogDataReader().getLogDirectory().getAbsolutePath(), childLogData);

      return childLogData;
   }


   public Synchronization synchronizeChildLogWithParent(String childLogToSynchronize, String parentLogSyncVariableName, String childLogSyncVariableName)
   {
      ChildLogData childLogData = childLogNames.get(childLogToSynchronize);
      YoVariable parentLogVariable = parentLogReader.getYoVariablesList()
                                                    .stream()
                                                    .filter(var -> var.getName().contains(parentLogSyncVariableName))
                                                    .findFirst()
                                                    .orElse(null);
      YoVariable childLogVariable = childLogData.getChildLogDataReader().getYoVariablesList()
                                                                   .stream()
                                                                   .filter(var -> var.getName().contains(childLogSyncVariableName))
                                                                   .findFirst()
                                                                   .orElse(null);
      int currentParentPosition = parentLogReader.getCurrentLogPosition();
      int currentChildPosition = childLogData.getChildLogDataReader().getCurrentLogPosition();
      parentLogReader.read();
      childLogData.getChildLogDataReader().read();
      double currentParentValue = parentLogVariable.getValueAsDouble();
      double currentChildValue = childLogVariable.getValueAsDouble();
      parentLogReader.read();
      childLogData.getChildLogDataReader().read();
      double nextParentValue = parentLogVariable.getValueAsDouble();
      double nextChildValue = childLogVariable.getValueAsDouble();
      double parentDT = nextParentValue - currentParentValue;
      double childDT = nextChildValue - currentChildValue;

      double timeRateMultiplier = childDT / parentDT;
      childLogData.getSynchronization().setOffset((int) ((currentChildPosition - currentParentPosition) * timeRateMultiplier));

      childLogData.seek(parentLogReader.getCurrentLogPosition());

      return childLogData.getSynchronization().toPacket();
   }

   private boolean isLogLoaded(File logDirectory)
   {
      if (path.equals(logDirectory.getAbsolutePath()))
         return true;
      else
         return childLogs.stream().anyMatch(childLog -> childLog.getPath().equals(logDirectory.getAbsolutePath()));
   }

   @Override
   public void seek(int position)
   {
      parentLogReader.seek(position);

      for (ChildLogData childLog : childLogs)
         childLog.seek(position);
   }

   @Override
   public boolean read()
   {
      // This has to be called before the added log to get the index of the main log correct.
      boolean ended = parentLogReader.read();
      for (ChildLogData childLog : childLogs)
         childLog.read();

      return ended;
   }

   @Override
   public double getCurrentRobotTime()
   {
      return parentLogReader.getCurrentRobotTime();
   }

   @Override
   public int getCurrentLogPosition()
   {
      return parentLogReader.getCurrentLogPosition();
   }

   @Override
   public File getLogDirectory()
   {
      return parentLogReader.getLogDirectory();
   }

   @Override
   public LogProperties getLogProperties()
   {
      return parentLogReader.getLogProperties();
   }

   @Override
   public YoLong getTimestamp()
   {
      return parentLogReader.getTimestamp();
   }

   @Override
   public double getDt()
   {
      return parentLogReader.getParser().getDt();
   }

   @Override
   public YoRegistry getLocalYoRegistry()
   {
      return sharedRegistry;
   }

   @Override
   public YoRegistry getLogRootRegistry()
   {
      return parentLogReader.getParser().getRootRegistry();
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
      return parentLogReader.getNumberOfEntries();
   }

   @Override
   public long getRelativeTimestamp(int position)
   {
      return parentLogReader.getRelativeTimestamp(position);
   }

   @Override
   public List<YoVariable> getYoVariablesList()
   {
      return yoVariables;
   }

   @Override
   public List<JointState> getJointStates()
   {
      return parentLogReader.getJointStates();
   }

   public List<ChildLogData> getChildLogData()
   {
      return childLogs;
   }

   public List<LogDataReaderInterface> getChildLogDataReaders()
   {
      List<LogDataReaderInterface> result = new ArrayList<>();
      for (ChildLogData addedLog : childLogs)
         result.add(addedLog.getChildLogDataReader());
      return result;
   }
}
