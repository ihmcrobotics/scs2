package us.ihmc.scs2.session.log;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
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
   private final LogDataReader mainLogReader;
   private final long mainDT;

   private final List<AddedLogData> addedLogs = new ArrayList<>();

   private final YoRegistry sharedRegistry = new YoRegistry("sharedRegistry");
   private final YoRegistry rootRegistry = new YoRegistry("rootRegistry");
   private final List<YoVariable> yoVariables = new ArrayList<>();

   private final YoGraphicGroupDefinition scs2Graphics = new YoGraphicGroupDefinition();
   private final YoGraphicsListRegistry scs1Graphics = new YoGraphicsListRegistry();

   public MultiLogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      mainLogReader = new LogDataReader(logDirectory, progressConsumer);
      sharedRegistry.addChild(mainLogReader.getLocalYoRegistry());
      long initialTimestamp = mainLogReader.getInitialTimestamp();
      mainDT = mainLogReader.getTimestamp(1) - initialTimestamp;
      yoVariables.addAll(mainLogReader.getParser().getYoVariablesList());
      rootRegistry.addChild(mainLogReader.getParser().getRootRegistry());

      mainLogReader.getLogSCS1YoGraphics().getYoGraphicsLists().forEach(scs1Graphics::registerYoGraphicsList);
      List<ArtifactList> artifactLists = new ArrayList<>();
      mainLogReader.getLogSCS1YoGraphics().getRegisteredArtifactLists(artifactLists);
      artifactLists.forEach(scs1Graphics::registerArtifactList);
      scs2Graphics.addChild(mainLogReader.getLogSCS2YoGraphics());
   }

   public LogDataReaderInterface addLog(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      AddedLogData addedLogData = new AddedLogData(logDirectory, progressConsumer, mainLogReader.getInitialTimestamp(), mainDT);
      addedLogData.seek(mainLogReader.getCurrentLogPosition());
//      sharedRegistry.addChild(addedLogData.logDataReader.getYoRegistry());
      yoVariables.addAll(addedLogData.logDataReader.getYoVariablesList());
      // TODO augment log properties.
      // TODO augment parser

      addedLogs.add(addedLogData);

      return addedLogData.logDataReader;
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
   public YoRegistry getLocalYoRegistry()
   {
      return sharedRegistry;
   }

   @Override
   public YoRegistry getLogRootRegistry()
   {
      return rootRegistry;
   }

   @Override
   public int getNumberOfEntries()
   {
      return mainLogReader.getNumberOfEntries();
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

   private class AddedLogData
   {
      public final LogDataReader logDataReader;
      public int relativeStart;
      public int seekMultiplier;
      private boolean inBounds;

      public AddedLogData(File logDirectory, ProgressConsumer progressConsumer, long mainInitialTimestamp, long mainDT) throws IOException
      {
         logDataReader = new LogDataReader(logDirectory, progressConsumer);
         long localDT = logDataReader.getTimestamp(1) - logDataReader.getInitialTimestamp();
         seekMultiplier = (int) (localDT / mainDT);
         relativeStart =  -((logDataReader.getInitialTimestamp() - mainInitialTimestamp) * seekMultiplier);
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
         return mainPosition * seekMultiplier + relativeStart;
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
