package us.ihmc.scs2.session.log;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
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
import java.util.Random;

public class CompositeLogDataReader implements LogDataReaderInterface
{
   private final LogDataReader parentLogReader;
   private final String path;

   private final List<ChildLogData> childLogs = new ArrayList<>();
   private final HashMap<String, ChildLogData> childLogNames = new HashMap<>();

   private final YoRegistry parentRegistry = new YoRegistry("parentRegistry");

   public CompositeLogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.path = logDirectory.getAbsolutePath();
      parentLogReader = new LogDataReader(logDirectory, progressConsumer);
      parentRegistry.addChild(parentLogReader.getLocalYoRegistry());

      for (ChildLog childLog : parentLogReader.getLogProperties().getChildLogs())
      {
         loadChildLog(logDirectory, childLog, progressConsumer);
      }
   }

   /**
    * Internal method used to load a child log from the configuration when this object is constructed. This happens only when opening a log that has child
    * members. This both adds the log, and binds the synchronization
    */
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

   /**
    * External method used to add a new log as a child to this composite log reader. This log is not yet synchronized, which gets called later using
    * {@link #synchronizeChildLogWithParent(String, String, String)}.
    */
   public ChildLogData addChildLog(File logDirectory) throws IOException
   {
      ChildLogData childLogData = addChildLogInternal(logDirectory, null, null);
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
      ChildLogData childLogData = new ChildLogData(this, logDirectory, progressConsumer);
      childLogData.seek(parentLogReader.getCurrentLogPosition());

      childLogs.add(childLogData);
      childLogNames.put(childLogData.getChildLogDataReader().getLogDirectory().getAbsolutePath(), childLogData);

      return childLogData;
   }


   public Synchronization synchronizeChildLogWithParent(String childLogToSynchronize, String parentLogSyncVariableName, String childLogSyncVariableName)
   {
      ChildLogData childLogData = childLogNames.get(childLogToSynchronize);

      YoVariable parentVariable = findVariable(parentLogReader, parentLogSyncVariableName);
      YoVariable childVariable = findVariable(childLogData.getChildLogDataReader(), childLogSyncVariableName);

      // First, get simple coefficients. This will allow us to compute the max and min range over which the data overlaps to perform a more precise fit.
      double[] parentCoefficients = computeEasyLinearCoefficients(parentLogReader, parentVariable);
      double[] childCoefficients = computeEasyLinearCoefficients(childLogData.getChildLogDataReader(), childVariable);

      double[] mapping = computeTimestampMapping(parentCoefficients, childCoefficients);
      ChildLogSynchronization synchronization = childLogData.getSynchronization();
      synchronization.setOffset(mapping[1]);
      synchronization.setJogRate(mapping[0]);

      // Compute the overlapping range of data that we care about this being synchronized.
      long minParentIndex = Math.max(0, synchronization.computeParentPosition(0));
      long maxParentIndex = Math.min(parentLogReader.getNumberOfEntries() - 1, synchronization.computeParentPosition(childLogData.getChildLogDataReader().getNumberOfEntries() - 1));
      long minChildIndex = Math.max(synchronization.computeChildPosition((int) minParentIndex), 0);
      long maxChildIndex = Math.min(childLogData.getChildLogDataReader().getNumberOfEntries() - 1, synchronization.computeChildPosition((int) maxParentIndex));

      double[] fitParentCoefficients = computeLinearFit(parentLogReader, parentVariable, (int) minParentIndex, (int) maxParentIndex);
      double[] fitChildCoefficients = computeLinearFit(childLogData.getChildLogDataReader(), childVariable, (int) minChildIndex, (int) maxChildIndex);

      double[] fitMapping = computeTimestampMapping(fitParentCoefficients, fitChildCoefficients);
      synchronization.setOffset(fitMapping[1]);
      synchronization.setJogRate(fitMapping[0]);

      return synchronization.toPacket();
   }

   private static YoVariable findVariable(LogDataReaderInterface logDataReader, String variableName)
   {
      return logDataReader.getYoVariablesList()
                          .stream()
                          .filter(var -> var.getName().contains(variableName))
                          .findFirst()
                          .orElse(null);
   }

   private static double[] computeTimestampMapping(double[] parentCoefficients, double[] childCoefficients)
   {
      double jogRate = parentCoefficients[0] / childCoefficients[0];
      double offset = ((parentCoefficients[1] - childCoefficients[1]) / childCoefficients[0]);
      return new double[]{jogRate, offset};
   }

   private static double[] computeEasyLinearCoefficients(LogDataReaderInterface logDataReader, YoVariable variable)
   {
      int currentPosition = logDataReader.getCurrentLogPosition();
      int finalPosition = logDataReader.getNumberOfEntries() - 1;
      logDataReader.seek(0);
      logDataReader.read();
      double initialValue = variable.getValueAsDouble();
      logDataReader.seek(finalPosition);
      logDataReader.read();
      double finalValue = variable.getValueAsDouble();

      logDataReader.seek(currentPosition);

      return new double[] {(finalValue - initialValue) / finalPosition, initialValue};
   }

   private static double[] computeLinearFit(LogDataReaderInterface logDataReader, YoVariable variable, int min, int max)
   {
      int samples = 50;
      int currentPosition = logDataReader.getCurrentLogPosition();

      Random random = new Random(1738L);
      DMatrixRMaj A = new DMatrixRMaj(50, 2);
      DMatrixRMaj b = new DMatrixRMaj(50, 1);

      for (int i = 0; i < samples; i++)
      {
         int position = random.nextInt(min, max + 1);
         logDataReader.seek(position);
         logDataReader.read();
         b.set(i, variable.getValueAsDouble());
         A.set(i, 0, position);
         A.set(i, 1, 1.0);
      }

      DMatrixRMaj ATransA = new DMatrixRMaj(2, 2);
      DMatrixRMaj lhs = new DMatrixRMaj(2, samples);
      CommonOps_DDRM.multInner(A, ATransA);
      CommonOps_DDRM.invert(ATransA);
      CommonOps_DDRM.multTransB(ATransA, A, lhs);
      DMatrixRMaj solution = new DMatrixRMaj(2, 1);
      CommonOps_DDRM.mult(lhs, b, solution);

      // re-initialize the reader
      logDataReader.seek(currentPosition);

      return solution.data;
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
   public void removeTimestampListeners()
   {
      parentLogReader.getTimestamp().removeListeners();
      for (ChildLogData childLogData : childLogs)
         childLogData.getChildLogDataReader().getTimestamp().removeListeners();
   }

   @Override
   public double getDt()
   {
      return parentLogReader.getParser().getDt();
   }

   @Override
   public YoRegistry getLocalYoRegistry()
   {
      return parentRegistry;
   }

   @Override
   public YoRegistry getLogRootRegistry()
   {
      return parentLogReader.getLogRootRegistry();
   }

   @Override
   public YoGraphicsListRegistry getLogSCS1YoGraphics()
   {
      return parentLogReader.getLogSCS1YoGraphics();
   }

   @Override
   public List<YoGraphicGroupDefinition> getLogSCS2YoGraphics()
   {
      return parentLogReader.getLogSCS2YoGraphics();
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
      return parentLogReader.getYoVariablesList();
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
