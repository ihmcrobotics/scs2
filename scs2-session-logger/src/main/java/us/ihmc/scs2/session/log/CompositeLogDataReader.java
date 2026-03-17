package us.ihmc.scs2.session.log;

import logger_msgs.msg.dds.ChildLog;
import logger_msgs.msg.dds.Synchronization;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

public class CompositeLogDataReader extends LogDataReader
{
   private final List<ChildLogData> childLogs = new ArrayList<>();
   private final HashMap<String, ChildLogData> childLogNames = new HashMap<>();

   public CompositeLogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      super(logDirectory, progressConsumer);

      for (ChildLog childLog : getLogProperties().getChildLogs())
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
      logData.getChildLogDataReader().seek(getCurrentLogPosition());
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

      ChildLog childLog = getLogProperties().getChildLogs().add();
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
      childLogData.seek(getCurrentLogPosition());

      childLogs.add(childLogData);
      childLogNames.put(childLogData.getChildLogDataReader().getLogDirectory().getAbsolutePath(), childLogData);

      return childLogData;
   }


   public Synchronization synchronizeChildLogWithParent(String childLogToSynchronize, String parentLogSyncVariableName, String childLogSyncVariableName)
   {
      ChildLogData childLogData = childLogNames.get(childLogToSynchronize);

      YoVariable parentVariable = findVariable(this, parentLogSyncVariableName);
      YoVariable childVariable = findVariable(childLogData.getChildLogDataReader(), childLogSyncVariableName);

      // First, get simple coefficients. This will allow us to compute the max and min range over which the data overlaps to perform a more precise fit.
      double[] parentCoefficients = computeEasyLinearCoefficients(this, parentVariable);
      double[] childCoefficients = computeEasyLinearCoefficients(childLogData.getChildLogDataReader(), childVariable);

      double[] mapping = computeTimestampMapping(parentCoefficients, childCoefficients);
      ChildLogSynchronization synchronization = childLogData.getSynchronization();
      synchronization.setOffset(mapping[1]);
      synchronization.setJogRate(mapping[0]);

      // Compute the overlapping range of data that we care about this being synchronized.
      long minParentIndex = Math.max(0, synchronization.computeParentPosition(0));
      long maxParentIndex = Math.min(this.getNumberOfEntries() - 1, synchronization.computeParentPosition(childLogData.getChildLogDataReader().getNumberOfEntries() - 1));
      long minChildIndex = Math.max(synchronization.computeChildPosition((int) minParentIndex), 0);
      long maxChildIndex = Math.min(childLogData.getChildLogDataReader().getNumberOfEntries() - 1, synchronization.computeChildPosition((int) maxParentIndex));

      double[] fitParentCoefficients = computeLinearFit(this, parentVariable, (int) minParentIndex, (int) maxParentIndex);
      double[] fitChildCoefficients = computeLinearFit(childLogData.getChildLogDataReader(), childVariable, (int) minChildIndex, (int) maxChildIndex);

      double[] fitMapping = computeTimestampMapping(fitParentCoefficients, fitChildCoefficients);
      synchronization.setOffset(fitMapping[1]);
      synchronization.setJogRate(fitMapping[0]);

      return synchronization.toPacket();
   }

   private static YoVariable findVariable(LogDataReader logDataReader, String variableName)
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

   private static double[] computeEasyLinearCoefficients(LogDataReader logDataReader, YoVariable variable)
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

   private static double[] computeLinearFit(LogDataReader logDataReader, YoVariable variable, int min, int max)
   {
      int samples = 50;
      int coefficients = 2;
      int currentPosition = logDataReader.getCurrentLogPosition();

      Random random = new Random(1738L);
      // This is performing a linear fit of the equation y = A x + b
      DMatrixRMaj A = new DMatrixRMaj(samples, coefficients);
      DMatrixRMaj b = new DMatrixRMaj(samples, 1);

      for (int i = 0; i < samples; i++)
      {
         int position = random.nextInt(min, max + 1);
         logDataReader.seek(position);
         logDataReader.read();
         b.set(i, variable.getValueAsDouble());
         A.set(i, 0, position);
         A.set(i, 1, 1.0);
      }

      DMatrixRMaj ATransA = new DMatrixRMaj(coefficients, coefficients);
      DMatrixRMaj lhs = new DMatrixRMaj(coefficients, samples);
      CommonOps_DDRM.multInner(A, ATransA);
      CommonOps_DDRM.invert(ATransA);
      CommonOps_DDRM.multTransB(ATransA, A, lhs);
      DMatrixRMaj solution = new DMatrixRMaj(coefficients, 1);
      CommonOps_DDRM.mult(lhs, b, solution);

      // re-initialize the reader
      logDataReader.seek(currentPosition);

      return solution.data;
   }

   private boolean isLogLoaded(File logDirectory)
   {
      if (this.logDirectory.getAbsolutePath().equals(logDirectory.getAbsolutePath()))
         return true;
      else
         return childLogs.stream().anyMatch(childLog -> childLog.getPath().equals(logDirectory.getAbsolutePath()));
   }

   @Override
   public void seek(int position)
   {
      super.seek(position);

      for (ChildLogData childLog : childLogs)
         childLog.seek(position);
   }

   @Override
   public boolean read()
   {
      // This has to be called before the added log to get the index of the main log correct.
      boolean ended = super.read();
      for (ChildLogData childLog : childLogs)
         childLog.read();

      return ended;
   }

   @Override
   public void removeTimestampListeners()
   {
      super.removeTimestampListeners();
      for (ChildLogData childLogData : childLogs)
         childLogData.getChildLogDataReader().getTimestamp().removeListeners();
   }

   public List<ChildLogData> getChildLogData()
   {
      return childLogs;
   }

   public List<LogDataReader> getChildLogDataReaders()
   {
      List<LogDataReader> result = new ArrayList<>();
      for (ChildLogData addedLog : childLogs)
         result.add(addedLog.getChildLogDataReader());
      return result;
   }
}
