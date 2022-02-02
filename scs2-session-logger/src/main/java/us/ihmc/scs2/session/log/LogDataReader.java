package us.ihmc.scs2.session.log;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.List;
import java.util.stream.IntStream;

import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.LogIndex;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.session.tools.RobotDataLogTools;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class LogDataReader
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final File logDirectory;
   private final LogPropertiesReader logProperties;
   private final YoVariableHandshakeParser parser;

   private final YoLong timestamp;
   private final YoDouble robotTime;
   private final FileChannel logChannel;
   private final FileInputStream logFileInputStream;
   private final List<YoVariable> yoVariables;

   // Compressed data helpers
   private final boolean compressed;
   private final LogIndex logIndex;
   private final ByteBuffer compressedBuffer;
   private int index = 0;

   private final List<JointState> jointStates;

   private final ByteBuffer logLine;
   private final LongBuffer logLongArray;

   private final YoInteger currentRecordTick;

   private final int numberOfEntries;
   private final long initialTimestamp;
   private final long finalTimestamp = 0;

   public LogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.logDirectory = logDirectory;
      logProperties = new LogPropertiesReader(RobotDataLogTools.propertyFile(logDirectory));
      RobotDataLogTools.updateLogs(logDirectory, logProperties, progressConsumer);
      LogTools.info("Loaded log properties.");

      parser = RobotDataLogTools.parseYoVariables(logDirectory, logProperties);
      LogTools.info("Loaded YoVariable definition.");

      LogTools.info("This log contains " + parser.getNumberOfVariables() + " YoVariables");

      timestamp = new YoLong("timestamp", registry);
      robotTime = new YoDouble("robotTime", registry);
      currentRecordTick = new YoInteger("currentRecordTick", registry);

      this.jointStates = parser.getJointStates();
      this.yoVariables = parser.getYoVariablesList();

      int jointStateOffset = yoVariables.size();
      int numberOfJointStates = JointState.getNumberOfJointStates(jointStates);
      int bufferSize = (1 + jointStateOffset + numberOfJointStates) * 8;

      File logdata = RobotDataLogTools.logDataFile(logDirectory, logProperties, true);

      logFileInputStream = new FileInputStream(logdata);
      logChannel = logFileInputStream.getChannel();

      compressed = logProperties.getVariables().getCompressed();
      if (compressed)
      {
         File indexData = new File(logDirectory, logProperties.getVariables().getIndexAsString());

         if (!indexData.exists())
         {
            throw new RuntimeException("Cannot find " + logProperties.getVariables().getIndexAsString());
         }
         logIndex = new LogIndex(indexData, logChannel.size());
         compressedBuffer = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
         numberOfEntries = logIndex.getNumberOfEntries();
         LogTools.info("Loaded indexing.");
      }
      else
      {
         numberOfEntries = (int) (logChannel.size() / bufferSize) - 1;
         logIndex = null;
         compressedBuffer = null;
      }

      logLine = ByteBuffer.allocate(bufferSize);
      logLongArray = logLine.asLongBuffer();

      try
      {
         if (compressed)
         {
            initialTimestamp = logIndex.getInitialTimestamp();
            positionChannel(0);
         }
         else
         {
            readLogLine();
            initialTimestamp = logLine.getLong(0);
            positionChannel(0);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public long getInitialTimestamp()
   {
      return initialTimestamp;
   }

   public long getFinalTimestamp()
   {
      return finalTimestamp;
   }

   public int getNumberOfEntries()
   {
      return numberOfEntries;
   }

   public YoLong getTimestamp()
   {
      return timestamp;
   }

   public void seek(int position)
   {
      currentRecordTick.set(position);
      try
      {
         positionChannel(position);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public boolean read()
   {
      boolean done = readAndProcessALogLineReturnTrueIfDone();
      if (!done)
      {
         currentRecordTick.increment();
      }
      return done;
   }

   private boolean readAndProcessALogLineReturnTrueIfDone()
   {
      try
      {
         if (!readLogLine())
         {
            System.out.println("Reached end of file, stopping simulation thread");
            //            scs.stop(); TODO Need to stop the session
            return true;
         }

         timestamp.set(logLongArray.get());
         robotTime.set(Conversions.nanosecondsToSeconds(timestamp.getLongValue() - initialTimestamp));

         IntStream.range(0, yoVariables.size()).parallel().forEach(i ->
         {
            yoVariables.get(i).setValueFromLongBits(logLongArray.get(logLongArray.position() + i), true);
         });

         logLongArray.position(logLongArray.position() + yoVariables.size());

         for (int i = 0; i < jointStates.size(); i++)
         {
            jointStates.get(i).update(logLongArray);
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      //      t.add(DT);
      return false;
   }

   private void positionChannel(int position) throws IOException
   {
      if (compressed)
      {
         index = position;
         if (index < logIndex.dataOffsets.length)
         {
            logChannel.position(logIndex.dataOffsets[position]);
         }
      }
      else
      {
         logChannel.position((long) position * (long) logLine.capacity());
      }
   }

   public long getTimestamp(int position)
   {
      if (!compressed)
      {
         throw new RuntimeException("Cannot get timestamp for non-compressed logs");
      }

      return logIndex.timestamps[position];
   }

   private boolean readLogLine() throws IOException
   {
      logLine.clear();
      logLongArray.clear();

      if (compressed)
      {
         if (index >= logIndex.getNumberOfEntries())
         {
            return false;
         }
         int size = logIndex.compressedSizes[index];
         compressedBuffer.clear();
         compressedBuffer.limit(size);

         int read = logChannel.read(compressedBuffer);

         if (read != size)
         {
            throw new RuntimeException("Expected read of " + size + ", got " + read + ". TODO: Implement loop for reading the full log line.");
         }
         compressedBuffer.flip();

         SnappyUtils.uncompress(compressedBuffer, logLine);
         ++index;

         return true;
      }
      else
      {
         int read = logChannel.read(logLine);
         if (read < 0)
         {
            return false;
         }
         else if (read != logLine.capacity())
         {
            throw new RuntimeException("Expected read of " + logLine.capacity() + ", got " + read + ". TODO: Implement loop for reading the full log line.");
         }
         else
         {
            return true;
         }
      }
   }

   public int getPosition(long timestamp)
   {
      return logIndex.seek(timestamp);
   }

   public long getRelativeTimestamp(int position)
   {
      return getRelativeTimestamp(getTimestamp(position));
   }

   public long getRelativeTimestamp(long timestamp)
   {
      return timestamp - initialTimestamp;
   }

   public double getRobotTime(long timestamp)
   {
      return Conversions.nanosecondsToSeconds(timestamp - initialTimestamp);
   }

   public double getCurrentRobotTime()
   {
      return robotTime.getValue();
   }

   public File getLogDirectory()
   {
      return logDirectory;
   }

   public LogPropertiesReader getLogProperties()
   {
      return logProperties;
   }

   public YoVariableHandshakeParser getParser()
   {
      return parser;
   }

   public int getCurrentLogPosition()
   {
      return currentRecordTick.getValue();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
