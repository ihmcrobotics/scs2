package us.ihmc.scs2.session.log;

import com.github.luben.zstd.Zstd;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.LogIndex;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.session.tools.RobotDataLogTools;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.FileChannel;
import java.util.List;
import java.util.stream.IntStream;

public class LogDataReader
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   protected final File logDirectory;
   private final LogPropertiesReader logProperties;
   private final YoVariableHandshakeParser parser;

   private final YoLong timestamp;
   private final YoDouble robotTime;
   private final FileChannel logChannel;
   private final FileInputStream logFileInputStream;
   private final List<YoVariable> yoVariables;

   // Compressed data helpers
   private final boolean compressed;
   private final CompressionType compressionType;
   private final LogIndex logIndex;
   private final ByteBuffer compressedBuffer;
   private int index = 0;

   // Batch decompression state
   private final int batchSize;
   private final int singleTickSize;
   private final ByteBuffer batchBuffer;
   private int tickIndexInBatch = 0;
   private int currentBatchTickCount = 0;

   private final List<JointState> jointStates;

   private final ByteBuffer logLine;
   private final LongBuffer logLongArray;

   private final YoInteger currentRecordTick;

   private final int numberOfEntries;
   private final long initialTimestamp;

   public LogDataReader(File logDirectory, ProgressConsumer progressConsumer) throws IOException
   {
      this.logDirectory = logDirectory;
      logProperties = new LogPropertiesReader(RobotDataLogTools.propertyFile(logDirectory));
      RobotDataLogTools.updateLogs(logDirectory, logProperties, progressConsumer);
      LogTools.info("Loaded log properties.");

      parser = RobotDataLogTools.parseYoVariables(logDirectory, logProperties);
      LogTools.info("Loaded YoVariable definition.");

      LogTools.info("This log contains " + parser.getNumberOfVariables() + " YoVariables in " + parser.getRegistries().size() + " registries.");

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
      compressionType = compressed ? CompressionType.SNAPPY : CompressionType.NONE;
      singleTickSize = bufferSize;

      if (compressed)
      {
         File indexData = new File(logDirectory, logProperties.getVariables().getIndexAsString());

         if (!indexData.exists())
         {
            throw new RuntimeException("Cannot find " + logProperties.getVariables().getIndexAsString() + " in " + logDirectory);
         }
         logIndex = new LogIndex(indexData, logChannel.size());

         // For legacy logs we need to check what the batch size is
         int storedBatchSize = 0;
         batchSize = storedBatchSize <= 0 ? 1 : storedBatchSize;

         int rawBatchBytes = bufferSize * batchSize;
         compressedBuffer = ByteBuffer.allocate(switch (compressionType)
                                                {
                                                   case ZSTD -> (int) Zstd.compressBound(rawBatchBytes);
                                                   default -> SnappyUtils.maxCompressedLength(rawBatchBytes);
                                                });
         batchBuffer = ByteBuffer.allocate(rawBatchBytes);

         // If using a batch size, the last batch will probably not contain full valid data because odds are the final tick
         // from the controller isn't an increment of the batch size. We only need to do this check if the batch size isn't 1
         int lastBatchTicks = batchSize; // Last batch is full
         if (batchSize > 1)
         {
            int storedValidTicksInLastBatch = 0;
            lastBatchTicks = storedValidTicksInLastBatch > 0 ? storedValidTicksInLastBatch : batchSize;

         }
         numberOfEntries = (logIndex.getNumberOfEntries() - 1) * batchSize + lastBatchTicks;
         LogTools.info("Loaded indexing.");
      }
      else
      {
         numberOfEntries = (int) (logChannel.size() / bufferSize) - 1;
         logIndex = null;
         compressedBuffer = null;
         batchSize = 1;
         batchBuffer = null;
      }

      logLine = ByteBuffer.allocate(bufferSize);
      logLongArray = logLine.asLongBuffer();

      try
      {
         if (compressed)
         {
            // Workaround for Nadia controller which logs a blank line initially
            if (logIndex.getInitialTimestamp() == 0)
               initialTimestamp = logIndex.timestamps[1];
            else
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

   public int getNumberOfEntries()
   {
      return numberOfEntries;
   }

   public YoLong getTimestamp()
   {
      return timestamp;
   }

   public void removeTimestampListeners()
   {
      timestamp.removeListeners();
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

   public double getDt()
   {
      return parser.getDt();
   }

   public void setToNaN()
   {
      timestamp.set(-1);
      robotTime.setToNaN();

      IntStream.range(0, yoVariables.size()).parallel().forEach(i ->
                                                                {
                                                                   yoVariables.get(i).setValueFromDouble(Double.NaN, false);
                                                                });

      // TODO do something with joint states?
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
                                                                      yoVariables.get(i)
                                                                                 .setValueFromLongBits(logLongArray.get(logLongArray.position() + i), true);
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
         int batchIndex = position / batchSize;
         int tickOffset = position % batchSize;

         currentBatchTickCount = 0;
         tickIndexInBatch = 0;
         index = batchIndex;

         if (batchIndex < logIndex.dataOffsets.length)
            logChannel.position(logIndex.dataOffsets[batchIndex]);

         if (tickOffset > 0)
         {
            readNextBatch();
            tickIndexInBatch = tickOffset;
         }
      }
      else
      {
         logChannel.position((long) position * (long) logLine.capacity());
      }
   }

   private boolean readNextBatch() throws IOException
   {
      // No more batches to read
      if (index >= logIndex.getNumberOfEntries())
         return false;

      // Read the compressed batch from the log channel into compressedBuffer
      int size = logIndex.compressedSizes[index];
      compressedBuffer.clear();
      compressedBuffer.limit(size);

      int read = logChannel.read(compressedBuffer);
      if (read != size)
         throw new RuntimeException("Expected read of " + size + ", got " + read + ". TODO: Implement loop for reading the full log line.");

      // Prepare buffers for decompression: flip compressedBuffer for reading, clear batchBuffer for writing
      compressedBuffer.flip();
      batchBuffer.clear();

      // Decompress into batchBuffer; position after decompression reflects how many bytes were written
      switch (compressionType)
      {
         case ZSTD ->
         {
            long result = Zstd.decompressByteArray(batchBuffer.array(), 0, batchBuffer.capacity(),
                                                   compressedBuffer.array(), compressedBuffer.position(), compressedBuffer.remaining());
            if (Zstd.isError(result))
               throw new RuntimeException("Zstd decompression failed: " + Zstd.getErrorName(result));
            batchBuffer.position((int) result);
         }
         default ->
         {
            try
            {
               SnappyUtils.uncompress(compressedBuffer, batchBuffer);
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      }

      // Derive how many complete ticks were decompressed and reset the tick cursor
      currentBatchTickCount = batchBuffer.position() / singleTickSize;
      tickIndexInBatch = 0;
      index++;
      return true;
   }

   public long getTimestamp(int position)
   {
      if (!compressed)
      {
         throw new RuntimeException("Cannot get timestamp for non-compressed logs");
      }

      return logIndex.timestamps[position / batchSize];
   }

   // Reads a single tick into logLine and rewinds logLongArray so callers can read from the start.
   // Returns false if there is no more data to read.
   private boolean readLogLine() throws IOException
   {
      logLine.clear();
      logLongArray.clear();

      if (compressed)
      {
         // Current batch exhausted, load the next one
         if (tickIndexInBatch >= currentBatchTickCount)
         {
            if (!readNextBatch())
               return false;
         }

         int tickStart = tickIndexInBatch * singleTickSize;
         batchBuffer.limit(tickStart + singleTickSize);
         batchBuffer.position(tickStart);
         logLine.put(batchBuffer);
         tickIndexInBatch++;

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

   // Converts a timestamp to a tick position; the index seek returns a batch index, so multiply by batchSize to get the first tick of that batch
   public int getPosition(long timestamp)
   {
      return logIndex.seek(timestamp) * batchSize;
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

   public List<YoVariable> getYoVariablesList()
   {
      return yoVariables;
   }

   public int getCurrentLogPosition()
   {
      return currentRecordTick.getValue();
   }

   public YoRegistry getLocalYoRegistry()
   {
      return registry;
   }

   public YoRegistry getLogRootRegistry()
   {
      return parser.getRootRegistry();
   }

   public List<YoGraphicGroupDefinition> getLogSCS2YoGraphics()
   {
      return parser.getSCS2YoGraphics();
   }

   public List<JointState> getJointStates()
   {
      return jointStates;
   }

   private enum CompressionType
   {
      NONE, SNAPPY, ZSTD;

      public static CompressionType fromString(String value)
      {
         return switch (value.trim().toLowerCase())
         {
            case "", "none" -> NONE;
            case "snappy" -> SNAPPY;
            case "zstd" -> ZSTD;
            default -> throw new IllegalArgumentException("Unsupported compression type: " + value);
         };
      }
   }
}
