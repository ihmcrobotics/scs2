package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.DataEnd;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder;
import us.ihmc.scs2.session.mcap.specs.records.MCAPSummaryBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;

public class IHMCLogToMCAPConverter
{
   private static final boolean WRITE_SUMMARY = true;
   private static final boolean WRITE_MESSAGE_INDEX_SUMMARY = false;

   private final List<YoVariable> yoVariables;
   private final LogDataReader logDataReader;
   private final MCAPDataOutput dataOutput;
   private final MCAPBuilder mcapBuilder = new MCAPBuilder();
   private final MCAPSummaryBuilder summaryBuilder = new MCAPSummaryBuilder();

   private final double chunkDuration = 0.10;

   IHMCLogToMCAPConverter(File inputIHMCLogFile, File outputMCAPFile, double chunkDuration) throws IOException, InterruptedException
   {
      dataOutput = MCAPDataOutput.wrap(new FileOutputStream(outputMCAPFile).getChannel());
      logDataReader = new LogDataReader(inputIHMCLogFile.getParentFile(), null);

      dataOutput.putBytes(Magic.MAGIC_BYTES); // header magic
      yoVariables = logDataReader.getParser().getYoVariablesList();
      yoVariables.forEach(mcapBuilder::getOrCreateVariableChannelRecord);
      mcapBuilder.getAllSchemas().forEach(record ->
                                          {
                                             record.write(dataOutput);
                                             if (WRITE_SUMMARY)
                                                summaryBuilder.update(record);
                                          });
      mcapBuilder.getAllChannels().forEach(record ->
                                           {
                                              record.write(dataOutput);
                                              if (WRITE_SUMMARY)
                                                 summaryBuilder.update(record);
                                           });

      writeData(chunkDuration);
      new MutableRecord(new DataEnd(0)).write(dataOutput);
      summaryBuilder.writeSummary(dataOutput);

      dataOutput.putBytes(Magic.MAGIC_BYTES);
      dataOutput.close();
   }

   private void writeData(double chunkDuration) throws InterruptedException
   {
      long chunkStartTime = -1;
      long[] chunkVariablePreviousValues = null;
      List<MutableRecord> currentChunkRecords = new ArrayList<>(5 * yoVariables.size());
      int index = 0;

      AtomicBoolean enableWriterThread = new AtomicBoolean(true);
      ConcurrentLinkedQueue<Runnable> writeOperations = new ConcurrentLinkedQueue<>();
      Thread writerThread = new Thread(() ->
                                       {
                                          while (enableWriterThread.get())
                                          {
                                             Runnable operation = writeOperations.poll();
                                             if (operation == null)
                                             {
                                                ThreadTools.sleep(10);
                                                continue;
                                             }

                                             operation.run();
                                          }
                                       });
      writerThread.start();

      while (!logDataReader.read())
      {
         if (index % 1000 == 0)
            System.out.println("Processing log entry " + index + " / " + logDataReader.getNumberOfEntries());
         index++;

         long currentTimestamp = logDataReader.getTimestamp().getValue();

         if (!currentChunkRecords.isEmpty() && currentTimestamp - chunkStartTime > chunkDuration * 1e9)
         {
            long chunkOffset = dataOutput.position();
            MutableChunk chunk = new MutableChunk();
            chunk.setRecords(currentChunkRecords);
            MutableRecord chunkRecord = new MutableRecord(chunk);
            List<MutableRecord> messageIndexRecords = chunk.records().generateMessageIndexList().stream().map(MutableRecord::new).toList();

            if (WRITE_SUMMARY)
            {
               summaryBuilder.update(chunkRecord);
               if (WRITE_MESSAGE_INDEX_SUMMARY)
                  summaryBuilder.update(chunkRecord.generateChunkIndexRecord(chunkOffset, messageIndexRecords));
            }

            writeOperations.add(() ->
                                {
                                   chunkRecord.write(dataOutput);
                                   messageIndexRecords.forEach(r -> r.write(dataOutput));
                                });

            chunkStartTime = -1;
            chunkVariablePreviousValues = null;
         }

         if (chunkStartTime == -1)
         {
            chunkVariablePreviousValues = null;
            chunkStartTime = currentTimestamp;
            currentChunkRecords.clear();
         }

         for (int i = 0; i < yoVariables.size(); i++)
         {
            YoVariable yoVariable = yoVariables.get(i);
            if (chunkVariablePreviousValues != null)
            {
               long valueAsLongBits = yoVariable.getValueAsLongBits();
               if (valueAsLongBits == chunkVariablePreviousValues[i])
                  continue; // Skipping variables that have not changed
               chunkVariablePreviousValues[i] = valueAsLongBits;
            }

            MutableMessage message = new MutableMessage();
            message.setLogTime(chunkStartTime);
            message.setPublishTime(chunkStartTime);
            message.setSequence(index);
            mcapBuilder.packVariableMessage(yoVariable, message);
            MutableRecord messageRecord = new MutableRecord(message);
            currentChunkRecords.add(messageRecord);
            if (WRITE_SUMMARY)
               summaryBuilder.update(messageRecord);
         }

         if (chunkVariablePreviousValues == null)
            chunkVariablePreviousValues = yoVariables.stream().mapToLong(YoVariable::getValueAsLongBits).toArray();
      }

      enableWriterThread.set(false);
      writerThread.join();
   }
}
