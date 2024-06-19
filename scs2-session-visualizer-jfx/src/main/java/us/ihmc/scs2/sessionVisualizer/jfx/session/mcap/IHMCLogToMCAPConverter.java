package us.ihmc.scs2.sessionVisualizer.jfx.session.mcap;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.session.mcap.output.MCAPDataOutput;
import us.ihmc.scs2.session.mcap.specs.records.DataEnd;
import us.ihmc.scs2.session.mcap.specs.records.MCAPBuilder2;
import us.ihmc.scs2.session.mcap.specs.records.MCAPSummaryBuilder;
import us.ihmc.scs2.session.mcap.specs.records.Magic;
import us.ihmc.scs2.session.mcap.specs.records.MutableChunk;
import us.ihmc.scs2.session.mcap.specs.records.MutableMessage;
import us.ihmc.scs2.session.mcap.specs.records.MutableRecord;
import us.ihmc.yoVariables.registry.YoRegistry;
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
   private static final boolean WRITE_MESSAGE_INDEX_SUMMARY = true;

   private final List<YoRegistry> yoRegistries;
   private final List<YoVariable> yoVariables;
   private final LogDataReader logDataReader;
   private final MCAPDataOutput dataOutput;
   private final MCAPBuilder2 mcapBuilder = new MCAPBuilder2();
   private final MCAPSummaryBuilder summaryBuilder = new MCAPSummaryBuilder(true);

   private final double chunkDuration = 0.10;

   IHMCLogToMCAPConverter(File inputIHMCLogFile, File outputMCAPFile, double chunkDuration) throws IOException, InterruptedException
   {
      dataOutput = MCAPDataOutput.wrap(new FileOutputStream(outputMCAPFile).getChannel());
      logDataReader = new LogDataReader(inputIHMCLogFile.getParentFile(), null);

      dataOutput.putBytes(Magic.MAGIC_BYTES); // header magic
      yoVariables = logDataReader.getParser().getYoVariablesList();
      yoRegistries = logDataReader.getParser().getRegistries().stream().filter(r -> r.getNumberOfVariables() > 0).toList();
      yoRegistries.forEach(mcapBuilder::addRegistryRecursively);
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
      YoRegistryVariableValues[] chunkSavedState = null;
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
         }

         if (chunkStartTime == -1)
         {
            chunkStartTime = currentTimestamp;
            currentChunkRecords.clear();
         }

         for (int i = 0; i < yoRegistries.size(); i++)
         {
            YoRegistry yoRegistry = yoRegistries.get(i);
            if (currentTimestamp != chunkStartTime && chunkSavedState != null && chunkSavedState[i] != null)
            {
               YoRegistryVariableValues variableValues = chunkSavedState[i];
               boolean hasChanged = false;

               for (int j = 0; j < yoRegistry.getVariables().size(); j++)
               {
                  YoVariable yoVariable = yoRegistry.getVariables().get(j);
                  long valueAsLongBits = yoVariable.getValueAsLongBits();
                  if (valueAsLongBits != variableValues.variableValues()[j])
                  {
                     variableValues.variableValues()[j] = valueAsLongBits;
                     hasChanged = true;
                  }
               }

               if (!hasChanged)
                  continue;
            }

            MutableMessage message = new MutableMessage();
            message.setLogTime(chunkStartTime);
            message.setPublishTime(chunkStartTime);
            message.setSequence(index);
            mcapBuilder.packRegistryMessage(yoRegistry, message);
            MutableRecord messageRecord = new MutableRecord(message);
            currentChunkRecords.add(messageRecord);
            if (WRITE_SUMMARY)
               summaryBuilder.update(messageRecord);
         }

         if (currentTimestamp != chunkStartTime)
         {
            if (chunkSavedState == null)
               chunkSavedState = new YoRegistryVariableValues[yoRegistries.size()];

            for (int i = 0; i < yoRegistries.size(); i++)
            {
               YoRegistry yoRegistry = yoRegistries.get(i);

               if (chunkSavedState[i] == null)
                  chunkSavedState[i] = new YoRegistryVariableValues(yoRegistry, new long[yoRegistry.getVariables().size()]);

               for (int j = 0; j < yoRegistry.getVariables().size(); j++)
               {
                  YoVariable yoVariable = yoRegistry.getVariables().get(j);
                  chunkSavedState[i].variableValues()[j] = yoVariable.getValueAsLongBits();
               }
            }
         }
      }

      enableWriterThread.set(false);
      writerThread.join();
   }

   private record YoRegistryVariableValues(YoRegistry registry, long[] variableValues)
   {
   }
}
