package us.ihmc.scs2.session.mcap;

import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.mcap.MCAP.Channel;
import us.ihmc.scs2.session.mcap.MCAP.Message;
import us.ihmc.scs2.session.mcap.MCAP.Opcode;
import us.ihmc.scs2.session.mcap.MCAP.Record;
import us.ihmc.scs2.session.mcap.MCAP.Schema;
import us.ihmc.scs2.session.mcap.MCAPBufferedChunk.ChunkBundle;
import us.ihmc.scs2.simulation.SpyList;

import java.io.IOException;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class MCAPConsoleLogManager
{
   public static final String FOXGLOVE_LOG = "foxglove::Log";
   private final long desiredLogDT;

   public enum MCAPLogLevel
   {UNKNOWN, DEBUG, INFO, WARNING, ERROR, FATAL}

   private final SpyList<MCAPConsoleLogItem> allConsoleLogItems = new SpyList<>();

   private int channelId = -1;

   public MCAPConsoleLogManager(MCAP mcap, MCAPBufferedChunk chunkBuffer, long desiredLogDT) throws IOException
   {
      this.desiredLogDT = desiredLogDT;
      Optional<Schema> logSchema = mcap.records()
                                       .stream()
                                       .filter(record -> record.op() == Opcode.SCHEMA)
                                       .map(record -> (Schema) record.body())
                                       .filter(schema -> schema.name().equals(FOXGLOVE_LOG))
                                       .findFirst();

      if (logSchema.isPresent())
      {
         int id = logSchema.get().id();

         Optional<Channel> logChannel = mcap.records()
                                            .stream()
                                            .filter(record -> record.op() == Opcode.CHANNEL)
                                            .map(record -> (Channel) record.body())
                                            .filter(channel -> channel.schemaId() == id)
                                            .findFirst();
         if (logChannel.isPresent())
         {
            if (!"cdr".equalsIgnoreCase(logChannel.get().messageEncoding()))
            {
               LogTools.error("Unsupported message encoding: {}", logChannel.get().messageEncoding());
               return;
            }

            channelId = logChannel.get().id();

            Thread loadingThread = createLoadingThread(chunkBuffer, desiredLogDT);
            loadingThread.start();
         }
      }
   }

   private Thread createLoadingThread(MCAPBufferedChunk chunkBuffer, long desiredLogDT)
   {
      Runnable loadingTask = () ->
      {
         int counter = 0;
         for (ChunkBundle bundle : chunkBuffer.getChunkBundles())
         {
            bundle.requestLoadChunkBundle(true, false, false);

            List<MCAPConsoleLogItem> orderedItems = new ArrayList<>();

            for (Record record : bundle.getChunkRecords())
            {
               if (record.op() != Opcode.MESSAGE)
                  continue;
               Message message = (Message) record.body();
               if (message.channelId() == channelId)
                  orderedItems.add(parseLogItem(message, desiredLogDT));
            }

            counter++;
            if (counter % (chunkBuffer.getChunkBundles().length / 10) == 0 || counter == chunkBuffer.getChunkBundles().length - 1)
               LogTools.info("Loaded {} log items from chunk bundle {}/{}", orderedItems.size(), counter, chunkBuffer.getChunkBundles().length);
            orderedItems.sort(Comparator.comparingLong(MCAPConsoleLogItem::logTime));
            allConsoleLogItems.addAll(orderedItems);
         }
      };
      Thread loadingThread = new Thread(loadingTask, getClass().getSimpleName() + "-LoadingThread");
      loadingThread.setDaemon(true);
      return loadingThread;
   }

   public SpyList<MCAPConsoleLogItem> getAllConsoleLogItems()
   {
      return allConsoleLogItems;
   }

   private static MCAPConsoleLogItem parseLogItem(Message message, long desiredLogDT)
   {
      /*
       * @formatter:off
       * Expected schema:
       * struct type_010f744a0100bb8d02000000_030c3_1
       * {
       *     unsigned long sec;
       *     unsigned long nsec;
       * };
       *
       * struct foxglove::Log
       * {
       *     type_010f744a0100bb8d02000000_030c3_1 instant;
       *     octet level;
       *     string message;
       *     string name;
       *     string file;
       *     unsigned long line;
       * };
       * @formatter:on
       */
      long logTime = MCAPMessageManager.round(message.logTime(), desiredLogDT);
      CDRDeserializer deserializer = new CDRDeserializer();
      deserializer.initialize(message.messageBuffer(), 0, message.dataLength());
      Instant instant = Instant.ofEpochSecond(deserializer.read_uint32(), deserializer.read_uint32());
      MCAPLogLevel logLevel = MCAPLogLevel.values()[deserializer.read_uint8()];
      String logMessage = deserializer.read_string();
      String processName = deserializer.read_string();
      String filename = deserializer.read_string();
      long lineNumberInFile = deserializer.read_uint32();
      deserializer.finalize(true);
      return new MCAPConsoleLogItem(logTime, instant, logLevel, logMessage, processName, filename, lineNumberInFile);
   }

   public record MCAPConsoleLogItem(long logTime, Instant instant, MCAPLogLevel logLevel, String message, String processName, String filename,
                                    long lineNumberInFile)
   {
   }
}
