package us.ihmc.robotDataLogger.websocket;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotDataLogger.YoMCAPVariableClient;
import us.ihmc.robotDataLogger.YoMCAPVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.client.discovery.WebsocketMCAPStarter;
import us.ihmc.robotDataLogger.websocket.dataBuffers.ConnectionStateListener;
import us.ihmc.scs2.session.mcap.encoding.CDRDeserializer;
import us.ihmc.scs2.session.mcap.specs.records.Channel;
import us.ihmc.scs2.session.mcap.specs.records.Chunk;
import us.ihmc.scs2.session.mcap.specs.records.Message;
import us.ihmc.scs2.session.mcap.specs.records.Opcode;
import us.ihmc.scs2.session.mcap.specs.records.Record;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.jupiter.api.Assertions.*;

public class YoMCAPServerClientTest
{
   @Test
   public void testSimpleEndToEnd() throws InterruptedException
   {
      Random random = new Random(1234);
      YoRegistry registry = newRegistry(random);
      DataServerSettings dataServerSettings = new DataServerSettings(false);
      YoMCAPVariableServer server = new YoMCAPVariableServer(getClass().getSimpleName(), null, dataServerSettings, 0.001);
      server.setMainRegistry(registry, null);
      server.start();

      AtomicLong lastTimestamp = new AtomicLong(-1);

      Thread testPublisherThread = new Thread(new Runnable()
      {
         private long initialTimestamp;

         @Override
         public void run()
         {
            if (initialTimestamp == 0)
               initialTimestamp = System.nanoTime();

            long timestamp = -1;
            int iterations = 30;
            // Wait for the client to connect
            ThreadTools.sleep(500);

            for (int i = 0; i < iterations; i++)
            {
               randomize(random, registry);
               timestamp = System.nanoTime() - initialTimestamp;

               if (i == iterations - 1)
                  lastTimestamp.set(timestamp);

               server.update(timestamp);
               ThreadTools.sleep(10);
            }

            System.out.println("Last timestamp: " + lastTimestamp.longValue() + " stopping server");
            server.close();
         }
      });
      testPublisherThread.start();

      MutableObject<WebsocketMCAPStarter> mcapStarterMutableObject = new MutableObject<>();
      List<Chunk> chunks = new ArrayList<>();
      MutableBoolean connected = new MutableBoolean(false);
      MutableBoolean connectionClosed = new MutableBoolean(false);

      YoMCAPVariableClient client = new YoMCAPVariableClient();
      //      client.setTimestampListener((timestamp) -> System.out.println("Timestamp: " + timestamp));
      client.setStarterMCAPConsumer(mcapStarter ->
                                    {
                                       assertEquals(registry.getNumberOfVariablesDeep(), mcapStarter.channelStarterChunk().records().size());
                                       checkMCAPStarter(mcapStarter);
                                       mcapStarterMutableObject.setValue(mcapStarter);
                                    });
      client.setRecordConsumer(((timestamp, newRecord) ->
      {
         Chunk messageChunk = newRecord.body();
         chunks.add(messageChunk);
         System.out.println("Timestamp: " + timestamp + " Record timestamp: " + messageChunk.messageEndTime());
      }));
      client.setConnectionStateListener(new ConnectionStateListener()
      {
         @Override
         public void connected()
         {
            connected.setTrue();
         }

         @Override
         public void connectionClosed()
         {
            connectionClosed.setTrue();
         }
      });
      client.start("localhost", dataServerSettings.getPort());

      testPublisherThread.join();

      System.out.println("Publisher stopped");

      long timeout = System.currentTimeMillis() + 1000;

      while (client.isConnected())
      {
         if (System.currentTimeMillis() > timeout)
         {
            fail("Client did not disconnect");
         }
         ThreadTools.sleep(100);
      }
      assertTrue(connected.booleanValue());
      assertTrue(connectionClosed.booleanValue());

      assertEquals(30, chunks.size());
      assertEquals(lastTimestamp.longValue(), chunks.get(chunks.size() - 1).messageEndTime());

      WebsocketMCAPStarter mcapStarter = mcapStarterMutableObject.getValue();
      Chunk lastChunk = chunks.get(chunks.size() - 1);

      for (Record record : lastChunk.records())
      {
         assertEquals(Opcode.MESSAGE, record.op());
         assertTrue(Message.class.isAssignableFrom(record.body().getClass()));
         Message message = record.body();
         assertEquals(lastChunk.messageStartTime(), message.publishTime());
         int channelId = message.channelId();
         Channel channel = mcapStarter.channelStarterChunk().getChannel(channelId);
         assertNotNull(channel);
         assertEquals("cdr", channel.messageEncoding());
         String variableNamespace = channel.topic().replaceAll("/", YoTools.NAMESPACE_SEPERATOR_STRING);
         YoVariable yoVariable = registry.findVariable(variableNamespace);
         assertNotNull(yoVariable);

         ByteBuffer messageDataBuffer = message.messageBuffer();
         CDRDeserializer deserializer = new CDRDeserializer();
         deserializer.initialize(messageDataBuffer);

         if (yoVariable instanceof YoBoolean yoBoolean)
         {
            assertEquals(yoBoolean.getValue(), deserializer.read_bool());
         }
         else if (yoVariable instanceof YoDouble yoDouble)
         {
            assertEquals(yoDouble.getValue(), deserializer.read_float64());
         }
         else if (yoVariable instanceof YoInteger yoInteger)
         {
            assertEquals(yoInteger.getValue(), deserializer.read_int32());
         }
         else if (yoVariable instanceof YoLong yoLong)
         {
            assertEquals(yoLong.getValue(), deserializer.read_int64());
         }
         else if (yoVariable instanceof YoEnum<?> yoEnum)
         {
            assertEquals(yoEnum.getOrdinal(), deserializer.read_uint8());
         }
         else
         {
            throw new RuntimeException("Unknown type: " + yoVariable.getClass().getSimpleName());
         }
      }
   }

   public static YoRegistry newRegistry(Random random)
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble testDouble = new YoDouble("testDouble", registry);
      testDouble.set(EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0));
      YoBoolean testBoolean = new YoBoolean("testBoolean", registry);
      testBoolean.set(random.nextBoolean());
      YoInteger testInteger = new YoInteger("testInteger", registry);
      testInteger.set(random.nextInt());
      YoLong testLong = new YoLong("testLong", registry);
      testLong.set(random.nextLong());
      YoEnum<Axis3D> testEnum = new YoEnum<>("testEnum", registry, Axis3D.class);
      testEnum.set(random.nextBoolean() ? Axis3D.X : Axis3D.Y);
      return registry;
   }

   private static void randomize(Random random, YoRegistry start)
   {
      for (int i = 0; i < start.getVariables().size(); i++)
      {
         YoVariable yoVariable = start.getVariables().get(i);

         if (yoVariable instanceof YoBoolean yoBoolean)
            yoBoolean.set(random.nextBoolean());
         else if (yoVariable instanceof YoDouble yoDouble)
            yoDouble.set(EuclidCoreRandomTools.nextDouble(random, 0.0, 100.0));
         else if (yoVariable instanceof YoInteger yoInteger)
            yoInteger.set(random.nextInt());
         else if (yoVariable instanceof YoLong yoLong)
            yoLong.set(random.nextLong());
         else if (yoVariable instanceof YoEnum<?> yoEnum)
            yoEnum.set(random.nextInt(yoEnum.getEnumSize()));
         else
            throw new RuntimeException("Unknown type: " + yoVariable.getClass().getSimpleName());
      }
   }

   /**
    * This test is used to verify that the MCAP starter is correctly parsed.
    * <p>
    * This is more like a simple regression test.
    * </p>
    */
   private static void checkMCAPStarter(WebsocketMCAPStarter mcapStarter)
   {
      String expected = """
            WebsocketMCAPStarter{
            header=
            	Header: 
            		-profile = us.ihmc.mcap-starter
            		-library = version 1.0,
            announcementMetadata=
            	WebsocketAnnouncementMetadata: 
            		-name = announcement
            		-metadata = serverName, serverVersion, isLoggingSession, hostName, port, hasRobotModel, hasResources,
            schemaStarterChunk=
            	WebsocketSchemaStarterChunk:
            		-messageStartTime = -11
            		-messageEndTime = -10
            		-compression = NONE
            		-recordsUncompressedLength = 350
            		-uncompressedCrc32 = 1234736452
            		-records =  
            			RecordDataInputBacked:
            				-op = SCHEMA
            				-bodyLength = 44
            				-bodyOffset = 339
            				-body = 
            							SchemaDataInputBacked:
            								-id = 0
            								-name = YoBoolean
            								-encoding = ros2msg
            								-dataLength = 14
            								-data = [10, 0, 0, 0, 98, 111, 111, 108, 32, 118, 97, 108, 117, 101]
            			RecordDataInputBacked:
            				-op = SCHEMA
            				-bodyLength = 46
            				-bodyOffset = 392
            				-body = 
            							SchemaDataInputBacked:
            								-id = 1
            								-name = YoDouble
            								-encoding = ros2msg
            								-dataLength = 17
            								-data = [13, 0, 0, 0, 102, 108, 111, 97, 116, 54, 52, 32, 118, 97, 108, 117, 101]
            			RecordDataInputBacked:
            				-op = SCHEMA
            				-bodyLength = 45
            				-bodyOffset = 447
            				-body = 
            							SchemaDataInputBacked:
            								-id = 3
            								-name = YoInteger
            								-encoding = ros2msg
            								-dataLength = 15
            								-data = [11, 0, 0, 0, 105, 110, 116, 51, 50, 32, 118, 97, 108, 117, 101]
            			RecordDataInputBacked:
            				-op = SCHEMA
            				-bodyLength = 42
            				-bodyOffset = 501
            				-body = 
            							SchemaDataInputBacked:
            								-id = 2
            								-name = YoLong
            								-encoding = ros2msg
            								-dataLength = 15
            								-data = [11, 0, 0, 0, 105, 110, 116, 54, 52, 32, 118, 97, 108, 117, 101]
            			RecordDataInputBacked:
            				-op = SCHEMA
            				-bodyLength = 128
            				-bodyOffset = 552
            				-body = 
            							SchemaDataInputBacked:
            								-id = 4
            								-name = YoEnum
            								-encoding = ros2msg
            								-dataLength = 101
            								-data = [97, 0, 0, 0, 117, 105, 110, 116, 56, 32, 118, 97, 108, 117, 101, 13, 10, 35, 32, 84, 79, 68, 79, 32, 84, 104, 101, 32, 99, 111, 110, 115, 116, 97, 110, 116, 115, 32, 115, 104, 111, 117, 108, 100, 110, 39, 116, 32, 104, 97, 118, 101, 32, 116, 111, 32, 98, 101, 32, 112, 117, 98, 108, 105, 115, 104, 101, 100, 32, 97, 108, 108, 32, 116, 104, 101, 32, 116, 105, 109, 101, 13, 10, 115, 116, 114, 105, 110, 103, 91, 93, 32, 99, 111, 110, 115, 116, 97, 110, 116, 115],
            channelStarterChunk=
            	WebsocketChannelStarterChunk:
            		-messageStartTime = -21
            		-messageEndTime = -20
            		-compression = NONE
            		-recordsUncompressedLength = 417
            		-uncompressedCrc32 = 1156307734
            		-records = 
            			RecordDataInputBacked:
            				-op = CHANNEL
            				-bodyLength = 42
            				-bodyOffset = 738
            				-body = 
            							ChannelDataInputBacked:
            								-id = 0
            								-schemaId = 1
            								-topic = testRegistry/testDouble
            								-messageEncoding = cdr
            								-metadata = [{}]
            			RecordDataInputBacked:
            				-op = CHANNEL
            				-bodyLength = 43
            				-bodyOffset = 789
            				-body = 
            							ChannelDataInputBacked:
            								-id = 1
            								-schemaId = 0
            								-topic = testRegistry/testBoolean
            								-messageEncoding = cdr
            								-metadata = [{}]
            			RecordDataInputBacked:
            				-op = CHANNEL
            				-bodyLength = 43
            				-bodyOffset = 841
            				-body = 
            							ChannelDataInputBacked:
            								-id = 2
            								-schemaId = 3
            								-topic = testRegistry/testInteger
            								-messageEncoding = cdr
            								-metadata = [{}]
            			RecordDataInputBacked:
            				-op = CHANNEL
            				-bodyLength = 40
            				-bodyOffset = 893
            				-body = 
            							ChannelDataInputBacked:
            								-id = 3
            								-schemaId = 2
            								-topic = testRegistry/testLong
            								-messageEncoding = cdr
            								-metadata = [{}]
            			RecordDataInputBacked:
            				-op = CHANNEL
            				-bodyLength = 40
            				-bodyOffset = 942
            				-body = 
            							ChannelDataInputBacked:
            								-id = 4
            								-schemaId = 4
            								-topic = testRegistry/testEnum
            								-messageEncoding = cdr
            								-metadata = [{}]
            			RecordDataInputBacked:
            				-op = CHANNEL
            				-bodyLength = 70
            				-bodyOffset = 991
            				-body = 
            							ChannelDataInputBacked:
            								-id = 5
            								-schemaId = 3
            								-topic = testRegistry/LoggerDebugRegistry/FullCircularBuffer
            								-messageEncoding = cdr
            								-metadata = [{}]
            			RecordDataInputBacked:
            				-op = CHANNEL
            				-bodyLength = 76
            				-bodyOffset = 1070
            				-body = 
            							ChannelDataInputBacked:
            								-id = 6
            								-schemaId = 3
            								-topic = testRegistry/LoggerDebugRegistry/lostTickInCircularBuffer
            								-messageEncoding = cdr
            								-metadata = [{}],
            robotModelAttachment=
            null,
            resourcesAttachment=
            null,
            dataEnd=
            	DataEnd:
            		-dataSectionCrc32 = 0,
            footer=
            	Footer:
            		-ofsSummarySection = 0
            		-ofsSummaryOffsetSection = 0
            		-summaryCrc32 = 0
            }""".replaceAll(" ", "");
      assertEquals(expected, mcapStarter.toString().replaceAll(" ", ""));
   }
}
