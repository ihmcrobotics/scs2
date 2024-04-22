package us.ihmc.robotDataLogger.websocket;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotDataLogger.YoMCAPVariableClient;
import us.ihmc.robotDataLogger.YoMCAPVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.websocket.dataBuffers.ConnectionStateListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.Random;

public class YoMCAPServerClientTest
{
   @Test
   public void testSimpleEndToEnd()
   {
      Random random = new Random(1234);
      YoRegistry registry = newRegistry(random);
      DataServerSettings dataServerSettings = new DataServerSettings(false);
      YoMCAPVariableServer server = new YoMCAPVariableServer(getClass().getSimpleName(), null, dataServerSettings, 0.001);
      server.setMainRegistry(registry, null);
      server.start();

      new Thread(new Runnable()
      {
         private long initialTimestamp;

         @Override
         public void run()
         {
            if (initialTimestamp == 0)
               initialTimestamp = System.nanoTime();
            for (int i = 0; i < 5000; i++)
            {
               randomize(random, registry);
               long timestamp = System.nanoTime() - initialTimestamp;
               server.update(timestamp);
               ThreadTools.sleep(1);
            }
         }
      }).start();

      YoMCAPVariableClient client = new YoMCAPVariableClient();
      //      client.setTimestampListener((timestamp) -> System.out.println("Timestamp: " + timestamp));
      client.setStarterMCAPConsumer((mcap) -> System.out.println("Received starter MCAP: " + mcap));
      //      client.setRecordConsumer(((timestamp, newRecord) -> System.out.println("Timestamp: " + timestamp + " Record: " + newRecord)));
      client.setConnectionStateListener(new ConnectionStateListener()
      {
         @Override
         public void connected()
         {
            System.out.println("Connected");
         }

         @Override
         public void connectionClosed()
         {
            System.out.println("Connection closed");
         }
      });
      client.start("localhost", dataServerSettings.getPort());

      ThreadTools.sleepForever();
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
}
