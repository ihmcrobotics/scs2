package us.ihmc.robotDataLogger.websocket;

import org.junit.jupiter.api.Test;
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

      YoMCAPVariableClient client = new YoMCAPVariableClient();
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
}
