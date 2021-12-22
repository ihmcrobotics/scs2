package us.ihmc.scs2.examples.yoServer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Parameter;
import com.martiansoftware.jsap.SimpleJSAP;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryRandomTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoServerExample
{
   private static final double DEFAULT_DT = 0.001;
   private static final int DEFAULT_SIZE = 10000;
   private static final int DEFAULT_NUMBER_OF_SECONDARY_REGISTRIES = 1;

   private static final int mainRegistryUpdatesPerSecondRegistryUpdates = 10;

   public YoServerExample(double dt, int numberOfYoVariables, int numberOfSecondaryRegistries)
   {
      Random random = new Random(523453);
      YoVariableServer yoServer = new YoVariableServer(getClass(), null, new DataServerSettings(false), dt);

      YoRegistry mainRegistry = new YoRegistry("MainRegistry");
      YoVariable[] mainVariables = SharedMemoryRandomTools.nextYoVariables(random, "main_", numberOfYoVariables, mainRegistry);
      yoServer.setMainRegistry(mainRegistry, null);

      YoRegistry[] secondaryRegistries = new YoRegistry[numberOfSecondaryRegistries];
      List<YoVariable> secondaryVariables = new ArrayList<>();

      for (int i = 0; i < numberOfSecondaryRegistries; i++)
      {
         secondaryRegistries[i] = new YoRegistry("SecondaryRegistry" + i);
         secondaryVariables.addAll(Arrays.asList(SharedMemoryRandomTools.nextYoVariables(random,
                                                                                         "secondary" + i + "_",
                                                                                         numberOfYoVariables,
                                                                                         secondaryRegistries[i])));
         yoServer.addRegistry(secondaryRegistries[i], null);
      }

      yoServer.start();

      ScheduledExecutorService scheduledThreadExecutor = Executors.newScheduledThreadPool(1,
                                                                                          ThreadTools.createNamedThreadFactory(getClass().getSimpleName()
                                                                                                + " - Main"));
      long startTimestamp = System.nanoTime();

      scheduledThreadExecutor.scheduleAtFixedRate(new Runnable()
      {
         int mainRegistryPublishCounter = 0;

         @Override
         public void run()
         {
            long currentTimestamp = System.nanoTime();
            SharedMemoryRandomTools.randomizeYoVariables(random, mainVariables);
            yoServer.update(currentTimestamp - startTimestamp);

            mainRegistryPublishCounter++;

            if (mainRegistryPublishCounter >= mainRegistryUpdatesPerSecondRegistryUpdates)
            {
               mainRegistryPublishCounter = 0;
               SharedMemoryRandomTools.randomizeYoVariables(random, secondaryVariables);
               for (YoRegistry secondaryRegistry : secondaryRegistries)
               {
                  yoServer.update(currentTimestamp - startTimestamp, secondaryRegistry);
               }
            }
         }
      }, 0, Conversions.secondsToNanoseconds(dt), TimeUnit.NANOSECONDS);
   }

   public static void main(String[] args) throws JSAPException
   {
      SimpleJSAP jsap = new SimpleJSAP("YoServerExample",
                                       "Starts a YoVariableServer",
                                       new Parameter[] {
                                             new FlaggedOption("publishPeriod",
                                                               JSAP.DOUBLE_PARSER,
                                                               String.valueOf(DEFAULT_DT),
                                                               JSAP.NOT_REQUIRED,
                                                               't',
                                                               "dt",
                                                               "Period at which data is published."),
                                             new FlaggedOption("size",
                                                               JSAP.INTEGER_PARSER,
                                                               String.valueOf(DEFAULT_SIZE),
                                                               JSAP.NOT_REQUIRED,
                                                               's',
                                                               "size",
                                                               "Number of YoVariables to be generated."),
                                             new FlaggedOption("secondaryRegistries",
                                                               JSAP.INTEGER_PARSER,
                                                               String.valueOf(DEFAULT_NUMBER_OF_SECONDARY_REGISTRIES),
                                                               JSAP.NOT_REQUIRED,
                                                               'r',
                                                               "secondReg",
                                                               "Number of secondary registries to be created.")});

      JSAPResult config = jsap.parse(args);
      if (jsap.messagePrinted())
      {
         System.out.println(jsap.getUsage());
         System.out.println(jsap.getHelp());
         System.exit(-1);
      }

      new YoServerExample(config.getDouble("publishPeriod"), config.getInt("size"), config.getInt("secondaryRegistries"));
   }
}
