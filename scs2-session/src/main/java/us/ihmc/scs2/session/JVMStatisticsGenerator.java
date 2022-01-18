package us.ihmc.scs2.session;

import java.lang.management.ClassLoadingMXBean;
import java.lang.management.CompilationMXBean;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.OperatingSystemMXBean;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public class JVMStatisticsGenerator
{
   private final YoRegistry registry;

   private final YoTimer timer;

   private final YoLong freeMemory;
   private final YoLong maxMemory;
   private final YoLong usedMemory;
   private final YoLong totalMemory;

   private final YoLong totalGCInvocations;
   private final YoLong totalGCTotalCollectionTimeMs;

   private final YoInteger loadedClassCount;
   private final YoLong totalLoadedClassCount;
   private final YoLong unloadedClassCount;

   private final YoLong totalCompilationTime;

   private final YoInteger availableProcessors;
   private final YoDouble systemLoadAverage;

   private final ArrayList<GCBeanHolder> gcBeanHolders = new ArrayList<>();
   private final ClassLoadingMXBean classLoadingMXBean = ManagementFactory.getClassLoadingMXBean();
   private final CompilationMXBean compilationMXBean = ManagementFactory.getCompilationMXBean();
   private final OperatingSystemMXBean operatingSystemMXBean = ManagementFactory.getOperatingSystemMXBean();

   public JVMStatisticsGenerator(String prefix, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(prefix + "SCS2JVMStatistics");

      timer = new YoTimer(prefix + "JVMStatsTimer", TimeUnit.MILLISECONDS, registry);

      freeMemory = new YoLong(prefix + "FreeMemoryInBytes", registry);
      maxMemory = new YoLong(prefix + "MaxMemoryInBytes", registry);
      usedMemory = new YoLong(prefix + "UsedMemoryInBytes", registry);
      totalMemory = new YoLong(prefix + "TotalMemoryInBytes", registry);

      totalGCInvocations = new YoLong(prefix + "TotalGCInvocations", registry);
      totalGCTotalCollectionTimeMs = new YoLong(prefix + "GCTotalCollectionTimeMs", registry);

      loadedClassCount = new YoInteger(prefix + "LoadedClassCount", registry);
      totalLoadedClassCount = new YoLong(prefix + "TotalLoadedClassCount", registry);
      unloadedClassCount = new YoLong(prefix + "UnloadedClassCount", registry);

      totalCompilationTime = new YoLong(prefix + "TotalCompilationTimeMs", registry);

      availableProcessors = new YoInteger(prefix + "AvailableProcessors", registry);
      systemLoadAverage = new YoDouble(prefix + "SystemLoadAverage", registry);

      createGCBeanHolders();

      availableProcessors.set(operatingSystemMXBean.getAvailableProcessors());
      maxMemory.set(Runtime.getRuntime().maxMemory());

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      timer.start();
      updateGCStatistics();
      updateClassLoadingStatistics();
      updateMemoryUsageStatistics();

      if (compilationMXBean != null)
      {
         totalCompilationTime.set(compilationMXBean.getTotalCompilationTime());
      }

      systemLoadAverage.set(operatingSystemMXBean.getSystemLoadAverage());
      timer.stop();
   }

   private void updateMemoryUsageStatistics()
   {
      freeMemory.set(Runtime.getRuntime().freeMemory());
      totalMemory.set(Runtime.getRuntime().totalMemory());
      usedMemory.set(totalMemory.getLongValue() - freeMemory.getLongValue());
   }

   private void updateClassLoadingStatistics()
   {
      loadedClassCount.set(classLoadingMXBean.getLoadedClassCount());
      totalLoadedClassCount.set(classLoadingMXBean.getTotalLoadedClassCount());
      unloadedClassCount.set(classLoadingMXBean.getUnloadedClassCount());
   }

   private void updateGCStatistics()
   {
      long totalInvocations = 0;
      long totalTime = 0;
      for (int i = 0; i < gcBeanHolders.size(); i++)
      {
         GCBeanHolder holder = gcBeanHolders.get(i);
         holder.update();

         totalInvocations += holder.gcInvocations.getLongValue();
         totalTime += holder.gcTotalCollectionTimeMs.getLongValue();
      }

      totalGCInvocations.set(totalInvocations);
      totalGCTotalCollectionTimeMs.set(totalTime);
   }

   private void createGCBeanHolders()
   {
      List<GarbageCollectorMXBean> gcbeans = java.lang.management.ManagementFactory.getGarbageCollectorMXBeans();
      //Install a notifcation handler for each bean
      for (int i = 0; i < gcbeans.size(); i++)
      {
         GarbageCollectorMXBean gcbean = gcbeans.get(i);
         String name = YoTools.ILLEGAL_CHARACTERS_PATTERN.matcher(gcbean.getName()).replaceAll("");
         gcBeanHolders.add(new GCBeanHolder(name, gcbean));
      }
   }

   private class GCBeanHolder
   {
      final GarbageCollectorMXBean gcBean;
      final YoLong gcInvocations;
      final YoLong gcTotalCollectionTimeMs;

      GCBeanHolder(String name, GarbageCollectorMXBean gcBean)
      {
         this.gcBean = gcBean;
         gcInvocations = new YoLong(name + "GCInvocations", registry);
         gcTotalCollectionTimeMs = new YoLong(name + "GCTotalTimeMs", registry);
      }

      void update()
      {
         gcInvocations.set(gcBean.getCollectionCount());
         gcTotalCollectionTimeMs.set(gcBean.getCollectionTime());
      }
   }
}
