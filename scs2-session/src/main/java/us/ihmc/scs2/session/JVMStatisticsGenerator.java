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
   private final YoRegistry registry = new YoRegistry("JVMStatistics");

   private final YoTimer timer = new YoTimer("jvmStatsTimer", TimeUnit.MILLISECONDS, registry);

   private final YoLong freeMemory = new YoLong("freeMemoryInBytes", registry);
   private final YoLong maxMemory = new YoLong("maxMemoryInBytes", registry);
   private final YoLong usedMemory = new YoLong("usedMemoryInBytes", registry);
   private final YoLong totalMemory = new YoLong("totalMemoryInBytes", registry);

   private final YoLong totalGCInvocations = new YoLong("totalGCInvocations", registry);
   private final YoLong totalGCTotalCollectionTimeMs = new YoLong("gcTotalCollectionTimeMs", registry);

   private final YoInteger loadedClassCount = new YoInteger("loadedClassCount", registry);
   private final YoLong totalLoadedClassCount = new YoLong("totalLoadedClassCount", registry);
   private final YoLong unloadedClassCount = new YoLong("unloadedClassCount", registry);

   private final YoLong totalCompilationTime = new YoLong("totalCompilationTimeMs", registry);

   private final YoInteger availableProcessors = new YoInteger("availableProcessors", registry);
   private final YoDouble systemLoadAverage = new YoDouble("systemLoadAverage", registry);

   private final ArrayList<GCBeanHolder> gcBeanHolders = new ArrayList<>();
   private final ClassLoadingMXBean classLoadingMXBean = ManagementFactory.getClassLoadingMXBean();
   private final CompilationMXBean compilationMXBean = ManagementFactory.getCompilationMXBean();
   private final OperatingSystemMXBean operatingSystemMXBean = ManagementFactory.getOperatingSystemMXBean();

   public JVMStatisticsGenerator(YoRegistry parentRegistry)
   {
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
