package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.charts.YoVariableChartData;
import us.ihmc.scs2.sharedMemory.LinkedYoVariable;
import us.ihmc.yoVariables.variable.YoVariable;

/*
 * FIXME The user should be able to pan the charts without modifying the buffer index. This would
 * involve allowing the buffer index to be outside the view when "desired".
 */
public class ChartDataManager implements Manager
{
   private final Map<YoVariable, LinkedYoVariable<?>> linkedVariableMap = new HashMap<>();
   private final Map<YoVariable, YoVariableChartData> chartDataMap = new ConcurrentHashMap<>();
   private final SessionVisualizerTopics topics;
   private final JavaFXMessager messager;
   private final YoManager yoManager;
   private final BackgroundExecutorManager backgroundExecutorManager;

   private Future<?> activeTask;

   public ChartDataManager(JavaFXMessager messager, SessionVisualizerTopics topics, YoManager yoManager, BackgroundExecutorManager backgroundExecutorManager)
   {
      this.topics = topics;
      this.messager = messager;
      this.yoManager = yoManager;
      this.backgroundExecutorManager = backgroundExecutorManager;
   }

   @Override
   public void startSession(Session session)
   {
      activeTask = backgroundExecutorManager.scheduleTaskInBackground(this::updateData, 0, 100, TimeUnit.MILLISECONDS);
   }

   @Override
   public void stopSession()
   {
      if (activeTask != null)
         activeTask.cancel(true);
      activeTask = null;
   }

   @Override
   public boolean isSessionLoaded()
   {
      return true;
   }

   private void updateData()
   {
      try
      {
         chartDataMap.values().removeIf(value ->
         {
            boolean remove = !value.isCurrentlyInUse();
            if (remove)
            {
               value.dispose();
               linkedVariableMap.remove(value.getYoVariable());
            }
            return remove;
         });
         chartDataMap.values().forEach(YoVariableChartData::updateBufferData);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public YoVariableChartData getYoVariableChartData(Object callerID, YoVariable yoVariable)
   {
      YoVariableChartData yoVariableChartData = chartDataMap.get(yoVariable);
      if (yoVariableChartData == null)
      {
         LinkedYoVariable<?> linkedYoVariable = linkedVariableMap.get(yoVariable);

         if (linkedYoVariable == null)
         {
            linkedYoVariable = yoManager.newLinkedYoVariable(yoVariable, this); // Make `this` a temporary user to indicate the linked variable is used while setting it up.
            linkedVariableMap.put(yoVariable, linkedYoVariable);
            yoVariableChartData = new YoVariableChartData(messager, topics, linkedYoVariable);
            linkedYoVariable.removeUser(this); // YoVariableChartData adds itself as user, we don't want `this` to be a user.
         }
         else
         {
            yoVariableChartData = new YoVariableChartData(messager, topics, linkedYoVariable);
         }

         yoVariableChartData.registerCaller(callerID);
         chartDataMap.put(yoVariable, yoVariableChartData);
      }
      else
      {
         yoVariableChartData.registerCaller(callerID);
      }
      return yoVariableChartData;
   }
}
