package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import us.ihmc.scs2.session.DaemonThreadFactory;

public class BackgroundExecutorManager
{
   private final ScheduledExecutorService backgroundExecutor;
   private final Map<Object, ConcurrentLinkedQueue<Runnable>> taskQueues = new HashMap<>();
   private final Map<Object, Future<?>> activeFutureMap = new ConcurrentHashMap<>();

   private final List<Future<?>> futures = new ArrayList<>();

   public BackgroundExecutorManager(int numberOfThreads)
   {
      backgroundExecutor = Executors.newScheduledThreadPool(numberOfThreads, new DaemonThreadFactory(getClass().getSimpleName(), Thread.NORM_PRIORITY));
   }

   public void scheduleInBackgroundWithCondition(BooleanSupplier condition, Runnable task)
   {
      if (backgroundExecutor.isShutdown())
         return;

      Runnable taskLocal = toPrintExceptionRunnable(task);
      Runnable delayedTask = new Runnable()
      {
         @Override
         public void run()
         {
            if (!condition.getAsBoolean())
            {
               backgroundExecutor.execute(this);
               return;
            }

            taskLocal.run();
         }
      };
      backgroundExecutor.execute(delayedTask);
   }

   public void queueTaskToExecuteInBackground(Object owner, Runnable task)
   {
      if (backgroundExecutor.isShutdown())
         return;
      ConcurrentLinkedQueue<Runnable> taskQueue = taskQueues.get(owner);
      if (taskQueue == null)
      {
         taskQueue = new ConcurrentLinkedQueue<>();
         taskQueues.put(owner, taskQueue);
      }

      ConcurrentLinkedQueue<Runnable> taskQueueFinal = taskQueue;

      Runnable chainingTask = () ->
      {
         task.run();
         Runnable nextTask = taskQueueFinal.poll();
         if (nextTask != null)
            executeInBackground(nextTask);
      };

      Future<?> activeFuture = activeFutureMap.get(owner);
      if (activeFuture == null || activeFuture.isDone())
      {
         activeFutureMap.put(owner, executeInBackground(chainingTask));
      }
      else
      {
         taskQueue.add(chainingTask);
      }
   }

   public Future<?> executeInBackground(Runnable task)
   {
      if (backgroundExecutor.isShutdown())
         return null;
      return backgroundExecutor.submit(toPrintExceptionRunnable(task));
   }

   public <V> Future<V> executeInBackground(Callable<V> task)
   {
      if (backgroundExecutor.isShutdown())
         return null;
      return backgroundExecutor.submit(toPrintExceptionCallable(task));
   }

   private static Runnable toPrintExceptionRunnable(Runnable runnable)
   {
      return () ->
      {
         try
         {
            runnable.run();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      };
   }

   private static <V> Callable<V> toPrintExceptionCallable(Callable<V> callable)
   {
      return () ->
      {
         try
         {
            return callable.call();
         }
         catch (Exception e)
         {
            e.printStackTrace();
            return null;
         }
      };
   }

   public Future<?> scheduleTaskInBackground(Runnable periodicTask, long initialDelay, long period, TimeUnit unit)
   {
      ScheduledFuture<?> newFuture = backgroundExecutor.scheduleAtFixedRate(periodicTask, initialDelay, period, unit);
      futures.add(newFuture);
      return newFuture;
   }

   public void stopSession()
   {
      futures.forEach(future ->
      {
         if (!future.isDone())
            future.cancel(false);
      });
      futures.clear();
   }

   public void shutdown()
   {
      backgroundExecutor.shutdown();
   }
}
