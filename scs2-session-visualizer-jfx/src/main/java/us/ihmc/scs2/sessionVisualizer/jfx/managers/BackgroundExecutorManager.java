package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import us.ihmc.scs2.session.DaemonThreadFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.*;
import java.util.function.BooleanSupplier;

/**
 * This class provides support for executing and managing tasks in the background.
 * <p>
 * The tasks are executed in a {@link ScheduledExecutorService}, such that they don't interfere with the JavaFX thread. In addition, the tasks can also be
 * planned to be executed at a later time given some user conditions.
 * </p>
 */
public class BackgroundExecutorManager
{
   /**
    * The executor service used to execute the tasks in the background.
    */
   private final ScheduledExecutorService backgroundExecutor;
   /**
    * The map used to store the tasks that are queued to be executed in the background.
    * <p>
    * The key is the owner of the task, which is used to identify the task queue.
    * </p>
    */
   private final Map<Object, ConcurrentLinkedQueue<Runnable>> taskQueues = new HashMap<>();
   /**
    * The map used to store the active tasks that are currently being executed in the background.
    * <p>
    * The key is the owner of the task, which is used to identify the task queue.
    * </p>
    */
   private final Map<Object, Future<?>> activeFutureMap = new ConcurrentHashMap<>();

   /**
    * The list of futures that are currently being executed in the background.
    */
   private final List<Future<?>> futures = new ArrayList<>();

   /**
    * Whether the session is being stopped.
    */
   private volatile boolean isStoppingSession = false;

   public BackgroundExecutorManager(int numberOfThreads)
   {
      backgroundExecutor = Executors.newScheduledThreadPool(numberOfThreads, new DaemonThreadFactory(getClass().getSimpleName(), Thread.NORM_PRIORITY));
   }

   /**
    * Schedules a task to be executed in the background at a later time given a condition.
    *
    * @param condition the condition that needs to be satisfied for the task to be executed.
    * @param task      the task to be executed.
    */
   public void scheduleInBackgroundWithCondition(BooleanSupplier condition, Runnable task)
   {
      if (isStoppingSession || backgroundExecutor.isShutdown())
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

   /**
    * Queues a task to be executed in the background.
    * <p>
    * The task is queued to be executed after the previous task queued by the same owner is executed.
    * </p>
    *
    * @param owner the owner of the task.
    * @param task  the task to be executed.
    */
   public void queueTaskToExecuteInBackground(Object owner, Runnable task)
   {
      queueTaskToExecuteInBackground(owner, false, task);
   }

   /**
    * Queues a task to be executed in the background.
    * <p>
    * The task is queued to be executed after the previous task queued by the same owner is executed.
    * </p>
    *
    * @param owner               the owner of the task.
    * @param cancelPreviousTasks whether to cancel the previous tasks queued by the same owner.
    * @param task                the task to be executed.
    */
   public void queueTaskToExecuteInBackground(Object owner, boolean cancelPreviousTasks, Runnable task)
   {
      if (isStoppingSession || backgroundExecutor.isShutdown())
         return;
      ConcurrentLinkedQueue<Runnable> taskQueue = taskQueues.computeIfAbsent(owner, k -> new ConcurrentLinkedQueue<>());

      if (cancelPreviousTasks)
         taskQueue.clear();

      Runnable chainingTask = () ->
      {
         task.run();
         Runnable nextTask = taskQueue.poll();
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

   /**
    * Executes a task in the background.
    *
    * @param task the task to be executed.
    * @return the future associated with the task.
    */
   public Future<?> executeInBackground(Runnable task)
   {
      if (isStoppingSession || backgroundExecutor.isShutdown())
         return null;
      return backgroundExecutor.submit(toPrintExceptionRunnable(task));
   }

   /**
    * Executes a task in the background.
    *
    * @param task the task to be executed.
    * @param <V>  the result type for the task.
    * @return the future associated with the task.
    */
   public <V> Future<V> executeInBackground(Callable<V> task)
   {
      if (isStoppingSession || backgroundExecutor.isShutdown())
         return null;
      return backgroundExecutor.submit(toPrintExceptionCallable(task));
   }

   /**
    * Wraps a runnable such that any exception thrown by the runnable is printed to the console.
    *
    * @param runnable the runnable to wrap.
    * @return the wrapped runnable.
    */
   private Runnable toPrintExceptionRunnable(Runnable runnable)
   {
      return () ->
      {
         if (isStoppingSession)
            return;

         try
         {
            runnable.run();
         }
         catch (Exception e)
         {
            if (!isStoppingSession)
               e.printStackTrace();
         }
      };
   }

   /**
    * Wraps a callable such that any exception thrown by the callable is printed to the console.
    *
    * @param callable the callable to wrap.
    * @param <V>      the result type for the callable.
    * @return the wrapped callable.
    */
   private <V> Callable<V> toPrintExceptionCallable(Callable<V> callable)
   {
      return () ->
      {
         if (isStoppingSession)
            return null;

         try
         {
            return callable.call();
         }
         catch (Exception e)
         {
            if (!isStoppingSession)
               e.printStackTrace();
            return null;
         }
      };
   }

   /**
    * Schedules a task to be executed in the background at a fixed rate.
    *
    * @param periodicTask the task to be executed.
    * @param initialDelay the initial delay before the first execution.
    * @param period       the period between two executions.
    * @param unit         the time unit of the initial delay and period.
    * @return the future associated with the task.
    */
   public Future<?> scheduleTaskInBackground(Runnable periodicTask, long initialDelay, long period, TimeUnit unit)
   {
      ScheduledFuture<?> newFuture = backgroundExecutor.scheduleAtFixedRate(periodicTask, initialDelay, period, unit);
      futures.add(newFuture);
      return newFuture;
   }

   /**
    * Schedules a task to be executed in the background after a delay.
    *
    * @param task  the task to be executed.
    * @param delay the delay before the execution.
    * @param unit  the time unit of the delay.
    * @return the future associated with the task.
    */
   public Future<?> scheduleTaskInBackground(Runnable task, long delay, TimeUnit unit)
   {
      ScheduledFuture<?> newFuture = backgroundExecutor.schedule(toPrintExceptionRunnable(task), delay, unit);
      futures.add(newFuture);
      return newFuture;
   }

   /**
    * Stops the session.
    * <p>
    * This method cancels all the tasks that are currently being executed in the background.
    * It also prevents any new task from being executed in the background.
    * </p>
    */
   public void stopSession()
   {
      isStoppingSession = true;

      futures.forEach(future ->
                      {
                         if (!future.isDone())
                            future.cancel(false);
                      });
      futures.clear();

      isStoppingSession = false;
   }

   /**
    * Shuts down the executor service.
    */
   public void shutdown()
   {
      backgroundExecutor.shutdown();
   }
}
