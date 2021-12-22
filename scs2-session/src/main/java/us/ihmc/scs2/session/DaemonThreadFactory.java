package us.ihmc.scs2.session;

import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;

public class DaemonThreadFactory implements ThreadFactory
{
   private final ThreadGroup group;
   private final String namePrefix;
   private final int threadPriority;
   private final AtomicInteger threadNumber = new AtomicInteger(1);

   public DaemonThreadFactory(String poolName)
   {
      this(poolName, Thread.NORM_PRIORITY);
   }

   public DaemonThreadFactory(String poolName, int threadPriority)
   {
      this(poolName, getCurrentGroup(), threadPriority);
   }

   public DaemonThreadFactory(String poolName, ThreadGroup threadGroup, int threadPriority)
   {
      group = threadGroup;
      namePrefix = poolName + " Pool [Thread-";
      this.threadPriority = threadPriority;
   }

   public ThreadGroup getThreadGroup()
   {
      return group;
   }

   public Thread newThread(Runnable r)
   {
      Thread thread = new Thread(group, r, namePrefix + threadNumber.getAndIncrement() + "]");
      thread.setDaemon(true);
      if (thread.getPriority() != threadPriority)
         thread.setPriority(threadPriority);
      return thread;
   }

   private static ThreadGroup getCurrentGroup()
   {
      return (System.getSecurityManager() != null) ? System.getSecurityManager().getThreadGroup() : Thread.currentThread().getThreadGroup();
   }
}
