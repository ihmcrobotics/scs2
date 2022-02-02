package us.ihmc.scs2.session.log;

public interface ProgressConsumer
{
   void started(String task);

   void info(String info);

   void error(String error);

   void progress(double progressPercentage);

   void done();

   default ProgressConsumer subProgress(double from, double to)
   {
      return subProgress(this, from, to);
   }

   static ProgressConsumer subProgress(ProgressConsumer original, double from, double to)
   {
      return new ProgressConsumer()
      {
         @Override
         public void started(String task)
         {
         }

         @Override
         public void info(String info)
         {
            original.info(info);
         }

         @Override
         public void error(String error)
         {
            original.error(error);
         }

         @Override
         public void progress(double progressPercentage)
         {
            original.progress(progressPercentage * (to - from) + from);
         }

         @Override
         public void done()
         {
         }
      };
   }
}