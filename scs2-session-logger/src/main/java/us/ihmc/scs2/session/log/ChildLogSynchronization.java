package us.ihmc.scs2.session.log;

import us.ihmc.robotDataLogger.Synchronization;

public class ChildLogSynchronization
{
   private double offset = -1;
   private double jogRate = -1;

   public ChildLogSynchronization()
   {
   }

   public void set(Synchronization synchronization)
   {
      setOffset(synchronization.getOffset());
      setJogRate(synchronization.getJogRate());
   }

   public void setOffset(double offest)
   {
      this.offset = offest;
   }

   public void setJogRate(double jogRate)
   {
      this.jogRate = jogRate;
   }

   public double getOffset()
   {
      return offset;
   }

   public double getJogRate()
   {
      return jogRate;
   }

   public Synchronization toPacket()
   {
      Synchronization synchronization = new Synchronization();
      synchronization.setJogRate(jogRate);
      synchronization.setOffset(offset);

      return synchronization;
   }

   public long computeChildPosition(int parentPosition)
   {
      return Math.round(parentPosition * jogRate + offset);
   }

   public long computeParentPosition(int childPosition)
   {
      return Math.round((childPosition - offset) / jogRate);
   }
}
