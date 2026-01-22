package us.ihmc.scs2.session.log;

import us.ihmc.robotDataLogger.Synchronization;

public class ChildLogSynchronization
{
   private int offset = -1;
   private int jogRate = -1;

   public ChildLogSynchronization()
   {
   }

   public void set(Synchronization synchronization)
   {
      setOffset(synchronization.getOffset());
      setJogRate(synchronization.getJogRate());
   }

   public void setOffset(int offest)
   {
      this.offset = offest;
   }

   public void setJogRate(int jogRate)
   {
      this.jogRate = jogRate;
   }

   public Synchronization toPacket()
   {
      Synchronization synchronization = new Synchronization();
      synchronization.setJogRate(jogRate);
      synchronization.setOffset(offset);

      return synchronization;
   }

   public long computeRelativePosition(int mainPosition)
   {
      return (long) mainPosition * jogRate + offset;
   }
}
