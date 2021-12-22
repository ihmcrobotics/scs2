package us.ihmc.scs2.sessionVisualizer.jfx.tools;

public class ScoredObject<T> implements Comparable<ScoredObject<T>>
{
   private final T object;
   private final Number score;

   public ScoredObject(T object, Number score)
   {
      this.object = object;
      this.score = score;
   }

   public T getObject()
   {
      return object;
   }

   public Number getScore()
   {
      return score;
   }

   @Override
   public int compareTo(ScoredObject<T> o)
   {
      return -Double.compare(score.doubleValue(), o.score.doubleValue());
   }
}