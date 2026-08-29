package us.ihmc.scs2.sessionVisualizer.jfx.messager;

/**
 * A typed, named channel key used with {@link SCS2Messager}.
 * <p>
 * Topics are meant to be declared once as {@code public static final} constants (see
 * {@link us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerMessagerAPI}) and shared by identity;
 * two {@code Topic} instances are never considered equal even if they share the same name.
 * </p>
 *
 * @param <T> the type of data carried by messages sent on this topic.
 */
public final class Topic<T>
{
   private final String name;

   public Topic(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   @Override
   public String toString()
   {
      return name;
   }
}
