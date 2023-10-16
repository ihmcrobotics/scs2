package us.ihmc.scs2.session.mcap;

public interface MCAPDebugPrinter
{
   void print(String string);

   default void println(String string)
   {
      print(string);
      print("\n");
   }
}
