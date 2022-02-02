package us.ihmc.scs2.session.remote;

import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;

public interface SimpleYoVariablesUpdatedListener extends YoVariablesUpdatedListener
{
   @Override
   default void setShowOverheadView(boolean showOverheadView)
   {
   }

   @Override
   default void connected()
   {
   }

   @Override
   default void disconnected()
   {
   }

   @Override
   default boolean changesVariables()
   {
      return true;
   }

   @Override
   default boolean updateYoVariables()
   {
      return true;
   }
}
