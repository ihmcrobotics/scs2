package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import us.ihmc.scs2.session.Session;

public interface Manager
{
   void startSession(Session session);

   void stopSession();

   boolean isSessionLoaded();
}
