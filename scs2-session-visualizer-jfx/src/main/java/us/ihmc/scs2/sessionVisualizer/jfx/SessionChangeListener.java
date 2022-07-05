package us.ihmc.scs2.sessionVisualizer.jfx;

import us.ihmc.scs2.session.Session;

/**
 * Interface for implementing a listener that is to be notified when the active session has changed.
 * 
 * @author Sylvain Bertrand
 */
public interface SessionChangeListener
{
   /**
    * Notifies the session just changed.
    * 
    * @param previousSession the previous session, can be {@code null}.
    * @param newSession      the new active session, can be {@code null} when stopping a session.
    */
   void sessionChanged(Session previousSession, Session newSession);
}