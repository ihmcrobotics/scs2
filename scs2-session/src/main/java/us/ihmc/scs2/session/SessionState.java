package us.ihmc.scs2.session;

/**
 * This enum describes the state of a session.
 */
public enum SessionState
{
   /**
    * A session becomes active right after {@link Session#startSessionThread()} and will remain active
    * until it is shutdown.
    */
   ACTIVE,
   /**
    * An inactive session is a session that was previously active and was then shutdown. The inactive
    * state represents the end-of-life state of a session.
    */
   INACTIVE;
}
