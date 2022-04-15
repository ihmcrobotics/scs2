package us.ihmc.scs2.session;

import us.ihmc.scs2.sharedMemory.YoSharedBuffer;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This enum represents the different modes a session can be in.
 */
public enum SessionMode
{
   /**
    * The running mode can represent (non-exhaustive list):
    * <ul>
    * <li>simulating with a simulation session,
    * <li>streaming data from the server with a remote session,
    * <li>reading a log file with a log session.
    * </ul>
    * <p>
    * In general, when the session is running, it is actively computing or receiving new
    * {@link YoVariable} data and then stores it to its {@link YoSharedBuffer}.
    * </p>
    */
   RUNNING,
   /**
    * When in playback mode, the session reads the {@link YoVariable} data from its
    * {@link YoSharedBuffer} at a regular rate. It literally plays back the data stored in the buffer
    * in between its in and out points.
    */
   PLAYBACK,
   /**
    * When in pause mode, the session "asleep" allowing the user to operate on the buffer, for instance
    * the user can scrub in the data history by setting the buffer current index.
    */
   PAUSE;
};
