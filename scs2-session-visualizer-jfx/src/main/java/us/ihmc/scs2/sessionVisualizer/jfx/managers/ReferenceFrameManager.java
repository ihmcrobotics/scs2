package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableMap;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrameChangedListener;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;
import us.ihmc.scs2.session.Session;
import us.ihmc.scs2.session.YoFixedMovingReferenceFrameUsingYawPitchRoll;
import us.ihmc.scs2.session.YoFixedReferenceFrameUsingYawPitchRoll;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.ObservedAnimationTimer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoCompositeTools;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.simulation.robot.RobotRootFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class ReferenceFrameManager implements Manager
{
   public static final String WORLD_FRAME = "worldFrame";

   private final ReferenceFrameWrapper worldFrame = new ReferenceFrameWrapper(ReferenceFrameTools.constructARootFrame(WORLD_FRAME));

   private final ObjectProperty<Map<String, ReferenceFrameWrapper>> uniqueNameToReferenceFrameMapProperty = new SimpleObjectProperty<>(this,
                                                                                                                                       "uniqueNameToReferenceFrameMap",
                                                                                                                                       null);
   private final ObjectProperty<Map<String, ReferenceFrameWrapper>> uniqueShortNameToReferenceFrameMapProperty = new SimpleObjectProperty<>(this,
                                                                                                                                            "uniqueShortNameToReferenceFrameMap",
                                                                                                                                            null);
   private final ObservableMap<String, ReferenceFrameWrapper> fullnameToReferenceFrameMap = FXCollections.observableMap(new ConcurrentHashMap<>());

   private final List<ReferenceFrameWrapper> undefinedFrames = new ArrayList<>();

   private final YoManager yoManager;
   private final BackgroundExecutorManager backgroundExecutorManager;
   private List<Runnable> cleanupTasks = null;
   private List<Runnable> updateTasks = null;
   private final ReferenceFrameChangedListener frameChangedListener;

   private final ObservedAnimationTimer taskRunner = new ObservedAnimationTimer(getClass().getSimpleName())
   {
      @Override
      public void handleImpl(long now)
      {
         if (updateTasks != null)
         {
            for (int i = 0; i < updateTasks.size(); i++)
               updateTasks.get(i).run();
         }
      }
   };

   public ReferenceFrameManager(YoManager yoManager, BackgroundExecutorManager backgroundExecutorManager)
   {
      this.yoManager = yoManager;
      this.backgroundExecutorManager = backgroundExecutorManager;

      frameChangedListener = change ->
      {
         if (!change.wasAdded())
            return;

         ReferenceFrame newFrame = change.getTarget();

         if (newFrame.getName().endsWith(Session.SCS2_INTERNAL_FRAME_SUFFIX))
            return;

         backgroundExecutorManager.queueTaskToExecuteInBackground(this, () ->
         {
            if (hasFrameBeenRemoved(newFrame))
               return;

            try
            {
               // Adding some delay so if YoVariables are needed, they are first linked.
               Thread.sleep(100);

               if (hasFrameBeenRemoved(newFrame))
                  return;

               registerNewSessionFramesNow(ReferenceFrameTools.collectFramesInSubtree(newFrame));
            }
            catch (InterruptedException e)
            {
            }
         });
      };
   }

   private static boolean hasFrameBeenRemoved(ReferenceFrame frame)
   {
      try
      {
         frame.getName();
         return false;
      }
      catch (RuntimeException e)
      {
         // The session may have ended and the frame removed, we just abort.
         return true;
      }
   }

   @Override
   public void startSession(Session session)
   {
      refreshReferenceFramesNow();
      registerNewSessionFramesNow(ReferenceFrameTools.collectFramesInSubtree(session.getInertialFrame()));
      session.getInertialFrame().addListener(frameChangedListener);
      addCleanupTask(() -> session.getInertialFrame().removeListener(frameChangedListener));
      taskRunner.start();
   }

   @Override
   public void stopSession()
   {
      taskRunner.stop();

      if (cleanupTasks != null)
      {
         cleanupTasks.forEach(Runnable::run);
         cleanupTasks.clear();
         cleanupTasks = null;
      }
      // Throw away all the reference frames but the world frame.
      worldFrame.clearChildren();
      uniqueNameToReferenceFrameMapProperty.set(null);
      uniqueShortNameToReferenceFrameMapProperty.set(null);
      fullnameToReferenceFrameMap.clear();
      undefinedFrames.clear();
   }

   @Override
   public boolean isSessionLoaded()
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return false;
      if (uniqueShortNameToReferenceFrameMapProperty.get() == null)
         return false;
      return !fullnameToReferenceFrameMap.isEmpty();
   }

   private void addCleanupTask(Runnable task)
   {
      if (cleanupTasks == null)
         cleanupTasks = new ArrayList<>();
      cleanupTasks.add(task);
   }

   private void addUpdateTask(Runnable task)
   {
      if (updateTasks == null)
         updateTasks = new ArrayList<>();
      updateTasks.add(task);
   }

   private void registerNewSessionFramesNow(Collection<ReferenceFrame> sessionFrames)
   {
      if (sessionFrames == null || sessionFrames.isEmpty())
         return;

      if (sessionFrames.stream().anyMatch(sessionFrame ->
                                          {
                                             boolean isRobotFrame = sessionFrame instanceof RobotRootFrame;
                                             // If it's a robot frame and that the root has not been registered yet, we postpone.
                                             return isRobotFrame && getReferenceFrameFromFullname(sessionFrame.getNameId()) == null;
                                          }))
      {
         // The robot is probably being created at this moment, let's postpone the registration.
         backgroundExecutorManager.scheduleTaskInBackground(() -> registerNewSessionFramesNow(sessionFrames), 100, TimeUnit.MILLISECONDS);
         return;
      }

      // Keep a reference of the new frames to ensure they're not GCed before we register them.
      LinkedList<ReferenceFrame> framesToRegister = new LinkedList<>(sessionFrames);

      while (!framesToRegister.isEmpty())
      {
         // Remove only once it's been processed.
         ReferenceFrame sessionFrame = framesToRegister.peek();
         ReferenceFrameWrapper newFrame;

         try
         {
            newFrame = duplicateReferenceFrame(sessionFrame);
         }
         catch (Exception e)
         {
            if (!isARobotFrame(sessionFrame))
            { // If it's a robot frame, we're just going to postpone.
               LogTools.error("Experienced problem setting up frame: {}.", sessionFrame.getNameId());
               e.printStackTrace();
            }
            newFrame = null;
         }

         if (newFrame != null)
         {
            fullnameToReferenceFrameMap.put(newFrame.getFullName(), newFrame);
         }
         else if (isARobotFrame(sessionFrame))
         {
            // The robot is probably being created at this moment, let's postpone the registration.
            backgroundExecutorManager.scheduleTaskInBackground(() -> registerNewSessionFramesNow(framesToRegister), 500, TimeUnit.MILLISECONDS);
         }

         framesToRegister.poll();
      }

      refreshReferenceFramesNow();
   }

   private static boolean isARobotFrame(ReferenceFrame frame)
   {
      if (frame instanceof RobotRootFrame)
         return true;
      return frame.getParent() != null && isARobotFrame(frame.getParent());
   }

   private ReferenceFrameWrapper duplicateReferenceFrame(ReferenceFrame sessionFrame)
   {
      if (sessionFrame == null || sessionFrame.isRootFrame())
         return null;
      ReferenceFrameWrapper resultFromFullname = getReferenceFrameFromFullname(sessionFrame.getNameId());
      if (resultFromFullname != null && resultFromFullname.isDefined())
         return null; // The frame has already been registered
      if (sessionFrame.getName().endsWith(Session.SCS2_INTERNAL_FRAME_SUFFIX))
         return null;

      String frameName = sessionFrame.getName();
      ReferenceFrame sessionParentFrame = sessionFrame.getParent();
      ReferenceFrameWrapper parentFrame = getReferenceFrameFromFullname(sessionParentFrame.getNameId());

      if (parentFrame == null)
      {
         if (!isARobotFrame(sessionFrame)) // If it's a robot frame, we're just going to postpone.
            LogTools.warn("Parent frame not found: {}.", sessionParentFrame.getNameId());
         return null;
      }

      ReferenceFrame frame;

      if (!sessionFrame.isFixedInParent())
      {
         LogTools.warn("Unhandled frame type: {}, {}.", sessionFrame.getNameId(), sessionFrame.getClass().getSimpleName());
         return null;
      }

      if (sessionFrame instanceof FixedReferenceFrame)
      {
         frame = new FixedReferenceFrame(frameName, parentFrame.getReferenceFrame(), sessionFrame.getTransformToParent());
      }
      else if (sessionFrame instanceof FixedMovingReferenceFrame)
      {
         frame = new FixedMovingReferenceFrame(frameName, parentFrame.getReferenceFrame(), sessionFrame.getTransformToParent());
      }
      else if (sessionFrame instanceof YoFixedReferenceFrameUsingYawPitchRoll)
      {
         YoFramePoseUsingYawPitchRoll sessionOffset = ((YoFixedReferenceFrameUsingYawPitchRoll) sessionFrame).getOffset();
         YoFramePoseUsingYawPitchRoll offset = SharedMemoryTools.duplicate(sessionOffset, yoManager.getRootRegistry(), parentFrame.getReferenceFrame());
         frame = new YoFixedReferenceFrameUsingYawPitchRoll(frameName, offset, parentFrame.getReferenceFrame());
         linkFrameYoVariables(frame, offset.getYoX(), offset.getYoY(), offset.getYoZ(), offset.getYoYaw(), offset.getYoPitch(), offset.getYoRoll());
      }
      else if (sessionFrame instanceof YoFixedMovingReferenceFrameUsingYawPitchRoll)
      {
         YoFramePoseUsingYawPitchRoll sessionOffset = ((YoFixedMovingReferenceFrameUsingYawPitchRoll) sessionFrame).getOffset();
         YoFramePoseUsingYawPitchRoll offset = SharedMemoryTools.duplicate(sessionOffset, yoManager.getRootRegistry(), parentFrame.getReferenceFrame());
         frame = new YoFixedMovingReferenceFrameUsingYawPitchRoll(frameName, offset, parentFrame.getReferenceFrame());
         linkFrameYoVariables(frame, offset.getYoX(), offset.getYoY(), offset.getYoZ(), offset.getYoYaw(), offset.getYoPitch(), offset.getYoRoll());
      }
      else
      {
         LogTools.warn("Unhandled frame type: {}, {}.", sessionFrame.getNameId(), sessionFrame.getClass().getSimpleName());
         frame = null;
      }

      return new ReferenceFrameWrapper(frame);
   }

   private void linkFrameYoVariables(ReferenceFrame frame, YoDouble... variablesToLink)
   {
      AtomicBoolean haveVariableChanged = new AtomicBoolean(true);

      for (int i = 0; i < variablesToLink.length; i++)
      {
         yoManager.getLinkedRootRegistry().linkYoVariable(variablesToLink[i], frame);
         variablesToLink[i].addListener(v -> haveVariableChanged.set(true));
      }

      addUpdateTask(() ->
                    {
                       if (haveVariableChanged.getAndSet(false))
                          frame.update();
                    });
   }

   public void refreshReferenceFramesNow()
   {
      List<ReferenceFrameWrapper> allReferenceFrames = worldFrame.collectSubtree();
      computeFullnameMap(allReferenceFrames);
      computeUniqueNameMaps(allReferenceFrames);
   }

   public void refreshReferenceFrames()
   {
      List<ReferenceFrameWrapper> allReferenceFrames = worldFrame.collectSubtree();
      backgroundExecutorManager.queueTaskToExecuteInBackground(this, () -> computeFullnameMap(allReferenceFrames));
      backgroundExecutorManager.queueTaskToExecuteInBackground(this, () -> computeUniqueNameMaps(allReferenceFrames));
   }

   private void computeUniqueNameMaps(Collection<ReferenceFrameWrapper> allReferenceFrames)
   {
      Map<ReferenceFrameWrapper, String> newMap = YoCompositeTools.computeUniqueNames(allReferenceFrames,
                                                                                      ReferenceFrameWrapper::getNamespace,
                                                                                      ReferenceFrameWrapper::getName);
      Map<String, ReferenceFrameWrapper> newUniqueNameToReferenceFrameMap = new LinkedHashMap<>();
      Map<String, ReferenceFrameWrapper> newUniqueShortNameToReferenceFrameMap = new LinkedHashMap<>();

      for (Entry<ReferenceFrameWrapper, String> entry : newMap.entrySet())
      {
         ReferenceFrameWrapper frame = entry.getKey();
         String value = entry.getValue();
         frame.setUniqueName(value);
         newUniqueNameToReferenceFrameMap.put(frame.getUniqueName(), frame);
         newUniqueShortNameToReferenceFrameMap.put(frame.getUniqueShortName(), frame);
      }

      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         uniqueNameToReferenceFrameMapProperty.set(newUniqueNameToReferenceFrameMap);
         uniqueShortNameToReferenceFrameMapProperty.set(newUniqueShortNameToReferenceFrameMap);
      });
   }

   private void computeFullnameMap(Collection<ReferenceFrameWrapper> allReferenceFrames)
   {
      for (ReferenceFrameWrapper newFrame : allReferenceFrames)
      {
         ReferenceFrameWrapper registeredFrame = fullnameToReferenceFrameMap.get(newFrame.getFullName());
         if (registeredFrame != null)
            registeredFrame.setReferenceFrame(newFrame.getReferenceFrame());
         else
            fullnameToReferenceFrameMap.put(newFrame.getFullName(), newFrame);
      }
   }

   public ReferenceFrameWrapper getWorldFrame()
   {
      return worldFrame;
   }

   public Collection<ReferenceFrameWrapper> getReferenceFrames()
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return Collections.emptyList();
      return uniqueNameToReferenceFrameMapProperty.get().values();
   }

   public Collection<String> getReferenceFrameUniqueNames()
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return Collections.emptyList();
      return uniqueNameToReferenceFrameMapProperty.get().keySet();
   }

   public Collection<String> getReferenceFrameUniqueShortNames()
   {
      if (uniqueShortNameToReferenceFrameMapProperty.get() == null)
         return Collections.emptyList();
      return uniqueShortNameToReferenceFrameMapProperty.get().keySet();
   }

   public Collection<String> getReferenceFrameFullnames()
   {
      return fullnameToReferenceFrameMap.keySet();
   }

   public ReferenceFrameWrapper getReferenceFrameFromUniqueName(String uniqueName)
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return null;
      ReferenceFrameWrapper frame = uniqueNameToReferenceFrameMapProperty.get().get(uniqueName);
      if (frame == null)
         return uniqueShortNameToReferenceFrameMapProperty.get().get(uniqueName);
      return frame;
   }

   public ReferenceFrameWrapper getReferenceFrameFromFullname(String fullname)
   {
      return getReferenceFrameFromFullname(fullname, false);
   }

   public ReferenceFrameWrapper getReferenceFrameFromFullname(String fullname, boolean createUndefinedFrameIfNeeded)
   {
      ReferenceFrameWrapper result = fullnameToReferenceFrameMap.get(fullname);
      if (result != null)
         return result;

      if (fullname.startsWith(ReferenceFrame.getWorldFrame().getName()))
      {
         fullname = fullname.replaceFirst(ReferenceFrame.getWorldFrame().getName(), WORLD_FRAME);
         result = fullnameToReferenceFrameMap.get(fullname);
      }

      if (result != null)
         return result;

      if (createUndefinedFrameIfNeeded)
      {
         String name = fullname.substring(fullname.lastIndexOf(ReferenceFrame.SEPARATOR) + 1);
         result = new ReferenceFrameWrapper(name, fullname);
         fullnameToReferenceFrameMap.put(fullname, result);
         undefinedFrames.add(result);
         backgroundExecutorManager.queueTaskToExecuteInBackground(this, () -> computeUniqueNameMaps(fullnameToReferenceFrameMap.values()));
         return result;
      }

      return null;
   }
}
