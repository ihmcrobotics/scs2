package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

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
import us.ihmc.scs2.sharedMemory.LinkedYoDouble;
import us.ihmc.scs2.sharedMemory.tools.SharedMemoryTools;
import us.ihmc.scs2.simulation.robot.RobotRootFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoDouble;

public class ReferenceFrameManager implements Manager
{
   public static final String WORLD_FRAME = "worldFrame";

   private final ReferenceFrame worldFrame = ReferenceFrameTools.constructARootFrame(WORLD_FRAME);

   private final ObjectProperty<Map<String, ReferenceFrame>> uniqueNameToReferenceFrameMapProperty = new SimpleObjectProperty<>(this,
                                                                                                                                "uniqueNameToReferenceFrameMap",
                                                                                                                                null);
   private final ObjectProperty<Map<ReferenceFrame, String>> referenceFrameToUniqueNameMapProperty = new SimpleObjectProperty<>(this,
                                                                                                                                "referenceFrameToUniqueNameMap",
                                                                                                                                null);
   private final ObjectProperty<Map<String, ReferenceFrame>> uniqueShortNameToReferenceFrameMapProperty = new SimpleObjectProperty<>(this,
                                                                                                                                     "uniqueShortNameToReferenceFrameMap",
                                                                                                                                     null);
   private final ObjectProperty<Map<ReferenceFrame, String>> referenceFrameToUniqueShortNameMapProperty = new SimpleObjectProperty<>(this,
                                                                                                                                     "referenceFrameToUniqueShortNameMap",
                                                                                                                                     null);

   private final ObservableMap<String, ReferenceFrame> fullnameToReferenceFrameMap = FXCollections.observableMap(new ConcurrentHashMap<>());

   private YoManager yoManager;
   private final BackgroundExecutorManager backgroundExecutorManager;
   private List<Runnable> cleanupTasks = null;
   private List<Runnable> updateTasks = null;
   private ReferenceFrameChangedListener frameChangedListener;

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
      computeFullnameMap(ReferenceFrameTools.getAllFramesInTree(worldFrame));
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
      referenceFrameToUniqueNameMapProperty.set(null);
      uniqueShortNameToReferenceFrameMapProperty.set(null);
      referenceFrameToUniqueShortNameMapProperty.set(null);
      fullnameToReferenceFrameMap.clear();
   }

   @Override
   public boolean isSessionLoaded()
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return false;
      if (referenceFrameToUniqueNameMapProperty.get() == null)
         return false;
      if (uniqueShortNameToReferenceFrameMapProperty.get() == null)
         return false;
      if (referenceFrameToUniqueShortNameMapProperty.get() == null)
         return false;
      if (fullnameToReferenceFrameMap.isEmpty())
         return false;
      return true;
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
      List<ReferenceFrame> frames = new ArrayList<>();
      LinkedList<ReferenceFrame> framesToRegister = new LinkedList<>(sessionFrames);

      while (!framesToRegister.isEmpty())
      {
         // Remove only once it's been processed.
         ReferenceFrame sessionFrame = framesToRegister.peek();
         ReferenceFrame frame;

         try
         {
            frame = duplicateReferenceFrame(sessionFrame);
         }
         catch (Exception e)
         {
            if (!isARobotFrame(sessionFrame))
            { // If it's a robot frame, we're just going to postpone.
               LogTools.error("Experienced problem setting up frame: {}.", sessionFrame.getNameId());
               e.printStackTrace();
            }
            frame = null;
         }

         if (frame != null)
         {
            fullnameToReferenceFrameMap.put(frame.getNameId(), frame);
            frames.add(frame);
         }
         else if (isARobotFrame(sessionFrame))
         {
            // The robot is probably being created at this moment, let's postpone the registration.
            backgroundExecutorManager.scheduleTaskInBackground(() -> registerNewSessionFramesNow(framesToRegister), 500, TimeUnit.MILLISECONDS);
         }

         framesToRegister.poll();
      }

      computeUniqueNameMaps(ReferenceFrameTools.getAllFramesInTree(worldFrame));
   }

   private static boolean isARobotFrame(ReferenceFrame frame)
   {
      if (frame instanceof RobotRootFrame)
         return true;
      return frame.getParent() == null ? false : isARobotFrame(frame.getParent());
   }

   private ReferenceFrame duplicateReferenceFrame(ReferenceFrame sessionFrame)
   {
      if (sessionFrame == null || sessionFrame.isRootFrame())
         return null;
      if (getReferenceFrameFromFullname(sessionFrame.getNameId()) != null)
         return null; // The frame has already been registered
      if (sessionFrame.getName().endsWith(Session.SCS2_INTERNAL_FRAME_SUFFIX))
         return null;

      String frameName = sessionFrame.getName();
      ReferenceFrame sessionParentFrame = sessionFrame.getParent();
      ReferenceFrame parentFrame = getReferenceFrameFromFullname(sessionParentFrame.getNameId());

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
         frame = new FixedReferenceFrame(frameName, parentFrame, sessionFrame.getTransformToParent());
      }
      else if (sessionFrame instanceof FixedMovingReferenceFrame)
      {
         frame = new FixedMovingReferenceFrame(frameName, parentFrame, sessionFrame.getTransformToParent());
      }
      else if (sessionFrame instanceof YoFixedReferenceFrameUsingYawPitchRoll)
      {
         YoFramePoseUsingYawPitchRoll sessionOffset = ((YoFixedReferenceFrameUsingYawPitchRoll) sessionFrame).getOffset();
         YoFramePoseUsingYawPitchRoll offset = SharedMemoryTools.duplicate(sessionOffset, yoManager.getRootRegistry(), parentFrame);
         frame = new YoFixedReferenceFrameUsingYawPitchRoll(frameName, offset, parentFrame);
         YoDouble[] variablesToLink = {offset.getYoX(), offset.getYoY(), offset.getYoZ(), offset.getYoYaw(), offset.getYoPitch(), offset.getYoRoll()};
         LinkedYoDouble[] linkedVariables = new LinkedYoDouble[variablesToLink.length];
         AtomicBoolean haveVariableChanged = new AtomicBoolean(true);

         for (int i = 0; i < variablesToLink.length; i++)
         {
            linkedVariables[i] = yoManager.getLinkedRootRegistry().linkYoVariable(variablesToLink[i], frame);
            variablesToLink[i].addListener(v -> haveVariableChanged.set(true));
         }

         addUpdateTask(() ->
                       {
                          if (haveVariableChanged.getAndSet(false))
                             frame.update();
                       });
      }
      else if (sessionFrame instanceof YoFixedMovingReferenceFrameUsingYawPitchRoll)
      {
         YoFramePoseUsingYawPitchRoll sessionOffset = ((YoFixedMovingReferenceFrameUsingYawPitchRoll) sessionFrame).getOffset();
         YoFramePoseUsingYawPitchRoll offset = SharedMemoryTools.duplicate(sessionOffset, yoManager.getRootRegistry(), parentFrame);
         frame = new YoFixedMovingReferenceFrameUsingYawPitchRoll(frameName, offset, parentFrame);
         YoDouble[] variablesToLink = {offset.getYoX(), offset.getYoY(), offset.getYoZ(), offset.getYoYaw(), offset.getYoPitch(), offset.getYoRoll()};
         LinkedYoDouble[] linkedVariables = new LinkedYoDouble[variablesToLink.length];
         AtomicBoolean haveVariableChanged = new AtomicBoolean(true);

         for (int i = 0; i < variablesToLink.length; i++)
         {
            linkedVariables[i] = yoManager.getLinkedRootRegistry().linkYoVariable(variablesToLink[i], frame);
            variablesToLink[i].addListener(v -> haveVariableChanged.set(true));
         }

         addUpdateTask(() ->
                       {
                          if (haveVariableChanged.getAndSet(false))
                             frame.update();
                       });
      }
      else
      {
         LogTools.warn("Unhandled frame type: {}, {}.", sessionFrame.getNameId(), sessionFrame.getClass().getSimpleName());
         frame = null;
      }
      return frame;
   }

   public void refreshReferenceFramesNow()
   {
      Collection<ReferenceFrame> allReferenceFrames = ReferenceFrameTools.getAllFramesInTree(worldFrame);
      computeFullnameMap(allReferenceFrames);
      computeUniqueNameMaps(allReferenceFrames);
   }

   public void refreshReferenceFrames()
   {
      Collection<ReferenceFrame> allReferenceFrames = ReferenceFrameTools.getAllFramesInTree(worldFrame);
      backgroundExecutorManager.queueTaskToExecuteInBackground(this, () -> computeFullnameMap(allReferenceFrames));
      backgroundExecutorManager.queueTaskToExecuteInBackground(this, () -> computeUniqueNameMaps(allReferenceFrames));
   }

   private void computeUniqueNameMaps(Collection<ReferenceFrame> allReferenceFrames)
   {
      Map<ReferenceFrame, String> newMap = YoCompositeTools.computeUniqueNames(allReferenceFrames,
                                                                               ReferenceFrameManager::getFrameNamespace,
                                                                               ReferenceFrame::getName);
      Map<String, ReferenceFrame> newUniqueNameToReferenceFrameMap = new LinkedHashMap<>();
      Map<ReferenceFrame, String> newReferenceFrameToUniqueNameMap = new LinkedHashMap<>();
      Map<String, ReferenceFrame> newUniqueShortNameToReferenceFrameMap = new LinkedHashMap<>();
      Map<ReferenceFrame, String> newReferenceFrameToUniqueShortNameMap = new LinkedHashMap<>();

      newMap.entrySet().forEach(e ->
                                {
                                   ReferenceFrame frame = e.getKey();
                                   String uniqueName = e.getValue();
                                   newUniqueNameToReferenceFrameMap.put(uniqueName, frame);
                                   newReferenceFrameToUniqueNameMap.put(frame, uniqueName);

                                   int firstSeparatorIndex = uniqueName.indexOf(".");
                                   int lastSeparatorIndex = uniqueName.lastIndexOf(".");
                                   String uniqueShortName = uniqueName;
                                   if (firstSeparatorIndex != lastSeparatorIndex)
                                      uniqueShortName = uniqueName.substring(0, firstSeparatorIndex) + "..." + uniqueName.substring(lastSeparatorIndex + 1,
                                                                                                                                    uniqueName.length());

                                   newUniqueShortNameToReferenceFrameMap.put(uniqueShortName, frame);
                                   newReferenceFrameToUniqueShortNameMap.put(frame, uniqueShortName);
                                });

      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         uniqueNameToReferenceFrameMapProperty.set(newUniqueNameToReferenceFrameMap);
         referenceFrameToUniqueNameMapProperty.set(newReferenceFrameToUniqueNameMap);
         uniqueShortNameToReferenceFrameMapProperty.set(newUniqueShortNameToReferenceFrameMap);
         referenceFrameToUniqueShortNameMapProperty.set(newReferenceFrameToUniqueShortNameMap);
      });
   }

   private void computeFullnameMap(Collection<ReferenceFrame> allReferenceFrames)
   {
      allReferenceFrames.forEach(frame -> fullnameToReferenceFrameMap.computeIfAbsent(frame.getNameId(), s -> frame));
   }

   private static List<String> getFrameNamespace(ReferenceFrame referenceFrame)
   {
      List<String> namespace = new ArrayList<>();
      for (ReferenceFrame ancestor = referenceFrame.getParent(); ancestor != null; ancestor = ancestor.getParent())
         namespace.add(0, ancestor.getName());
      return namespace;
   }

   public ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public Collection<ReferenceFrame> getReferenceFrames()
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

   public ReferenceFrame getReferenceFrameFromUniqueName(String uniqueName)
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return null;
      ReferenceFrame frame = uniqueNameToReferenceFrameMapProperty.get().get(uniqueName);
      if (frame == null)
         return uniqueShortNameToReferenceFrameMapProperty.get().get(uniqueName);
      return frame;
   }

   public ReferenceFrame getReferenceFrameFromFullname(String fullname)
   {
      ReferenceFrame result = fullnameToReferenceFrameMap.get(fullname);
      if (result != null)
         return result;

      if (fullname.startsWith(ReferenceFrame.getWorldFrame().getName()))
      {
         fullname = fullname.replaceFirst(ReferenceFrame.getWorldFrame().getName(), WORLD_FRAME);
         return fullnameToReferenceFrameMap.get(fullname);
      }
      return null;
   }

   public String getUniqueName(ReferenceFrame referenceFrame)
   {
      if (referenceFrameToUniqueNameMapProperty.get() == null)
         return null;
      return referenceFrameToUniqueNameMapProperty.get().get(referenceFrame);
   }

   public String getUniqueShortName(ReferenceFrame referenceFrame)
   {
      if (referenceFrameToUniqueShortNameMapProperty.get() == null)
         return null;
      return referenceFrameToUniqueShortNameMapProperty.get().get(referenceFrame);
   }

   public Map<ReferenceFrame, String> getReferenceFrameToUniqueNameMap()
   {
      return referenceFrameToUniqueNameMapProperty.get();
   }

   public Map<String, ReferenceFrame> getUniqueNameToReferenceFrameMap()
   {
      return uniqueNameToReferenceFrameMapProperty.get();
   }

   public Map<String, ReferenceFrame> getUniqueShortNameToReferenceFrameMap()
   {
      return uniqueShortNameToReferenceFrameMapProperty.get();
   }

   public Map<String, ReferenceFrame> getFullnameToReferenceFrameMap()
   {
      return fullnameToReferenceFrameMap;
   }
}
