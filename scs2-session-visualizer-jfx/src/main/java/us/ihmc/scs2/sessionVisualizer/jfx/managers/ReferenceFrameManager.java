package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

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

   private final ObservableMap<String, ReferenceFrame> fullnameToReferenceFrameMap = FXCollections.observableMap(new ConcurrentHashMap<>());

   private YoManager yoManager;
   private final BackgroundExecutorManager backgroundExecutorManager;
   private List<Runnable> cleanupTasks = null;
   private List<Runnable> updateTasks = null;
   private ReferenceFrameChangedListener frameChangedListener = change ->
   {
      if (change.wasAdded())
         registerNewSessionFrames(ReferenceFrameTools.collectFramesInSubtree(change.getTarget()));
   };

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
      fullnameToReferenceFrameMap.clear();
   }

   @Override
   public boolean isSessionLoaded()
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return false;
      if (referenceFrameToUniqueNameMapProperty.get() == null)
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

   private void registerNewSessionFrames(Collection<ReferenceFrame> sessionFrames)
   {
      backgroundExecutorManager.queueTaskToExecuteInBackground(this, () -> registerNewSessionFramesNow(sessionFrames));
   }

   private void registerNewSessionFramesNow(Collection<ReferenceFrame> sessionFrames)
   {
      if (sessionFrames == null || sessionFrames.isEmpty())
         return;

      // Keep a reference of the new frames to ensure they're not GCed before we register them.
      List<ReferenceFrame> frames = new ArrayList<>();

      for (ReferenceFrame sessionFrame : sessionFrames)
      {
         ReferenceFrame frame = duplicateReferenceFrame(sessionFrame);
         if (frame != null)
         {
            fullnameToReferenceFrameMap.put(frame.getNameId(), frame);
            frames.add(frame);
         }
      }

      computeUniqueNameMaps(ReferenceFrameTools.getAllFramesInTree(worldFrame));
   }

   private ReferenceFrame duplicateReferenceFrame(ReferenceFrame sessionFrame)
   {
      if (sessionFrame == null || sessionFrame.isRootFrame())
         return null;
      if (getReferenceFrameFromFullname(sessionFrame.getNameId()) != null)
         return null; // The frame has already been registered

      String frameName = sessionFrame.getName();
      ReferenceFrame sessionParentFrame = sessionFrame.getParent();
      ReferenceFrame parentFrame = getReferenceFrameFromFullname(sessionParentFrame.getNameId());

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

         for (int i = 0; i < variablesToLink.length; i++)
         {
            linkedVariables[i] = yoManager.getLinkedRootRegistry().linkYoVariable(variablesToLink[i]);
            linkedVariables[i].addUser(frame);
         }

         addUpdateTask(() ->
         {
            boolean updateFrame = false;

            for (LinkedYoDouble linkedVariable : linkedVariables)
            {
               updateFrame |= linkedVariable.pull();
            }

            if (updateFrame)
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

         for (int i = 0; i < variablesToLink.length; i++)
         {
            linkedVariables[i] = yoManager.getLinkedRootRegistry().linkYoVariable(variablesToLink[i]);
            linkedVariables[i].addUser(frame);
         }

         addUpdateTask(() ->
         {
            boolean updateFrame = false;

            for (LinkedYoDouble linkedVariable : linkedVariables)
            {
               updateFrame |= linkedVariable.pull();
            }

            if (updateFrame)
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

      newMap.entrySet().forEach(e ->
      {
         newUniqueNameToReferenceFrameMap.put(e.getValue(), e.getKey());
         newReferenceFrameToUniqueNameMap.put(e.getKey(), e.getValue());
      });

      JavaFXMissingTools.runLaterIfNeeded(getClass(), () ->
      {
         uniqueNameToReferenceFrameMapProperty.set(newUniqueNameToReferenceFrameMap);
         referenceFrameToUniqueNameMapProperty.set(newReferenceFrameToUniqueNameMap);
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

   public Collection<String> getReferenceFrameFullnames()
   {
      return fullnameToReferenceFrameMap.keySet();
   }

   public ReferenceFrame getReferenceFrameFromUniqueName(String uniqueName)
   {
      if (uniqueNameToReferenceFrameMapProperty.get() == null)
         return null;
      return uniqueNameToReferenceFrameMapProperty.get().get(uniqueName);
   }

   public ReferenceFrame getReferenceFrameFromFullname(String fullname)
   {
      return fullnameToReferenceFrameMap.get(fullname);
   }

   public String getUniqueName(ReferenceFrame referenceFrame)
   {
      if (referenceFrameToUniqueNameMapProperty.get() == null)
         return null;
      return referenceFrameToUniqueNameMapProperty.get().get(referenceFrame);
   }

   public Map<ReferenceFrame, String> getReferenceFrameToUniqueNameMap()
   {
      return referenceFrameToUniqueNameMapProperty.get();
   }

   public Map<String, ReferenceFrame> getUniqueNameToReferenceFrameMap()
   {
      return uniqueNameToReferenceFrameMapProperty.get();
   }

   public Map<String, ReferenceFrame> getFullnameToReferenceFrameMap()
   {
      return fullnameToReferenceFrameMap;
   }
}
