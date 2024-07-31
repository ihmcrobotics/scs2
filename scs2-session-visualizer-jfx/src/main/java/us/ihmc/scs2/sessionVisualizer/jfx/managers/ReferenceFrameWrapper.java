package us.ihmc.scs2.sessionVisualizer.jfx.managers;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.WeakList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;

/**
 * Wrapper around a {@link ReferenceFrame} to provide the ability to have undefined reference frames and other minor utilities.
 */
public class ReferenceFrameWrapper
{
   private static final RigidBodyTransformReadOnly IDENTITY = new RigidBodyTransform();
   private final String name;
   private String fullName;
   private List<String> namespace;
   private String uniqueName;
   private String uniqueShortName;
   private ReferenceFrame referenceFrame;
   private final WeakList<Runnable> changeListeners = new WeakList<>();

   public ReferenceFrameWrapper(String name, String fullName)
   {
      this(name, fullName, null);
   }

   public ReferenceFrameWrapper(ReferenceFrame referenceFrame)
   {
      this(referenceFrame.getName(), referenceFrame.getNameId(), referenceFrame);
   }

   private ReferenceFrameWrapper(String name, String fullName, ReferenceFrame referenceFrame)
   {
      this.name = name;
      this.fullName = fullName;
      this.referenceFrame = referenceFrame;
   }

   public String getName()
   {
      return name;
   }

   public String getFullName()
   {
      return fullName;
   }

   public String getUniqueName()
   {
      return uniqueName;
   }

   public String getUniqueShortName()
   {
      if (uniqueShortName == null)
      {
         int firstSeparatorIndex = uniqueName.indexOf(".");
         int lastSeparatorIndex = uniqueName.lastIndexOf(".");
         if (firstSeparatorIndex != lastSeparatorIndex)
            uniqueShortName = uniqueName.substring(0, firstSeparatorIndex) + "..." + uniqueName.substring(lastSeparatorIndex + 1);
         else
            uniqueShortName = uniqueName;
      }

      return uniqueShortName;
   }

   public List<String> getNamespace()
   {
      if (namespace == null)
      {
         namespace = Arrays.asList(fullName.split(ReferenceFrame.SEPARATOR));
         namespace = namespace.subList(0, namespace.size() - 1);
      }
      return namespace;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void setUniqueName(String uniqueName)
   {
      this.uniqueName = uniqueName;
      uniqueShortName = null;
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      if (referenceFrame == this.referenceFrame)
         return;

      if (!Objects.equals(name, referenceFrame.getName()))
      {
         throw new IllegalArgumentException(
               "The name of the reference frame does not match the name of this wrapper: " + name + " vs " + referenceFrame.getName());
      }

      this.referenceFrame = referenceFrame;
      fullName = referenceFrame.getNameId();
      namespace = null;
      changeListeners.forEach(Runnable::run);
   }

   public boolean isDefined()
   {
      return referenceFrame != null;
   }

   public boolean isRootFrame()
   {
      // Making undefined reference frames root frames.
      return !isDefined() || referenceFrame.isRootFrame();
   }

   public RigidBodyTransformReadOnly getTransformToRoot()
   {
      if (referenceFrame == null)
         return IDENTITY;
      else
         return referenceFrame.getTransformToRoot();
   }

   public <T extends Transformable> T transformFromThisToDesiredFrame(ReferenceFrameWrapper desiredFrame, T transformable)
   {
      if (referenceFrame != null && desiredFrame.referenceFrame != null)
         referenceFrame.transformFromThisToDesiredFrame(desiredFrame.referenceFrame, transformable);
      return transformable;
   }

   public <T extends Transformable> T transformToRootFrame(T transformable)
   {
      if (referenceFrame != null && !referenceFrame.isRootFrame())
         referenceFrame.transformFromThisToDesiredFrame(referenceFrame.getRootFrame(), transformable);
      return transformable;
   }

   public List<ReferenceFrameWrapper> collectSubtree()
   {
      if (referenceFrame == null)
         return List.of(this);
      else
         return collectSubtree(this, new ArrayList<>());
   }

   private static <T extends Collection<ReferenceFrameWrapper>> T collectSubtree(ReferenceFrameWrapper start, T collectionToPack)
   {
      collectionToPack.add(start);

      if (start.referenceFrame != null)
      {
         for (int i = 0; i < start.referenceFrame.getNumberOfChildren(); i++)
         {
            ReferenceFrame child = start.referenceFrame.getChild(i);
            if (child != null)
               collectSubtree(new ReferenceFrameWrapper(child), collectionToPack);
         }
      }

      return collectionToPack;
   }

   public void clearChildren()
   {
      if (referenceFrame != null)
         referenceFrame.clearChildren();
   }

   public void addChangeListener(Runnable listener)
   {
      changeListeners.add(listener);
   }

   public void removeChangeListener(Runnable listener)
   {
      changeListeners.remove(listener);
   }

   @Override
   public String toString()
   {
      return "%s{name='%s', fullName='%s'}".formatted(getClass().getSimpleName(), name, fullName);
   }
}
