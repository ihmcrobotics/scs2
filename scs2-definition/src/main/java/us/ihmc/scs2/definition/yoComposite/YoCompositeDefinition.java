package us.ihmc.scs2.definition.yoComposite;

import java.util.Arrays;

import javax.xml.bind.annotation.XmlSeeAlso;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * A {@code YoCompositeDefinition} is used to represent the template for a composite type. A
 * composite is an abstract representation of a type with components which can be expressed in a
 * reference frame.
 * <p>
 * The user typically does not need to deal with this class directly and should instead try to use
 * the implementations of it directly. It is meant for facilitating type management in SCS2.
 * </p>
 * <p>
 * Each component can be backed by a {@code YoVariable}, in which case the component is set to the
 * variable name/fullname, or is a constant value, which case it is set to the string representation
 * of the value.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlSeeAlso({YoOrientation3DDefinition.class, YoTuple2DDefinition.class, YoTuple3DDefinition.class})
public abstract class YoCompositeDefinition
{
   /**
    * Returns the type of this composite, e.g. "YoTuple2D".
    *
    * @return this composite type.
    */
   public abstract String getType();

   /**
    * Returns the identifiers (or name) in order associated to each component.
    *
    * @return the identifier in order for each component.
    */
   public abstract String[] getComponentIdentifiers();

   /**
    * Returns the value in order for each component.
    *
    * @return the value in order for each component.
    */
   public abstract String[] getComponentValues();

   /**
    * Sets the name id ({@link ReferenceFrame#getNameId()}) of the reference frame in which this
    * composite is to be expressed, or {@code null} if it is expressed in world frame.
    *
    * @param referenceFrame the name id ({@link ReferenceFrame#getNameId()} of the reference frame.
    */
   public abstract void setReferenceFrame(String referenceFrame);

   /**
    * Returns the name id ({@link ReferenceFrame#getNameId()}) of the reference frame this composite is
    * expressed in, or {@code null} if it is expressed in world frame.
    *
    * @return the fullname of the reference frame this composite is expressed in, or {@code null} if it
    *         is expressed in world frame.
    */
   public abstract String getReferenceFrame();

   @Override
   public final boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof YoCompositeDefinition)
      {
         YoCompositeDefinition other = (YoCompositeDefinition) object;

         if (getType() == null ? other.getType() != null : !getType().equals(other.getType()))
            return false;

         String[] thisIDs = getComponentIdentifiers();
         String[] otherIDs = other.getComponentIdentifiers();

         if (thisIDs == null ? otherIDs != null : !Arrays.equals(thisIDs, otherIDs))
            return false;

         String[] thisValues = getComponentValues();
         String[] otherValues = other.getComponentValues();

         if ((thisValues == null ? otherValues != null : !Arrays.equals(thisValues, otherValues)) || (getReferenceFrame() == null ? other.getReferenceFrame() != null : !getReferenceFrame().equals(other.getReferenceFrame())))
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }

   /**
    * Returns a {@code String} representation of this composite.
    * <p>
    * The returned string can later be used for parsing the composite back using the
    * {@code parse(String)} method from the relevant class.
    * </p>
    */
   @Override
   public final String toString()
   {
      String description = getType() + "(";
      String[] ids = getComponentIdentifiers();
      String[] values = getComponentValues();
      for (int i = 0; i < ids.length; i++)
      {
         if (i > 0)
            description += ", ";
         description += "%s=%s".formatted(ids[i], values[i]);
      }
      description += ", frame=" + getReferenceFrame() + ")";
      return description;
   }
}
