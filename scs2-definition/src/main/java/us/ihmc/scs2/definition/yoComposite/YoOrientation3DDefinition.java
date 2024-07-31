package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlSeeAlso;

/**
 * A {@code YoOrientation3DDefinition} is used to represent the template for a 3D orientation.
 * <p>
 * Each component can be backed by a {@code YoVariable}, in which case the component is set to the
 * variable name/fullname, or is a constant value, which case it is set to the string representation
 * of the value.
 * </p>
 *
 * @author Sylvain Bertrand
 */
@XmlSeeAlso({YoQuaternionDefinition.class, YoYawPitchRollDefinition.class})
public abstract class YoOrientation3DDefinition extends YoCompositeDefinition
{
   public abstract YoOrientation3DDefinition copy();

   /**
    * Parses the given {@code value} into a {@link YoOrientation3DDefinition}. The given {@code String}
    * representation is expected to have been generated using {@link #toString()}. If the format
    * differs, this method will throw an {code IllegalArgumentException}.
    *
    * @param value the {@code String} representation of a {@link YoOrientation3DDefinition}.
    * @return the parsed orientation 3D object.
    */
   public static YoOrientation3DDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoQuaternionDefinition.YoQuaternion))
         return YoQuaternionDefinition.parse(value);
      if (value.startsWith(YoYawPitchRollDefinition.YoYawPitchRoll))
         return YoYawPitchRollDefinition.parse(value);

      throw new IllegalArgumentException("Unknown orientation 3D format: " + value);
   }
}
