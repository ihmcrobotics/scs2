package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlSeeAlso;

@XmlSeeAlso({YoQuaternionDefinition.class, YoYawPitchRollDefinition.class})
public abstract class YoOrientation3DDefinition extends YoCompositeDefinition
{
   public static YoOrientation3DDefinition parse(String value)
   {
      value = value.trim();

      if (value.startsWith(YoQuaternionDefinition.YoQuaternion))
         return YoQuaternionDefinition.parse(value);
      if (value.startsWith(YoYawPitchRollDefinition.YoYawPitchRoll))
         return YoYawPitchRollDefinition.parse(value);

      throw new IllegalArgumentException("Unknown orientation 3D format: " + value);
   }
}
