package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlSeeAlso;

@XmlSeeAlso({YoQuaternionDefinition.class, YoYawPitchRollDefinition.class})
public abstract class YoOrientation3DDefinition extends YoCompositeDefinition
{
}
