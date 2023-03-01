package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoTuple3D")
public class YoTuple3DDefinition extends YoCompositeDefinition
{
   public static final String YoTuple3D = "YoTuple3D";
   public static final String[] YoTuple3DIdentifiers = new String[] {"x", "y", "z"};

   private String x, y, z;
   private String referenceFrame;

   public YoTuple3DDefinition()
   {
   }

   public YoTuple3DDefinition(String x, String y, String z)
   {
      this(x, y, z, null);
   }

   public YoTuple3DDefinition(String x, String y, String z, String referenceFrame)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.referenceFrame = referenceFrame;
   }

   public void setX(double x)
   {
      this.x = Double.toString(x);
   }

   @XmlElement
   public void setX(String x)
   {
      this.x = x;
   }

   public void setY(double y)
   {
      this.y = Double.toString(y);
   }

   @XmlElement
   public void setY(String y)
   {
      this.y = y;
   }

   public void setZ(double z)
   {
      this.z = Double.toString(z);
   }

   @XmlElement
   public void setZ(String z)
   {
      this.z = z;
   }

   @XmlElement
   public void setReferenceFrame(String referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public String getX()
   {
      return x;
   }

   public String getY()
   {
      return y;
   }

   public String getZ()
   {
      return z;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoTuple3D;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoTuple3DIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {x, y, z};
   }
}
