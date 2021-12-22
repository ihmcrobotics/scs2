package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoQuaternion")
public class YoQuaternionDefinition extends YoOrientation3DDefinition
{
   public static final String YoQuaternion = "YoQuaternion";
   public static final String[] YoQuaternionIdentifiers = new String[] {"qx", "qy", "qz", "qs"};

   private String x, y, z, s;
   private String referenceFrame;

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

   public void setS(double s)
   {
      this.s = Double.toString(s);
   }

   @XmlElement
   public void setS(String s)
   {
      this.s = s;
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

   public String getS()
   {
      return s;
   }

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoQuaternion;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoQuaternionIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {x, y, z, s};
   }
}
