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

   public YoQuaternionDefinition()
   {
   }

   public YoQuaternionDefinition(String x, String y, String z, String s)
   {
      this(x, y, z, s, null);
   }

   public YoQuaternionDefinition(String x, String y, String z, String s, String referenceFrame)
   {
      this.x = x;
      this.y = y;
      this.z = z;
      this.s = s;
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

   public static YoQuaternionDefinition parse(String value)
   {
      if (value == null)
         return null;

      value = value.trim();

      if (value.startsWith(YoQuaternion))
      {
         value = value.substring(value.indexOf("=") + 1).trim();
         String x = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String y = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String z = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String s = value.substring(0, value.indexOf(","));
         value = value.substring(value.indexOf("=") + 1).trim();
         String frame = value.substring(0, value.length() - 1);

         if (x.toLowerCase().equals("null"))
            x = null;
         if (y.toLowerCase().equals("null"))
            y = null;
         if (z.toLowerCase().equals("null"))
            z = null;
         if (s.toLowerCase().equals("null"))
            s = null;
         if (frame.toLowerCase().equals("null"))
            frame = null;

         return new YoQuaternionDefinition(x, y, z, s, frame);
      }
      else
      {
         throw new IllegalArgumentException("Unknown quaternion format: " + value);
      }
   }
}
