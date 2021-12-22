package us.ihmc.scs2.definition.yoComposite;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlRootElement;

@XmlRootElement(name = "YoTuple2D")
public class YoTuple2DDefinition extends YoCompositeDefinition
{
   public static final String YoTuple2D = "YoTuple2D";
   public static final String[] YoTuple2DIdentifiers = new String[] {"x", "y"};

   private String x, y;
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

   @Override
   public String getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public String getType()
   {
      return YoTuple2D;
   }

   @Override
   public String[] getComponentIdentifiers()
   {
      return YoTuple2DIdentifiers;
   }

   @Override
   public String[] getComponentValues()
   {
      return new String[] {x, y};
   }
}
