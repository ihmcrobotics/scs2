package us.ihmc.scs2.simulation.parameters;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoContactPointBasedContactParameters implements ContactPointBasedContactParametersBasics
{
   private YoDouble kxy;
   private YoDouble bxy;
   private YoDouble kz;
   private YoDouble bz;
   private YoDouble stiffeningLength;
   private YoDouble alphaSlip;
   private YoDouble alphaStick;
   private YoBoolean enableSlip;

   public YoContactPointBasedContactParameters(String prefix, YoRegistry registry)
   {
      String kxyName = "kxy";
      String bxyName = "bxy";
      String kzName = "kz";
      String bzName = "bz";
      String stiffLengthName = "stiffeningLength";
      String alphaSlipName = "alphaSlip";
      String alphaStickName = "alphaStick";
      String enableSlipName = "enableSlip";

      if (prefix != null && !prefix.isEmpty())
      {
         kxyName = prefix + StringUtils.capitalize(kxyName);
         bxyName = prefix + StringUtils.capitalize(bxyName);
         kzName = prefix + StringUtils.capitalize(kzName);
         bzName = prefix + StringUtils.capitalize(bzName);
         stiffLengthName = prefix + StringUtils.capitalize(stiffLengthName);
         alphaSlipName = prefix + StringUtils.capitalize(alphaSlipName);
         alphaStickName = prefix + StringUtils.capitalize(alphaStickName);
         enableSlipName = prefix + StringUtils.capitalize(enableSlipName);
      }

      kxy = new YoDouble(kxyName, registry);
      bxy = new YoDouble(bxyName, registry);
      kz = new YoDouble(kzName, registry);
      bz = new YoDouble(bzName, registry);
      stiffeningLength = new YoDouble(stiffLengthName, registry);
      alphaSlip = new YoDouble(alphaSlipName, registry);
      alphaStick = new YoDouble(alphaStickName, registry);
      enableSlip = new YoBoolean(enableSlipName, registry);
   }

   @Override
   public void setKxy(double kxy)
   {
      this.kxy.set(kxy);
   }

   @Override
   public void setBxy(double bxy)
   {
      this.bxy.set(bxy);
   }

   @Override
   public void setKz(double kz)
   {
      this.kz.set(kz);
   }

   @Override
   public void setBz(double bz)
   {
      this.bz.set(bz);
   }

   @Override
   public void setStiffeningLength(double stiffeningLength)
   {
      this.stiffeningLength.set(stiffeningLength);
   }

   @Override
   public void setAlphaSlip(double alphaSlip)
   {
      this.alphaSlip.set(alphaSlip);
   }

   @Override
   public void setAlphaStick(double alphaStick)
   {
      this.alphaStick.set(alphaStick);
   }

   @Override
   public void setEnableSlip(boolean enableSlip)
   {
      this.enableSlip.set(enableSlip);
   }

   @Override
   public double getKxy()
   {
      return kxy.getValue();
   }

   @Override
   public double getBxy()
   {
      return bxy.getValue();
   }

   @Override
   public double getKz()
   {
      return kz.getValue();
   }

   @Override
   public double getBz()
   {
      return bz.getValue();
   }

   @Override
   public double getStiffeningLength()
   {
      return stiffeningLength.getValue();
   }

   @Override
   public double getAlphaSlip()
   {
      return alphaSlip.getValue();
   }

   @Override
   public double getAlphaStick()
   {
      return alphaStick.getValue();
   }

   @Override
   public boolean isSlipEnabled()
   {
      return enableSlip.getValue();
   }
}
