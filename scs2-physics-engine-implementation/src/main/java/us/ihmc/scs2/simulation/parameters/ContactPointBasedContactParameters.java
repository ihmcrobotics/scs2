package us.ihmc.scs2.simulation.parameters;

public class ContactPointBasedContactParameters implements ContactPointBasedContactParametersBasics
{
   private double kxy;
   private double bxy;
   private double kz;
   private double bz;
   private double stiffeningLength;
   private double alphaSlip;
   private double alphaStick;
   private boolean enableSlip;

   public static ContactPointBasedContactParameters defaultParameters()
   {
      ContactPointBasedContactParameters parameters = new ContactPointBasedContactParameters();

      parameters.setKxy(16000.0);//1422.0);
      parameters.setBxy(650.0);//15.6);
      parameters.setKz(650.0);//125.0);
      parameters.setBz(500.0);//300.0);
      parameters.setStiffeningLength(0.008);
      parameters.setAlphaSlip(0.7);
      parameters.setAlphaStick(0.7);
      parameters.setEnableSlip(false);

      return parameters;
   }

   @Override
   public void setKxy(double kxy)
   {
      this.kxy = kxy;
   }

   @Override
   public void setBxy(double bxy)
   {
      this.bxy = bxy;
   }

   @Override
   public void setKz(double kz)
   {
      this.kz = kz;
   }

   @Override
   public void setBz(double bz)
   {
      this.bz = bz;
   }

   @Override
   public void setStiffeningLength(double stiffeningLength)
   {
      this.stiffeningLength = stiffeningLength;
   }

   @Override
   public void setAlphaSlip(double alphaSlip)
   {
      this.alphaSlip = alphaSlip;
   }

   @Override
   public void setAlphaStick(double alphaStick)
   {
      this.alphaStick = alphaStick;
   }

   @Override
   public void setEnableSlip(boolean enableSlip)
   {
      this.enableSlip = enableSlip;
   }

   @Override
   public double getKxy()
   {
      return kxy;
   }

   @Override
   public double getBxy()
   {
      return bxy;
   }

   @Override
   public double getKz()
   {
      return kz;
   }

   @Override
   public double getBz()
   {
      return bz;
   }

   @Override
   public double getStiffeningLength()
   {
      return stiffeningLength;
   }

   @Override
   public double getAlphaSlip()
   {
      return alphaSlip;
   }

   @Override
   public double getAlphaStick()
   {
      return alphaStick;
   }

   @Override
   public boolean isSlipEnabled()
   {
      return enableSlip;
   }
}
