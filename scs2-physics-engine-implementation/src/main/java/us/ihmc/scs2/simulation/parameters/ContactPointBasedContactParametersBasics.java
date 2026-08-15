package us.ihmc.scs2.simulation.parameters;

public interface ContactPointBasedContactParametersBasics extends ContactPointBasedContactParametersReadOnly
{
   default void set(ContactPointBasedContactParametersReadOnly other)
   {
      setKxy(other.getKxy());
      setBxy(other.getBxy());
      setKz(other.getKz());
      setBz(other.getBz());
      setStiffeningLength(other.getStiffeningLength());
      setAlphaSlip(other.getAlphaSlip());
      setAlphaStick(other.getAlphaStick());
      setEnableSlip(other.isSlipEnabled());
   }

   void setKxy(double kxy);

   void setBxy(double bxy);

   void setKz(double kz);

   void setBz(double bz);

   void setStiffeningLength(double stiffeningLength);

   void setAlphaSlip(double alphaSlip);

   void setAlphaStick(double alphaStick);

   void setEnableSlip(boolean enableSlip);
}
