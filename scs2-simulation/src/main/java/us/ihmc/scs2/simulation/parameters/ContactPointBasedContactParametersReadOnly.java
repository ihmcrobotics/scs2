package us.ihmc.scs2.simulation.parameters;

public interface ContactPointBasedContactParametersReadOnly
{
   double getKxy();

   double getBxy();

   double getKz();

   double getBz();

   double getStiffeningLength();

   double getAlphaSlip();

   double getAlphaStick();

   boolean isSlipEnabled();

   // TODO This is not the best place for this parameter, but the whole thing needs to be refactored.
   boolean isJointWrenchCalculationEnabled();
}
