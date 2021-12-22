package us.ihmc.scs2.examples.sessionVisualizer.jfx.yoGraphic;

import java.util.stream.Collectors;
import java.util.stream.DoubleStream;

import javafx.beans.property.SimpleDoubleProperty;
import javafx.scene.paint.Color;
import us.ihmc.scs2.examples.sessionVisualizer.jfx.Simple3DViewer;
import us.ihmc.scs2.sessionVisualizer.jfx.yoGraphic.YoPolynomialFX3D;

public class YoPolynomialFX3DVisualizer
{
   public static void main(String[] args)
   {
      YoPolynomialFX3D yoPolynomialFX3D = new YoPolynomialFX3D();
      double[] coefficientsX = new double[]{-0.5, 0.0, 22.270833333333286, -79.08333333333294, 96.35416666666606, -38.541666666666394};
      double[] coefficientsY = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      double[] coefficientsZ = new double[]{0.0, 0.0, 13.020833333333286, -52.08333333333307, 65.10416666666626, -26.041666666666483};
      yoPolynomialFX3D.setCoefficientsX(DoubleStream.of(coefficientsX).mapToObj(SimpleDoubleProperty::new).collect(Collectors.toList()));
      yoPolynomialFX3D.setCoefficientsY(DoubleStream.of(coefficientsY).mapToObj(SimpleDoubleProperty::new).collect(Collectors.toList()));
      yoPolynomialFX3D.setCoefficientsZ(DoubleStream.of(coefficientsZ).mapToObj(SimpleDoubleProperty::new).collect(Collectors.toList()));
      yoPolynomialFX3D.setNumberOfCoefficientsX(yoPolynomialFX3D.getCoefficientsX().size());
      yoPolynomialFX3D.setNumberOfCoefficientsY(yoPolynomialFX3D.getCoefficientsY().size());
      yoPolynomialFX3D.setNumberOfCoefficientsZ(yoPolynomialFX3D.getCoefficientsZ().size());
      yoPolynomialFX3D.setStartTime(0.0);
      yoPolynomialFX3D.setEndTime(1.0);
      yoPolynomialFX3D.setSize(0.02);
      yoPolynomialFX3D.setColor(Color.AQUAMARINE);
      yoPolynomialFX3D.setTimeResolution(128);
      yoPolynomialFX3D.render();
      yoPolynomialFX3D.computeBackground();
      yoPolynomialFX3D.render();
      Simple3DViewer.view3DObjects(yoPolynomialFX3D.getNode());
   }
}
