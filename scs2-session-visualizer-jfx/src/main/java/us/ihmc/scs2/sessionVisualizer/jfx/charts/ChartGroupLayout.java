package us.ihmc.scs2.sessionVisualizer.jfx.charts;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javafx.util.Pair;
import us.ihmc.scs2.sessionVisualizer.jfx.yoComposite.YoComposite;
import us.ihmc.yoVariables.variable.YoVariable;

public class ChartGroupLayout extends ChartGroupModel
{
   private final Map<ChartIdentifier, List<YoVariable>> layout;

   public ChartGroupLayout(ChartGroupModel model, YoComposite yoComposite)
   {
      super(model);

      List<YoVariable> yoComponents = yoComposite.getYoComponents();
      if (model.getChartIdentifiers().size() != yoComponents.size())
         throw new IllegalArgumentException("Given model does not match yoComposite: " + yoComposite.getPattern().getType());

      layout = new HashMap<>();

      List<ChartIdentifier> chartIdentifiers = getChartIdentifiers();

      for (int i = 0; i < chartIdentifiers.size(); i++)
      {
         ChartIdentifier chartIdentifier = chartIdentifiers.get(i);
         List<YoVariable> yoVariables = layout.get(chartIdentifier);
         if (yoVariables == null)
         {
            yoVariables = new ArrayList<>();
            layout.put(chartIdentifier, yoVariables);
         }
         yoVariables.add(yoComponents.get(i));
      }
   }

   public ChartGroupLayout(String name, Map<ChartIdentifier, List<? extends YoVariable>> layout)
   {
      super(name, new ArrayList<>(layout.keySet()));
      this.layout = new LinkedHashMap<>();
      layout.entrySet().forEach(entry -> this.layout.put(entry.getKey(), new ArrayList<>(entry.getValue())));
   }

   public List<YoVariable> getYoVariables(ChartIdentifier chartIdentifier)
   {
      return layout.get(chartIdentifier);
   }

   public ChartGroupLayout shift(int rowShift, int columnShift)
   {
      return new ChartGroupLayout(getName(),
                                  layout.entrySet().stream().map(entry -> new Pair<>(entry.getKey().shift(rowShift, columnShift), entry.getValue()))
                                        .collect(Collectors.toMap(Pair::getKey, Pair::getValue)));
   }

   @Override
   public int hashCode()
   {
      return layout.hashCode();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ChartGroupLayout)
      {
         ChartGroupLayout other = (ChartGroupLayout) object;
         return layout.equals(other.layout);
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return getName() + ": " + layout.toString();
   }
}
