package us.ihmc.scs2.sessionVisualizer.jfx.controllers.yoPieChart;

import javafx.animation.AnimationTimer;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.chart.PieChart;
import javafx.scene.chart.PieChart.Data;
import javafx.scene.control.Button;
import javafx.scene.input.MouseEvent;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import us.ihmc.scs2.sessionVisualizer.jfx.controllers.VisualizerController;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerWindowToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class YoPieChartController implements VisualizerController
{
   @FXML
   public Button saveButton;

   @FXML
   public Button loadButton;
   public Button clearButton;

   @FXML
   private YoPieChartVariableController variable0Controller, variable1Controller, variable2Controller, variable3Controller, variable4Controller, variable5Controller, variable6Controller, variable7Controller, variable8Controller, variable9Controller, variable10Controller, variable11Controller;

   private List<YoPieChartVariableController> variableControllers;
   private final List<YoVariable> yoVariablesInPieChart = new ArrayList<>();

   @FXML
   private PieChart pieChart;

   @FXML
   private Stage stage;

   @Override
   public void initialize(SessionVisualizerWindowToolkit toolkit)
   {
      variableControllers = Arrays.asList(variable0Controller,
                                          variable1Controller,
                                          variable2Controller,
                                          variable3Controller,
                                          variable4Controller,
                                          variable5Controller,
                                          variable6Controller,
                                          variable7Controller,
                                          variable8Controller,
                                          variable9Controller,
                                          variable10Controller,
                                          variable11Controller);

      for (int i = 0; i < variableControllers.size(); i++)
      {
         YoPieChartVariableController variableController = variableControllers.get(i);
         variableController.initialize(toolkit.getGlobalToolkit(), pieChart);
      }

      AnimationTimer animationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            if (pieChart.getData().isEmpty())
               return;

            // Get all the YoVariables held in the individual controllers to always stay synced
            for (int i = 0; i < variableControllers.size(); i++)
            {
               YoPieChartVariableController variableController = variableControllers.get(i);

               yoVariablesInPieChart.add(variableController.getYoVariable());
            }

            // Disable animations on the PieChart by setting the CSS property
            pieChart.setStyle("-fx-pie-animation-time: 0ms;");

            // For YoVariables that exist, update the data held in the pie chart
            ObservableList<Data> currentData = pieChart.getData();
            for (PieChart.Data data : currentData)
            {
               String name = data.getName();

               for (YoVariable yoVariable : yoVariablesInPieChart)
               {
                  // We allow the YoVariables to be null, so we skip those
                  if (yoVariable != null && yoVariable.getName().equals(name))
                  {
                     data.setPieValue(yoVariable.getValueAsDouble());
                     break;
                  }
               }
            }
         }
      };

      animationTimer.start();

      stage.initOwner(toolkit.getWindow());
      stage.show();
      JavaFXMissingTools.centerWindowInOwner(stage, toolkit.getWindow());
   }

   @FXML
   public void loadLayout(MouseEvent mouseEvent)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setTitle("Load Layout");

      // Add custom extension filter
      fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Pie Chart Files", "*.pie.chart"));

      // Open file dialog to let the user choose the layout to load
      Stage stage = (Stage) ((javafx.scene.Node) mouseEvent.getSource()).getScene().getWindow();
      File file = fileChooser.showOpenDialog(stage);

      if (file != null)
      {
         loadFromXML(file);
      }
   }

   @FXML
   public void clearChart(MouseEvent mouseEvent)
   {
      yoVariablesInPieChart.clear();

      for (YoPieChartVariableController variableController : variableControllers)
      {
         variableController.clear();
      }
      pieChart.getData().clear();
   }

   private void loadFromXML(File file)
   {
      try
      {
         DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
         DocumentBuilder docBuilder = docFactory.newDocumentBuilder();
         Document doc = docBuilder.parse(file);

         // Normalize the XML structure (optional but recommended)
         doc.getDocumentElement().normalize();

         // Get the root element (YoChartGroupConfigurationList)
         Element rootElement = doc.getDocumentElement();

         // Get the list of all chartConfigurations elements
         NodeList chartConfigList = rootElement.getElementsByTagName("chartConfigurations");

         // Iterate through the chartConfigurations elements
         for (int i = 0; i < chartConfigList.getLength(); i++)
         {
            Node chartConfigNode = chartConfigList.item(i);

            if (chartConfigNode.getNodeType() == Node.ELEMENT_NODE)
            {
               Element chartConfigElement = (Element) chartConfigNode;

               // Get the yoVariables element within the chartConfigurations
               NodeList yoVariablesList = chartConfigElement.getElementsByTagName("yoVariables");

               if (yoVariablesList.getLength() > 0)
               {
                  String yoVariableText = yoVariablesList.item(0).getTextContent();

                  // Assuming variableControllers is already initialized and corresponds to a grid layout
                  if (i < variableControllers.size())
                  {
                     YoPieChartVariableController controller = variableControllers.get(i);
                     controller.setYoVariable(yoVariableText);
                  }
               }
            }
         }

         System.out.println("Layout loaded successfully!");
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   @FXML
   public void saveLayout(MouseEvent mouseEvent)
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setTitle("Save Layout");
      fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Pie Chart Files", "*.pie.chart"));

      Stage stage = (Stage) ((javafx.scene.Node) mouseEvent.getSource()).getScene().getWindow();
      File file = fileChooser.showSaveDialog(stage);

      if (file != null)
      {
         String filePath = file.getAbsolutePath();
         if (!filePath.endsWith(".pie.chart"))
         {
            file = new File(filePath + ".pie.chart");
         }
         saveToXML(file);
      }
   }

   private void saveToXML(File file)
   {
      try
      {
         DocumentBuilderFactory builderFactory = DocumentBuilderFactory.newInstance();
         DocumentBuilder documentBuilder = builderFactory.newDocumentBuilder();

         Document doc = documentBuilder.newDocument();
         Element rootElement = doc.createElement("YoChartGroupConfigurationList");
         rootElement.setAttribute("name", "pieChartLayout");
         doc.appendChild(rootElement);

         // Create XML structure for each variable controller
         for (YoPieChartVariableController controller : variableControllers)
         {
            if (controller.getYoVariable() != null)
            {
               Element chartConfiguration = doc.createElement("chartConfigurations");

               // Add the YoVariable
               controller.getYoVariable().getType();
               Element yoVariable = doc.createElement("yoVariables");
               yoVariable.appendChild(doc.createTextNode(controller.getYoVariable().getFullNameString()));
               chartConfiguration.appendChild(yoVariable);

               // Add more elements if needed (chart style, identifier, bounds, etc.)
               rootElement.appendChild(chartConfiguration);  // Append to root element
            }
         }

         // Write the content into the XML file
         TransformerFactory transformerFactory = TransformerFactory.newInstance();
         Transformer transformer = transformerFactory.newTransformer();
         transformer.setOutputProperty(OutputKeys.INDENT, "yes");
         DOMSource source = new DOMSource(doc);
         StreamResult result = new StreamResult(file);

         transformer.transform(source, result);

         System.out.println("Layout saved successfully!");
      }
      catch (ParserConfigurationException | javax.xml.transform.TransformerException e)
      {
         e.printStackTrace();
      }
   }
}
