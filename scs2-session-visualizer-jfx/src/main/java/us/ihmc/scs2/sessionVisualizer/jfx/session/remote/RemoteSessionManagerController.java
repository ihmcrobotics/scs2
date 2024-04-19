package us.ihmc.scs2.sessionVisualizer.jfx.session.remote;

import com.jfoenix.controls.JFXTreeTableColumn;
import com.jfoenix.controls.RecursiveTreeItem;
import com.jfoenix.controls.datamodels.treetable.RecursiveTreeObject;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.collections.FXCollections;
import javafx.collections.ListChangeListener;
import javafx.collections.ObservableList;
import javafx.collections.ObservableMap;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.ProgressIndicator;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TreeItem;
import javafx.scene.control.TreeTableColumn;
import javafx.scene.control.TreeTableView;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.robotDataLogger.StaticHostListLoader;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.websocket.client.discovery.DataServerDiscoveryClient;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;
import us.ihmc.scs2.session.remote.FunctionalDataServerDiscoveryListener;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerIOTools;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerTopics;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.SessionVisualizerToolkit;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionControlsController;
import us.ihmc.scs2.sessionVisualizer.jfx.session.SessionInfoController;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.IntegerConverter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.JavaFXMissingTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.MenuTools;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.PositiveIntegerValueFilter;
import us.ihmc.scs2.sessionVisualizer.jfx.tools.TreeTableViewTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;

public class RemoteSessionManagerController implements SessionControlsController
{
   private final static String OFFLINE_HOSTNAME_DESCRIPTION = "[Offline]";
   private final static String OFFLINE_SESSIONNAME_DESCRIPTION = "";
   public static final int DEFAULT_TIMEOUT = 25000; //ms

   private final static Integer DEFAULT_PORT = 8008;

   @FXML
   private AnchorPane mainPane;
   @FXML
   private TreeTableView<SessionInfo> sessionTreeTableView;
   @FXML
   private Pane staticControlsPane;
   @FXML
   private TextField staticHostTextField, staticPortTextField;
   @FXML
   private Button createStaticHostButton;

   @FXML
   private ProgressIndicator loadingSpinner;
   @FXML
   private Button startSessionButton, endSessionButton;

   private YoVariableClient client;
   private final RemoteSessionFactory sessionFactory = new RemoteSessionFactory();

   private TreeItem<SessionInfo> rootSession;
   private ObservableMap<HTTPDataServerDescription, TreeItem<SessionInfo>> descriptionToTreeItemMap = FXCollections.observableMap(new HashMap<>());

   private final ObjectProperty<HTTPDataServerDescription> staticDescriptionProperty = new SimpleObjectProperty<>(this, "staticDescription", null);
   private final ObservableList<HTTPDataServerDescription> registeredStaticDescriptions = FXCollections.observableArrayList();

   private final BooleanProperty sessionInProgressProperty = new SimpleBooleanProperty(this, "sessionInProgress", false);

   private DataServerDiscoveryClient discoveryClient;
   private BackgroundExecutorManager backgroundExecutorManager;
   private YoClientInformationPaneController informationPaneController;

   private Stage stage;

   @SuppressWarnings("unchecked")
   public void initialize(SessionVisualizerToolkit toolkit)
   {
      this.backgroundExecutorManager = toolkit.getBackgroundExecutorManager();
      client = new YoVariableClient(sessionFactory);

      mainPane.getStylesheets().add(SessionVisualizerIOTools.GENERAL_STYLESHEET.toExternalForm());

      TreeTableColumn<SessionInfo, String> hostColumn = createColumn("Host", 150.0, 100.0, 200.0, SessionInfo::getHost);
      TreeTableColumn<SessionInfo, String> portColumn = createColumn("Port", 80.0, SessionInfo::getPort);
      TreeTableColumn<SessionInfo, String> hostNameColumn = createColumn("HostName", 175.0, 100.0, 250.0, SessionInfo::getHostName);
      TreeTableColumn<SessionInfo, String> sessionNameColumn = createColumn("SessionName", 250.0, 200.0, 500.0, SessionInfo::getSessionName);

      rootSession = new RecursiveTreeItem<>(FXCollections.observableArrayList(), RecursiveTreeObject<SessionInfo>::getChildren);

      sessionTreeTableView.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);
      sessionTreeTableView.setRoot(rootSession);
      sessionTreeTableView.setShowRoot(false);
      sessionTreeTableView.getColumns().setAll(hostColumn, portColumn, hostNameColumn, sessionNameColumn);

      TextFormatter<Integer> portFormatter = new TextFormatter<>(new IntegerConverter(), DEFAULT_PORT, new PositiveIntegerValueFilter());
      staticPortTextField.setTextFormatter(portFormatter);

      createStaticHostButton.setDisable(true);

      staticHostTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                     {
                                                        createStaticHostButton.setDisable(isEmpty(newValue) || isEmpty(staticPortTextField.getText()));
                                                     });
      staticPortTextField.textProperty().addListener((o, oldValue, newValue) ->
                                                     {
                                                        createStaticHostButton.setDisable(isEmpty(newValue) || isEmpty(staticPortTextField.getText()));
                                                     });

      createStaticHostButton.setOnAction(e ->
                                         {
                                            staticDescriptionProperty.set(new HTTPDataServerDescription(staticHostTextField.getText(),
                                                                                                        Integer.parseInt(staticPortTextField.getText()),
                                                                                                        null,
                                                                                                        true));
                                         });

      staticDescriptionProperty.addListener((o, oldValue, newValue) ->
                                            {
                                               if (newValue == null)
                                               {
                                                  staticHostTextField.setText(null);
                                                  portFormatter.setValue(DEFAULT_PORT);
                                               }
                                               else
                                               {
                                                  staticHostTextField.setText(newValue.getHost());
                                                  portFormatter.setValue(newValue.getPort());
                                               }
                                            });

      rootSession.getChildren().addListener(new ListChangeListener<TreeItem<SessionInfo>>()
      {
         @Override
         public void onChanged(Change<? extends TreeItem<SessionInfo>> c)
         {
            while (c.next())
            {
               if (c.wasRemoved())
                  c.getRemoved().forEach(item -> descriptionToTreeItemMap.remove(item.getValue().getDescription()));
               if (c.wasAdded())
                  c.getAddedSubList().forEach(item -> descriptionToTreeItemMap.put(item.getValue().getDescription(), item));
            }
         }
      });

      discoveryClient = new DataServerDiscoveryClient((FunctionalDataServerDiscoveryListener) connection -> updateConnection(connection), true);

      staticDescriptionProperty.addListener((o, oldValue, newValue) ->
                                            {
                                               if (newValue != null && newValue.getHost() != null)
                                               {
                                                  LogTools.info("Adding description");
                                                  addDescription(newValue);
                                                  staticDescriptionProperty.set(null);
                                               }
                                            });

      MenuTools.setupContextMenu(sessionTreeTableView,
                                 TreeTableViewTools.removeMenuItemFactory(false,
                                                                          sessionInfo -> registeredStaticDescriptions.contains(sessionInfo.getDescription()),
                                                                          sessionInfo -> registeredStaticDescriptions.remove(sessionInfo.getDescription())));

      startSessionButton.setDisable(true);
      endSessionButton.disableProperty().bind(sessionInProgressProperty.not());
      sessionTreeTableView.disableProperty().bind(sessionInProgressProperty);
      staticControlsPane.disableProperty().bind(sessionInProgressProperty);

      sessionInProgressProperty.addListener((o, oldValue, newValue) -> startSessionButton.setDisable(newValue));

      loadingSpinner.disabledProperty().addListener((o, oldValue, newValue) ->
                                                    {
                                                       if (newValue)
                                                          startSessionButton.setDisable(true);
                                                    });

      sessionTreeTableView.getSelectionModel().selectedItemProperty().addListener((o, oldValue, newValue) ->
                                                                                  {
                                                                                     if (newValue == null)
                                                                                     {
                                                                                        startSessionButton.setDisable(true);
                                                                                     }
                                                                                     else
                                                                                     {
                                                                                        HTTPDataServerConnection connection = newValue.getValue()
                                                                                                                                      .getConnection();
                                                                                        if (connection == null)
                                                                                           startSessionButton.setDisable(true);
                                                                                        else
                                                                                           startSessionButton.setDisable(!connection.isConnected());
                                                                                     }
                                                                                  });

      StaticHostListLoader.load().forEach(this::addDescription);
      registeredStaticDescriptions.addListener((ListChangeListener<HTTPDataServerDescription>) c ->
      {
         try
         {
            StaticHostListLoader.save(new ArrayList<>(c.getList()));
            LogTools.info("Save static host list.");
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot save host list. " + e.getMessage());
         }
      });

      startSessionButton.setOnAction(e -> startSession());
      sessionTreeTableView.setOnMouseClicked(e ->
                                             {
                                                if (e.getClickCount() != 2)
                                                   return;
                                                startSession();
                                             });

      endSessionButton.setOnAction(e ->
                                   {
                                      stopSession();
                                   });

      try
      {
         FXMLLoader loader = new FXMLLoader(SessionVisualizerIOTools.REMOTE_SESSION_INFO_PANE_FXML_URL);
         loader.load();
         informationPaneController = loader.getController();
         informationPaneController.initialize();
         informationPaneController.start();
         informationPaneController.activeSessionProperty().bind(sessionFactory.activeSessionProperty());
         SessionVisualizerTopics topics = toolkit.getTopics();
         JavaFXMessager messager = toolkit.getMessager();
         sessionFactory.activeSessionProperty().addListener((o, oldValue, newValue) -> messager.submitMessage(topics.getStartNewSessionRequest(), newValue));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      stage = new Stage();
      stage.setScene(new Scene(mainPane));
      stage.setTitle("Active remote sessions");
      stage.getIcons().add(SessionVisualizerIOTools.REMOTE_SESSION_IMAGE);
      toolkit.getMainWindow().addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, e ->
      {
         if (!e.isConsumed())
            shutdown();
      });
   }

   private static boolean isEmpty(String query)
   {
      return query == null || query.isEmpty();
   }

   private void updateConnection(HTTPDataServerConnection connection)
   {
      if (connection.getTarget().isPersistant() || connection.isConnected())
         JavaFXMissingTools.runLater(getClass(), () -> addConnection(connection));
      else
         JavaFXMissingTools.runLater(getClass(), () -> removeConnection(connection));
   }

   private TreeItem<SessionInfo> addDescription(HTTPDataServerDescription description)
   {
      TreeItem<SessionInfo> treeItem = descriptionToTreeItemMap.get(description);
      if (treeItem == null)
      {
         SessionInfo sessionInfo = new SessionInfo();
         sessionInfo.setDescription(description);
         treeItem = new TreeItem<>(sessionInfo);
         rootSession.getChildren().add(treeItem);
         if (description.isPersistant())
         {
            registeredStaticDescriptions.add(description);
            discoveryClient.addHost(description);
         }
      }
      return treeItem;
   }

   private void addConnection(HTTPDataServerConnection connection)
   {
      TreeItem<SessionInfo> treeItem = addDescription(connection.getTarget());
      treeItem.getValue().setConnection(connection);
   }

   private void removeDescription(HTTPDataServerDescription description)
   {
      TreeItem<SessionInfo> treeItem = descriptionToTreeItemMap.get(description);
      if (treeItem == null)
         return;
      rootSession.getChildren().remove(treeItem);
      registeredStaticDescriptions.remove(description);
   }

   private void removeConnection(HTTPDataServerConnection connection)
   {
      removeDescription(connection.getTarget());
   }

   private void startSession()
   {
      TreeItem<SessionInfo> selectedItem = sessionTreeTableView.getSelectionModel().getSelectedItem();
      if (selectedItem == null)
         return;

      if (sessionInProgressProperty.get())
         unloadSession();
      setIsLoading(true);
      sessionInProgressProperty.set(true);

      backgroundExecutorManager.executeInBackground(() ->
                                                    {
                                                       try
                                                       {
                                                          client.start(DEFAULT_TIMEOUT, selectedItem.getValue().getConnection());
                                                       }
                                                       catch (Throwable e)
                                                       {
                                                          e.printStackTrace();
                                                          JavaFXMissingTools.runLater(getClass(), () -> unloadSession());
                                                       }
                                                    });
   }

   @Override
   public void notifySessionLoaded()
   {
      setIsLoading(false);
   }

   private void stopSession()
   {
      if (!sessionInProgressProperty.get())
         return;
      sessionFactory.unloadSession();
      try
      {
         client.stop();
      }
      catch (RuntimeException e)
      {
         // Just be silent about it, it's possible that there's no session ongoing in the case of a crash in startSession()
      }
      setIsLoading(false);
      sessionInProgressProperty.set(false);
   }

   @Override
   public void shutdown()
   {
      setIsLoading(false);
      sessionInProgressProperty.set(false);
      try
      {
         if (sessionFactory.activeSessionProperty().get() != null)
         {
            stopSession();
            ThreadTools.sleep(500);
         }

         client.stop();
      }
      catch (RuntimeException e)
      {
         // The current implementation throws a runtime exception when it's never open a connection.
      }
      discoveryClient.close();
      informationPaneController.stop();
      stage.close();
   }

   @Override
   public void unloadSession()
   {
      stopSession();
   }

   @Override
   public SessionInfoController getSessionInfoController()
   {
      return informationPaneController;
   }

   @Override
   public Stage getStage()
   {
      return stage;
   }

   private void setIsLoading(boolean isLoading)
   {
      JavaFXMissingTools.runLaterIfNeeded(getClass(), () -> loadingSpinner.setVisible(isLoading));
   }

   private TreeTableColumn<SessionInfo, String> createColumn(String name, double prefWidth, Function<SessionInfo, StringProperty> fieldProvider)
   {
      return createColumn(name, prefWidth, prefWidth, prefWidth, fieldProvider);
   }

   private TreeTableColumn<SessionInfo, String> createColumn(String name,
                                                             double prefWidth,
                                                             double minWidth,
                                                             double maxWidth,
                                                             Function<SessionInfo, StringProperty> fieldProvider)
   {
      TreeTableColumn<SessionInfo, String> column = new JFXTreeTableColumn<>(name);
      column.setPrefWidth(prefWidth);
      column.setMinWidth(minWidth);
      column.setMaxWidth(maxWidth);
      column.setCellValueFactory(param -> fieldProvider.apply(param.getValue().getValue()));
      return column;
   }

   public static class SessionInfo extends RecursiveTreeObject<SessionInfo>
   {
      private final StringProperty host = new SimpleStringProperty(this, "host", null);
      private final StringProperty port = new SimpleStringProperty(this, "port", null);
      private final StringProperty hostName = new SimpleStringProperty(this, "hostName", OFFLINE_HOSTNAME_DESCRIPTION);
      private final StringProperty sessionName = new SimpleStringProperty(this, "sessionName", OFFLINE_SESSIONNAME_DESCRIPTION);

      private final ObjectProperty<HTTPDataServerDescription> descriptionProperty = new SimpleObjectProperty<>(this, "description", null);
      private final ObjectProperty<HTTPDataServerConnection> connectionProperty = new SimpleObjectProperty<>(this, "connection", null);

      public SessionInfo()
      {
         descriptionProperty.addListener((o, oldValue, newValue) ->
                                         {
                                            if (newValue != null)
                                            {
                                               host.set(newValue.getHost());
                                               port.set(Integer.toString(newValue.getPort()));
                                               updateConnection(connectionProperty.get());
                                            }
                                         });

         connectionProperty.addListener((o, oldValue, newValue) -> updateConnection(newValue));
      }

      private void updateConnection(HTTPDataServerConnection connection)
      {
         if (connection != null && connection.isConnected())
         {
            hostName.set(connection.getAnnouncement().getHostNameAsString());
            sessionName.set(connection.getAnnouncement().getNameAsString());
         }
         else
         {
            hostName.set(OFFLINE_HOSTNAME_DESCRIPTION);
            sessionName.set(OFFLINE_SESSIONNAME_DESCRIPTION);
         }
      }

      public void setDescription(HTTPDataServerDescription description)
      {
         descriptionProperty.set(description);
      }

      public HTTPDataServerDescription getDescription()
      {
         return descriptionProperty.get();
      }

      public ObjectProperty<HTTPDataServerDescription> descriptionProperty()
      {
         return descriptionProperty;
      }

      public void setConnection(HTTPDataServerConnection connection)
      {
         connectionProperty.set(connection);
      }

      public HTTPDataServerConnection getConnection()
      {
         return connectionProperty.get();
      }

      public ObjectProperty<HTTPDataServerConnection> connectionProperty()
      {
         return connectionProperty;
      }

      public StringProperty getHost()
      {
         return host;
      }

      public StringProperty getPort()
      {
         return port;
      }

      public StringProperty getHostName()
      {
         return hostName;
      }

      public StringProperty getSessionName()
      {
         return sessionName;
      }
   }
}
