"""
Interactive viewer for UXsim simulation results using PyQt5.
After running the simulation, you can launch this viewer to visualize the simulation results interactively.
GUI part was mainly written by Claude 3 Opus under supervision of Toru Seo.

Examples
--------
Usage:
    >>> ... #define the World object W
    >>> W.exec_simulation()     #you run the simulation.
    >>> from uxsim.ResultGUIViewer import launch_World_viewer
    >>> launch_World_viewer(W)
"""

import sys
import numpy as np
from matplotlib import colormaps
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QGraphicsItem, QMenu, QSlider, QVBoxLayout, QWidget, QHBoxLayout, QLabel, QPushButton, QInputDialog, QMessageBox, QTableView, QDialog, QFileDialog
from PyQt5.QtGui import QPen, QColor, QPainter, QPainterPath
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer, QAbstractTableModel


class EdgeItem(QGraphicsItem):
    def __init__(self, name, start_node, end_node, density_list, Link):
        super().__init__()
        self.name = name
        self.start_node = start_node
        self.end_node = end_node
        self.density_list = density_list
        self.Link = Link
        self.setAcceptHoverEvents(True)
        self.t = 0
        self.curve_direction = -1
        self.show_name = True
        self.setZValue(0)

    def boundingRect(self):
        return self.shape().boundingRect()

    def shape(self):
        path = QPainterPath(self.start_node.pos())
        dx = self.end_node.pos().x() - self.start_node.pos().x()
        dy = self.end_node.pos().y() - self.start_node.pos().y()
        
        length = (dx**2 + dy**2)**0.5
        offset = length/10
        if dx == 0:
            offset = QPointF(-offset*self.curve_direction if dy > 0 else offset*self.curve_direction, 0)
        else:
            normal = QPointF(-dy, dx)
            normal /= (normal.x()**2 + normal.y()**2)**0.5
            offset = normal * (offset * self.curve_direction)

        control_point = (self.start_node.pos() + self.end_node.pos()) / 2 + offset
        path.quadTo(control_point, self.end_node.pos())
        return path

    def paint(self, painter, option, widget):
        path = self.shape()
        length = path.length()
        num_segments = self.density_list.shape[1]
        segment_length = length / num_segments
        maxlw = 10
        minlw = 0.5

        for i in range(num_segments):
            density = self.density_list[self.t, i]
            speed = self.Link.v_mat[self.t, i]
            lw = max([density*self.Link.delta*self.Link.lanes])*(maxlw-minlw)+minlw
            
            c = colormaps["viridis"](speed/self.Link.u)
            color = QColor(int(c[0]*255), int(c[1]*255), int(c[2]*255), 255)
            pen = QPen(color, lw)
            painter.setPen(pen)

            start_percent = i / num_segments
            end_percent = (i + 1) / num_segments
            start_point = path.pointAtPercent(start_percent)
            end_point = path.pointAtPercent(end_percent)
            segment_path = QPainterPath(start_point)
            segment_path.lineTo(end_point)

            painter.drawPath(segment_path)

        if self.show_name:
            # リンクの中央点を取得
            center_point = path.pointAtPercent(0.5)

            # テキストのバウンディングボックスを取得
            font_metrics = painter.fontMetrics()
            text_rect = font_metrics.boundingRect(self.name)

            # テキストの位置を計算
            text_x = center_point.x() - text_rect.width() / 2
            text_y = center_point.y() - text_rect.height() / 2

            # テキストを描画
            painter.setPen(Qt.blue)
            painter.drawText(int(text_x), int(text_y), self.name)

    # def time_space_diagram_traj(self):
    #     print(f"Hoge function called for edge: {self.Link}")
    #     self.Link.W.analyzer.time_space_diagram_traj(self.Link.name)

    # def time_space_diagram_density(self):
    #     print(f"Fuga function called for edge: {self.Link}")
    #     self.Link.W.analyzer.time_space_diagram_density(self.Link.name)
    
    def contextMenuEvent(self, event):
        menu = QMenu()
        menu.addAction(self.name)
        menu.addSeparator()
        action_time_space_diagram_traj = menu.addAction("Time-space diagram of vehicle trajectories")
        action_time_space_diagram_density = menu.addAction("Time-space diagram of density")
        action_cumu_curve = menu.addAction("Cumulative curve")
        action = menu.exec_(event.screenPos())
        if action == action_time_space_diagram_traj:
            self.Link.W.analyzer.time_space_diagram_traj(self.Link.name)
        elif action == action_time_space_diagram_density:
            self.Link.W.analyzer.time_space_diagram_density(self.Link.name)
        elif action == action_cumu_curve:
            self.Link.W.analyzer.cumulative_curves(self.Link)
            
    def set_curve_direction(self, direction):
        self.curve_direction = direction

    def set_show_name(self, show_name):
        self.show_name = show_name


class NodeItem(QGraphicsItem):
    def __init__(self, name, x, y, Node):
        super().__init__()
        self.name = name
        self.setPos(x, y)
        self.Node = Node
        self.show_name = True
        self.setZValue(1)

    def boundingRect(self):
        return QRectF(-10, -10, 20, 20)

    def paint(self, painter, option, widget):
        painter.setBrush(Qt.white)
        painter.drawEllipse(-10, -10, 20, 20)
        if self.show_name:
            painter.setPen(QColor(0, 128, 0))
            painter.drawText(-5, 5, self.name)
            
    def set_show_name(self, show_name):
        self.show_name = show_name


class VehicleItem(QGraphicsItem):
    def __init__(self, x, y):
        super().__init__()
        self.setPos(x, y)
        self.setZValue(2)

    def boundingRect(self):
        return QRectF(-5, -5, 10, 10)

    def paint(self, painter, option, widget):
        painter.setBrush(Qt.red)
        painter.drawEllipse(-5, -5, 10, 10)


class GraphWidget(QGraphicsView):
    def __init__(self, nodes, edges, vehicle_list):
        super().__init__()
        self.setRenderHint(QPainter.Antialiasing)
        self.setScene(QGraphicsScene(self))
        self.setSceneRect(0, 0, 800, 800)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)

        self.vehicle_list = vehicle_list
        self.show_vehicles = True

        for node_data in nodes:
            name, x, y, Node = node_data
            node = NodeItem(name, x, y, Node)
            self.scene().addItem(node)

        self.edges = []
        for edge_data in edges:
            name, start_node_name, end_node_name, density_list, Node = edge_data
            start_node = self.find_node(start_node_name)
            end_node = self.find_node(end_node_name)
            if start_node and end_node:
                edge = EdgeItem(name, start_node, end_node, density_list, Node)
                self.scene().addItem(edge)
                self.edges.append(edge)
        
        self.set_vehice_items()

    def set_vehice_items(self):
        self.vehicle_items = []
        if self.vehicle_list != None:
            for t, edge_name, x in self.vehicle_list:
                edge = self.find_edge(edge_name)
                if edge:
                    path = edge.shape()
                    point = path.pointAtPercent(x)
                    vehicle = VehicleItem(point.x(), point.y())
                    self.scene().addItem(vehicle)
                    self.vehicle_items.append(vehicle)

    def find_node(self, name):
        for item in self.scene().items():
            if isinstance(item, NodeItem) and item.name == name:
                return item
        return None

    def find_edge(self, name):
        for item in self.scene().items():
            if isinstance(item, EdgeItem) and item.name == name:
                return item
        return None

    def set_t(self, t):
        for edge in self.edges:
            edge.t = t
        if self.show_vehicles and self.vehicle_list != None:
            for vehicle, (vt, edge_name, x) in zip(self.vehicle_items, self.vehicle_list):
                if vt == t:
                    edge = self.find_edge(edge_name)
                    if edge:
                        path = edge.shape()
                        point = path.pointAtPercent(x)
                        vehicle.setPos(point.x(), point.y())
                    vehicle.setVisible(True)
                else:
                    vehicle.setVisible(False)
        self.viewport().update()
        
    def wheelEvent(self, event):
        zoom_factor = 1.15
        if event.angleDelta().y() > 0:
            self.scale(zoom_factor, zoom_factor)
        else:
            self.scale(1 / zoom_factor, 1 / zoom_factor)
        
    def set_curve_direction(self, direction):
        for edge in self.edges:
            edge.set_curve_direction(direction)
        self.viewport().update()

    def set_show_names(self, show_names):
        for item in self.scene().items():
            if isinstance(item, (NodeItem, EdgeItem)):
                item.set_show_name(show_names)
        self.viewport().update()

    def set_show_vehicles(self, show_vehicles):
        self.show_vehicles = show_vehicles
        for vehicle in self.vehicle_items:
            vehicle.setVisible(show_vehicles)
        self.viewport().update()

class MainWindow(QMainWindow):
    def __init__(self, W, nodes, edges, vehicle_list, tmax, dt):
        super().__init__()
        self.setWindowTitle("UXsim result viewer")
        self.W = W
        self.tmax = tmax
        self.dt = dt
        self.playing = False
        self.curve_direction = 1
        self.show_names = True
        self.show_vehicles = False

        central_widget = QWidget()
        layout = QVBoxLayout()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

        slider_layout = QHBoxLayout()
        self.t_slider = QSlider(Qt.Horizontal)
        self.t_slider.setMinimum(0)
        self.t_slider.setMaximum(tmax - 1)
        self.t_slider.setValue(0)
        self.t_slider.valueChanged.connect(self.update_graph)
        slider_layout.addWidget(self.t_slider)

        self.t_label = QLabel("0/{}".format(tmax))
        slider_layout.addWidget(self.t_label)

        layout.addLayout(slider_layout)

        button_layout = QHBoxLayout()
        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self.play_animation)
        button_layout.addWidget(self.play_button)

        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_animation)
        self.stop_button.setEnabled(False)
        button_layout.addWidget(self.stop_button)

        layout.addLayout(button_layout)

        self.graph_widget = GraphWidget(nodes, edges, vehicle_list)
        layout.addWidget(self.graph_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_t)
        
        menu_bar = self.menuBar()
        
        # menu_file = menu_bar.addMenu("File")
        # acrion_save_world = menu_file.addAction("Save World")
        # acrion_save_world.triggered.connect(lambda: self.save_world())

        menu_data = menu_bar.addMenu("Data")
        action_basic_stats = menu_data.addAction("Basic Statistics")
        action_basic_stats.triggered.connect(lambda: self.show_dataframe("Basic", self.W.analyzer.basic_to_pandas()))
        action_basic_stats = menu_data.addAction("Link Statistics")
        action_basic_stats.triggered.connect(lambda: self.show_dataframe("Link", self.W.analyzer.link_to_pandas()))
        action_basic_stats = menu_data.addAction("OD Demand Statistics")
        action_basic_stats.triggered.connect(lambda: self.show_dataframe("OD Demand", self.W.analyzer.od_to_pandas()))
        action_basic_stats = menu_data.addAction("Vehicle Trip Statistics")
        action_basic_stats.triggered.connect(lambda: self.show_dataframe("Vehicle Trip", self.W.analyzer.vehicle_trip_to_pandas()))
        action_basic_stats = menu_data.addAction("Vehicle Detailed Statistics")
        action_basic_stats.triggered.connect(lambda: self.show_dataframe("Vehicle", self.W.analyzer.vehicles_to_pandas()))
        
        menu_settings = menu_bar.addMenu("Settings")
        option_curve_direction = menu_settings.addMenu("Link Curve Direction")
        curve_right_action = option_curve_direction.addAction("Right-handed")
        curve_right_action.triggered.connect(lambda: self.set_curve_direction(1))
        curve_left_action = option_curve_direction.addAction("Left-handed")
        curve_left_action.triggered.connect(lambda: self.set_curve_direction(-1))
        
        option_display = menu_settings.addMenu("Display")
        show_names_action = option_display.addAction("Show Names")
        show_names_action.setCheckable(True)
        show_names_action.setChecked(True)
        show_names_action.triggered.connect(self.toggle_show_names)

        menu_Vehicle = menu_bar.addMenu("Vehicle Analysis")
        # show_vehicles_action = menu_Vehicle.addAction("Show Vehicle")
        # show_vehicles_action.setCheckable(True)
        # show_vehicles_action.setChecked(False)
        # show_vehicles_action.triggered.connect(self.toggle_show_vehicles)
        action_show_vehicle = menu_Vehicle.addAction("Highlight Vehicle by ID")
        action_show_vehicle.triggered.connect(self.show_vehicle_by_id)

        menu_Animation = menu_bar.addMenu("Export Results")
        action_csv = menu_Animation.addAction("Export Results to CSV files")
        action_csv.triggered.connect(lambda: self.W.analyzer.output_data())
        action_network_anim_detailed0 = menu_Animation.addAction("Export Network Animation (link-level)")
        action_network_anim_detailed0.triggered.connect(lambda: self.W.analyzer.network_anim(detailed=0))
        action_network_anim_detailed1 = menu_Animation.addAction("Export Network Animation (link segment-level)")
        action_network_anim_detailed1.triggered.connect(lambda: self.W.analyzer.network_anim(detailed=1))
        action_network_anim_fancy = menu_Animation.addAction("Export Network Animation (vehicle-level)")
        action_network_anim_fancy.triggered.connect(lambda: self.W.analyzer.network_fancy())

        self.update_graph()

    def show_dataframe(self, title, df):
        viewer = DataFrameViewer(df, title, self)
        viewer.show()

    def save_world(self, default_filename='untitled.pkl_dill'):
        #TODO: do something about "maximum recursion depth exceeded in comparison" error
        import dill as pickle
        filename, _ = QFileDialog.getSaveFileName(None, 'Save the world', default_filename, 'Pickle (by Dill package) Files (*.pkl_dill);;All Files (*)')
        
        if filename:
            try:
                with open(filename, 'wb') as file:
                    pickle.dump(self.W, file)
                print(f'World saved successfully: {filename}')
            except Exception as e:
                print(f'Error saving object: {str(e)}')

    def update_graph(self):
        t = self.t_slider.value()
        edie_dt = self.graph_widget.edges[0].Link.edie_dt
        self.graph_widget.set_t(t)
        self.t_label.setText("{}/{}".format(t*edie_dt, self.tmax*edie_dt))

    def play_animation(self):
        if not self.playing:
            self.playing = True
            self.play_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.timer.start(100)

    def stop_animation(self):
        if self.playing:
            self.playing = False
            self.play_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.timer.stop()

    def update_t(self):
        t = self.t_slider.value()
        t = (t + 1) % self.tmax
        self.t_slider.setValue(t)
        
    def set_curve_direction(self, direction):
        self.curve_direction = direction
        self.graph_widget.set_curve_direction(direction)
        
    def toggle_show_names(self):
        self.show_names = not self.show_names
        self.graph_widget.set_show_names(self.show_names)

    def toggle_show_vehicles(self):
        self.show_vehicles = not self.show_vehicles
        self.graph_widget.set_show_vehicles(self.show_vehicles)
    
    def show_vehicle_by_id(self):
        vehicle_id, ok = QInputDialog.getText(self, "Highlight Vehicle", "<b>Enter Vehicle ID</b><br>Note that fast vehicles will be plotted as multiple dots in the animation.")
        if ok and vehicle_id:
            self.vehicle_id = vehicle_id
            if vehicle_id not in self.W.VEHICLES:
                QMessageBox.warning(self, "Vehicle ID not found", "The specified Vehicle ID was not found.")
                return
            veh = self.W.VEHICLES[vehicle_id]
            self.graph_widget.vehicle_list = [(int(veh.log_t[i]/self.dt), veh.log_link[i].name, veh.log_x[i]/veh.log_link[i].length) for i in range(len(veh.log_t)) if veh.log_link[i] != -1]
            print(veh, self.graph_widget.vehicle_list)

            self.graph_widget.set_vehice_items()

            self.graph_widget.set_show_vehicles(True)


class PandasModel(QAbstractTableModel):
    def __init__(self, data):
        super(PandasModel, self).__init__()
        self._data = data

    def rowCount(self, parent=None):
        return self._data.shape[0]

    def columnCount(self, parent=None):
        return self._data.shape[1]

    def data(self, index, role=Qt.DisplayRole):
        if index.isValid() and role == Qt.DisplayRole:
            return str(self._data.iloc[index.row(), index.column()])
        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return str(self._data.columns[section])
            elif orientation == Qt.Vertical:
                return str(self._data.index[section])
        return None


class DataFrameViewer(QDialog):
    def __init__(self, data, title, parent=None):
        super(DataFrameViewer, self).__init__(parent)
        self.setWindowTitle(title)
        self.setLayout(QVBoxLayout())
        self.model = PandasModel(data)
        self.view = QTableView()
        self.view.setModel(self.model)
        self.layout().addWidget(self.view)
        
        self.resize(1200, 600)


def launch_World_viewer(W, return_app_window=False):
    """
    Launch the interactive viewer for the simulation result of the given World object.

    Parameters
    ----------
    W : World
        The World object to visualize.
    return_app_window : bool, optional
        If True, this function returns the QApplication and MainWindow objects. Default is False.
    """

    print("Launching the interactive viewer for the simulation result (Do NOT close this terminal!)...")

    W.show_mode = 1
    W.save_mode = 1
    W.analyzer.compute_edie_state()
    tmax = W.LINKS[0].q_mat.shape[0]
    nodes = [[n.name, n.x, n.y, n] for n in W.NODES]
    minx = 0
    maxx = 0
    miny = 0
    maxy = 0
    for i,n in enumerate(nodes):
        if i == 0:
            minx = n[1]
            maxx = n[1]
            miny = n[2]
            maxy = n[2]
        else:
            minx = min(minx, n[1])
            maxx = max(maxx, n[1])
            miny = min(miny, n[2])
            maxy = max(maxy, n[2])
    xysize = 800
    xybuffer = 20
    if maxx-minx > maxy-miny:
        for node in nodes:
            node[1] = (node[1] - minx) / (maxx - minx) * (xysize - xybuffer*2) + xybuffer
            node[2] = (maxy - node[2]) / (maxx - minx) * (xysize - xybuffer*2) + xybuffer
    else:
        for node in nodes:
            node[1] = (node[1] - minx) / (maxy - miny) * (xysize - xybuffer*2) + xybuffer
            node[2] = (maxy - node[2]) / (maxy - miny) * (xysize - xybuffer*2) + xybuffer

    edges = [[l.name, l.start_node.name, l.end_node.name, l.k_mat, l] for l in W.LINKS]
    dt = W.LINKS[0].edie_dt

    app = QApplication(sys.argv)
    window = MainWindow(W, nodes, edges, None, tmax, dt)
    window.show()
    if return_app_window:
        return app, window
    sys.exit(app.exec_())
