#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
 
# ================================================================
# Stati della FSM
# ================================================================
 
STATE_IDLE = -1       # In attesa di un comando dal coordinatore
STATE_SEARCH = 0
STATE_ALIGN = 1
STATE_APPROACH = 2
STATE_FINISHED = 3
 
 
# ================================================================
# Mappa: ID marker -> azione da inviare automaticamente al braccio
# quando il rover finisce di agganciarsi (STATE_FINISHED) a quel
# marker. Il nodo pubblica da solo, senza aspettare un coordinatore.
# ================================================================
 
DOCK_TRIGGER_MAP = {

    655: {'topic': 'iiwa2/pick_command'},
 
    201: {'topic': '/iiwa1/start_place_command'},
}
 
 
class ArucoDockingNode(Node):
    """
    Nodo di docking ArUco, comandabile via topic.
 
    - Ascolta /rover/dock_command (std_msgs/Int32): il coordinatore
      pubblica l'ID del marker da raggiungere. Il nodo interrompe
      qualsiasi docking in corso e ricomincia a cercare quel marker.
 
    - Pubblica /rover/dock_status (std_msgs/String) con uno di:
        "idle"       -> nessun comando ricevuto, o docking completato
                        e non ancora richiesto un nuovo target
        "searching"  -> sta ruotando cercando il marker
        "aligning"   -> marker visto, si sta allineando lateralmente
        "approaching"-> allineato, si sta avvicinando
        "docked"     -> fermo alla distanza target, FINITO
 
      Il coordinatore aspetta "docked" prima di procedere.
    """
 
    def __init__(self):
        super().__init__('aruco_docking_node')
 
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
 
        self.image_sub = self.create_subscription(
            Image, '/rover/camera/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/rover/camera/camera_info', self.info_callback, 10
        )
 
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/rover/dock_status', 10)
 
        # Il coordinatore (o un comando manuale) pubblica qui l'ID
        # del marker da raggiungere.
        self.dock_command_sub = self.create_subscription(
            Int32, '/rover/dock_command', self.dock_command_callback, 10
        )

        self.selected_tool = 'scalpel'  # default se non e' mai stato richiesto nulla
        self.tool_request_sub = self.create_subscription(
            String, '/mission/tool_request', self.tool_request_callback, 10
        )
 
        # Un publisher String per ciascun topic usato in DOCK_TRIGGER_MAP,
        # creato una sola volta all'avvio (evita di crearli a runtime).
        self.trigger_pubs = {}
        for marker_id, action in DOCK_TRIGGER_MAP.items():
            topic = action['topic']
            if topic not in self.trigger_pubs:
                self.trigger_pubs[topic] = self.create_publisher(String, topic, 10)
 
        # True quando il trigger per il docking CORRENTE e' gia' stato
        # inviato - evita di rimandare il comando ad ogni ciclo mentre
        # si resta fermi in STATE_FINISHED.
        self.trigger_sent = False
 
        self.marker_size = 0.1  # 10 cm - verifica che combaci con la dimensione reale nel world
        self.state = STATE_IDLE
        self.target_id = None
 
        self.marker_detected = False
        self.last_detection_time = 0.0
        self.latest_tvec = None
 
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        try:
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
        except AttributeError:
            self.detector = None
 
        # Pubblica lo stato periodicamente, cosi' il coordinatore lo
        # vede subito anche se si e' appena messo in ascolto.
        self.status_timer = self.create_timer(0.2, self.publish_status)
 
        self.get_logger().info('Nodo di docking ArUco pronto. In attesa di /rover/dock_command.')
 
    # ============================================================
    # COMANDO DAL COORDINATORE
    # ============================================================
 
    def dock_command_callback(self, msg: Int32):
        self.target_id = int(msg.data)
        self.state = STATE_SEARCH
        self.marker_detected = False
        self.trigger_sent = False  # nuovo docking, nuovo trigger da inviare
        self.get_logger().info(f'>>> Nuovo target ricevuto: marker ID {self.target_id}. Inizio ricerca.')
 
    def tool_request_callback(self, msg: String):
        self.selected_tool = msg.data.strip().lower()
        self.get_logger().info(f'>>> Strumento richiesto per questa missione: "{self.selected_tool}"')
 
    def send_arm_trigger(self):
        """
        Chiamata quando lo stato passa a FINISHED (docked). Pubblica
        il comando al braccio giusto UNA SOLA VOLTA per questo
        docking, usando il topic da DOCK_TRIGGER_MAP e lo strumento
        da self.selected_tool (impostato via /mission/tool_request).
        """
        if self.trigger_sent:
            return
 
        action = DOCK_TRIGGER_MAP.get(self.target_id)
        if action is None:
            self.get_logger().warn(
                f'Nessuna azione configurata in DOCK_TRIGGER_MAP per il marker {self.target_id}.'
            )
            self.trigger_sent = True  # evita di ritentare all'infinito su un ID non mappato
            return
 
        msg = String()
        msg.data = self.selected_tool
        self.trigger_pubs[action['topic']].publish(msg)
        self.trigger_sent = True
 
        self.get_logger().info(
            f">>> Rover fermo sul marker {self.target_id}: inviato '{self.selected_tool}' su {action['topic']}"
        )
 
    def publish_status(self):
        status_map = {
            STATE_IDLE: 'idle',
            STATE_SEARCH: 'searching',
            STATE_ALIGN: 'aligning',
            STATE_APPROACH: 'approaching',
            STATE_FINISHED: 'docked',
        }
        msg = String()
        msg.data = status_map.get(self.state, 'idle')
        self.status_pub.publish(msg)
 
    # ============================================================
    # CAMERA
    # ============================================================
 
    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
 
    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
 
        # Se non c'e' un target attivo, non facciamo nulla (risparmia
        # CPU e non muove il rover senza un comando esplicito).
        if self.state == STATE_IDLE or self.target_id is None:
            return
 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Errore CvBridge: {e}')
            return
 
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
 
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
 
        self.marker_detected = False
        if ids is not None and self.target_id in ids.flatten():
            index = np.where(ids.flatten() == self.target_id)[0][0]
            marker_corners = corners[index]
 
            try:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [marker_corners], self.marker_size, self.camera_matrix, self.dist_coeffs
                )
                self.latest_tvec = tvecs[0][0]
                self.marker_detected = True
                self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
 
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.drawFrameAxes(
                    cv_image, self.camera_matrix, self.dist_coeffs,
                    rvecs[0], tvecs[0], self.marker_size / 2
                )
            except Exception:
                pass
 
        self.update_fsm()
 
        cv2.imshow('Rover Docking View', cv_image)
        cv2.waitKey(1)
 
    # ============================================================
    # FSM
    # ============================================================
 
    def update_fsm(self):
        cmd = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_seen = current_time - self.last_detection_time
        is_visible = self.marker_detected and (time_since_seen < 0.5)
 
        if self.state == STATE_SEARCH:
            if is_visible:
                self.get_logger().info(f'>>> Marker {self.target_id} individuato! Allineamento.')
                self.state = STATE_ALIGN
            else:
                cmd.angular.z = 0.3
 
        elif self.state == STATE_ALIGN:
            if not is_visible:
                self.state = STATE_SEARCH
                self.vel_pub.publish(Twist())
                return
 
            x = self.latest_tvec[0]
            cmd.angular.z = float(np.clip(-2.5 * x, -0.8, 0.8))
 
            if abs(x) < 0.03:
                self.get_logger().info('>>> Allineato lateralmente. Avvicinamento.')
                self.state = STATE_APPROACH
 
        elif self.state == STATE_APPROACH:
            if not is_visible:
                self.get_logger().warn('Marker perso durante l\'avvicinamento, torno in ricerca.')
                self.state = STATE_SEARCH
                self.vel_pub.publish(Twist())
                return
 
            x = self.latest_tvec[0]
            z = self.latest_tvec[2]
 
            desired_z = 0.15
            error_z = z - desired_z
 
            linear_speed = 2 * error_z
            angular_speed = -3 * x
 
            cmd.linear.x = float(np.clip(linear_speed, -0.45, 0.45))
            cmd.angular.z = float(np.clip(angular_speed, -0.8, 0.8))
 
            if abs(error_z) < 0.05:
                self.get_logger().info(f'>>> DOCKING COMPLETATO su marker {self.target_id}. <<<')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.state = STATE_FINISHED
                self.send_arm_trigger()  # invia il comando al braccio SUBITO, non al prossimo ciclo
 
        elif self.state == STATE_FINISHED:
           # cmd.linear.x = 0.0
           # cmd.angular.z = 0.0
           return
 
        self.vel_pub.publish(cmd)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
 
 
if __name__ == '__main__':
    main()

