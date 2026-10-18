#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Int32
import time

# ====================================================================
# CONFIGURAZIONE WAYPOINT (Coordinate esatte estratte da RViz)
# ====================================================================

WAYPOINT_IIWA2 = {"x": -4.8, "y": 2.0, "yaw_z": 0.0, "yaw_w": 1.0}
WAYPOINT_IIWA1 = {"x": -2.0, "y": -2.0, "yaw_z": 0.0, "yaw_w": 1.0}

MARKER_ID_IIWA2 = 655
MARKER_ID_IIWA1 = 201

DOCKING_TIMEOUT_SEC = 600.0
PICK_TIMEOUT_SEC = 600.0


class TaskCoordinator(Node):

    def __init__(self):
        super().__init__('task_coordinator')

        # ------------------------------------------------------------
        # Nav2
        # ------------------------------------------------------------
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ------------------------------------------------------------
        # Docking ArUco
        # ------------------------------------------------------------
        self.dock_command_pub = self.create_publisher(Int32, '/rover/dock_command', 10)
        self.dock_status = 'idle'
        self.dock_status_sub = self.create_subscription(
            String, '/rover/dock_status', self.dock_status_callback, 10
        )

        # ------------------------------------------------------------
        # Stato pick di iiwa1 / iiwa2
        # ------------------------------------------------------------
        self.iiwa1_status = 'idle'
        self.iiwa2_status = 'idle'

        self.iiwa1_status_sub = self.create_subscription(
            String, '/iiwa1/pick_status', self.iiwa1_status_callback, 10
        )
        self.iiwa2_status_sub = self.create_subscription(
            String, '/iiwa2/pick_status', self.iiwa2_status_callback, 10
        )

        # ------------------------------------------------------------
        # Innesco Missione da Tastiera
        # ------------------------------------------------------------
        self.tool_request_sub = self.create_subscription(
            String, '/mission/tool_request', self.start_mission_callback, 10
        )
        
        self.mission_in_progress = False
        self.pending_tool = None

        self.get_logger().info('Task Coordinator pronto. In attesa del comando da terminale...')

    # ==================================================================
    # CALLBACKS DI STATO E AVVIO
    # ==================================================================

    def dock_status_callback(self, msg: String):
        self.dock_status = msg.data

    def iiwa1_status_callback(self, msg: String):
        self.iiwa1_status = msg.data

    def iiwa2_status_callback(self, msg: String):
        self.iiwa2_status = msg.data
        
    def start_mission_callback(self, msg: String):
        if self.mission_in_progress or self.pending_tool is not None:
            self.get_logger().warn("Missione già in corso, ignoro il nuovo comando.")
            return
            
        # Non lanciamo la missione qui, ci limitiamo a salvare il comando
        # per evitare il blocco del thread di ROS2!
        self.pending_tool = msg.data.strip().lower()

    # ==================================================================
    # NAV2 - VAI A UN WAYPOINT GROSSOLANO
    # ==================================================================

    def navigate_to(self, waypoint: dict) -> bool:
        self.get_logger().info(
            f'[NAV2] Partenza verso X={waypoint["x"]}, Y={waypoint["y"]} ...'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        goal_msg.pose.pose.orientation.z = waypoint['yaw_z']
        goal_msg.pose.pose.orientation.w = waypoint['yaw_w']

        self.nav_to_pose_client.wait_for_server()

        send_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('[NAV2] Destinazione rifiutata!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # Abbiamo rimosso la cancellazione del goal finita per evitare deadlock
        time.sleep(0.5)

        self.get_logger().info('[NAV2] Waypoint raggiunto, passo il controllo ad ArUco.')
        return True

    # ==================================================================
    # DOCKING ARUCO - RIFINITURA CENTIMETRICA E SVEGLIA NODO
    # ==================================================================

    def dock_to_marker(self, marker_id: int) -> bool:
        self.get_logger().info(f'[DOCK] Avvio docking fine su marker ID {marker_id}...')

        # Resetto lo stato nel caso ci fosse un "docked" di una missione precedente
        self.dock_status = 'idle'

        # ---> QUESTO È IL COMANDO CHE SVEGLIA IL NODO ARUCO <---
        msg = Int32()
        msg.data = marker_id
        self.dock_command_pub.publish(msg)

        start_time = time.time()

        while rclpy.ok():
            # Questo spin farà aggiornare lo stato di self.dock_status
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.dock_status == 'docked':
                self.get_logger().info(f'[DOCK] Docking completato su marker {marker_id}.')
                return True

            if time.time() - start_time > DOCKING_TIMEOUT_SEC:
                self.get_logger().error(
                    f'[DOCK] Timeout ({DOCKING_TIMEOUT_SEC}s) in attesa del docking su marker {marker_id}.'
                )
                return False

        return False

    # ==================================================================
    # ATTESA STATO PICK (generica, riusata per iiwa1 e iiwa2)
    # ==================================================================

    def wait_for_pick_done(self, get_status_fn, label: str) -> bool:
        self.get_logger().info(f'[{label}] In attesa del completamento...')

        start_time = time.time()
        seen_busy = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            status = get_status_fn()

            if status == 'busy':
                seen_busy = True

            if seen_busy and status == 'done':
                self.get_logger().info(f'[{label}] Completato con successo.')
                return True

            if seen_busy and status == 'error':
                self.get_logger().error(f'[{label}] Il nodo ha segnalato un errore.')
                return False

            if time.time() - start_time > PICK_TIMEOUT_SEC:
                self.get_logger().error(
                    f'[{label}] Timeout ({PICK_TIMEOUT_SEC}s) in attesa del completamento.'
                )
                return False

        return False

    # ==================================================================
    # MISSIONE COMPLETA
    # ==================================================================

    def run_mission(self, tool: str) -> bool:
        self.get_logger().info('====================================================')
        self.get_logger().info(f'INIZIO MISSIONE: trasporto "{tool}" da iiwa2 a iiwa1')
        self.get_logger().info('====================================================')

        # --------------------------------------------------------------
        # FASE 1 - Rover verso iiwa2
        # --------------------------------------------------------------
        if not self.navigate_to(WAYPOINT_IIWA2):
            return False

        # Inizia il docking e "sveglia" il nodo ArUco pubblicando l'ID
        if not self.dock_to_marker(MARKER_ID_IIWA2):
            return False

        # --------------------------------------------------------------
        # FASE 2 - iiwa2 fa pick (innescato dal docking stesso)
        # --------------------------------------------------------------
        self.get_logger().info('[IIWA2] In attesa che il pick parta e finisca...')

        if not self.wait_for_pick_done(lambda: self.iiwa2_status, 'IIWA2'):
            return False

        # --------------------------------------------------------------
        # FASE 3 - Rover verso iiwa1
        # --------------------------------------------------------------
        if not self.navigate_to(WAYPOINT_IIWA1):
            return False

        # Inizia il docking per il secondo tavolo
        if not self.dock_to_marker(MARKER_ID_IIWA1):
            return False

        # --------------------------------------------------------------
        # FASE 4 - iiwa1 fa place (innescato dal docking stesso)
        # --------------------------------------------------------------
        self.get_logger().info('[IIWA1] In attesa che il place parta e finisca...')

        if not self.wait_for_pick_done(lambda: self.iiwa1_status, 'IIWA1'):
            return False

        self.get_logger().info('====================================================')
        self.get_logger().info('MISSIONE COMPLETATA CON SUCCESSO!')
        self.get_logger().info('====================================================')

        return True


def main(args=None):
    rclpy.init(args=args)
    coordinator = TaskCoordinator()

    try:
        # Questo ciclo principale sostituisce l'uso rigido della callback,
        # prevenendo definitivamente il blocco del processo (deadlock).
        while rclpy.ok():
            rclpy.spin_once(coordinator, timeout_sec=0.1)

            # Se c'è un comando in sospeso, lo elabora
            if coordinator.pending_tool is not None:
                tool = coordinator.pending_tool
                coordinator.pending_tool = None
                
                coordinator.mission_in_progress = True
                coordinator.run_mission(tool)
                coordinator.mission_in_progress = False

    except KeyboardInterrupt:
        pass

    coordinator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
