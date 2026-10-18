#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
import time

class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        
        # 1. Crea il client per inviare le coordinate a Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 2. Crea i publisher per comandare le prese degli strumenti
        self.tool_pubs = {}
        for arm in ['iiwa1', 'iiwa2']:
            for tool in ['scalpel', 'scissors', 'forceps']:
                for action in ['attach', 'detach']:
                    topic_name = f'/{arm}/picker/{tool}/{action}'
                    self.tool_pubs[topic_name] = self.create_publisher(Empty, topic_name, 10)

    def send_goal(self, x, y, yaw_z=0.0, yaw_w=1.0):
        """Invia una coordinata a Nav2 e restituisce il future."""
        self.get_logger().info(f'Partenza verso X: {x}, Y: {y} ...')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Orientamento (Quaternione base)
        goal_msg.pose.pose.orientation.z = yaw_z
        goal_msg.pose.pose.orientation.w = yaw_w

        self.nav_to_pose_client.wait_for_server()
        return self.nav_to_pose_client.send_goal_async(goal_msg)

    def trigger_tool(self, arm, tool, action):
        """Pubblica un messaggio Empty sul topic corretto per gestire lo strumento."""
        topic_name = f'/{arm}/picker/{tool}/{action}'
        self.get_logger().info(f'Eseguo azione robotica: {topic_name}')
        msg = Empty()
        self.tool_pubs[topic_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    coordinator = TaskCoordinator()

    coordinator.get_logger().info('Inizializzazione missione sala operatoria...')
    
    # =====================================================================
    # FASE 1: Viaggio verso IIWA 2
    # =====================================================================
    future = coordinator.send_goal(-2.3, 4.91)
    rclpy.spin_until_future_complete(coordinator, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        coordinator.get_logger().error('Destinazione rifiutata da Nav2!')
        return

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(coordinator, result_future)
    coordinator.get_logger().info('Arrivato al Tavolo 2 (iiwa2)!')

    # =====================================================================
    # FASE 2: Interazione Strumenti (iiwa2 carica il bisturi sul rover)
    # =====================================================================
    time.sleep(2.0) # Aspetta 2 secondi per stabilizzare il rover
    coordinator.trigger_tool('iiwa2', 'scalpel', 'attach')
    coordinator.get_logger().info('Strumento caricato sul rover da iiwa2.')
    time.sleep(2.0)

    # =====================================================================
    # FASE 3: Viaggio verso IIWA 1
    # =====================================================================
    future = coordinator.send_goal(1.81, -0.0195)
    rclpy.spin_until_future_complete(coordinator, future)
    goal_handle = future.result()
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(coordinator, result_future)
    coordinator.get_logger().info('Arrivato al Tavolo 1 (iiwa1)!')

    # =====================================================================
    # FASE 4: Rilascio Strumento a iiwa1
    # =====================================================================
    time.sleep(2.0)
    coordinator.trigger_tool('iiwa2', 'scalpel', 'detach')
    # iiwa1 prende lo strumento dal rover
    coordinator.trigger_tool('iiwa1', 'scalpel', 'attach')
    
    coordinator.get_logger().info('MISSIONE COMPLETATA CON SUCCESSO! 🎉')

    # Chiusura pulita
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
