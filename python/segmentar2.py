"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool, String
from proyecto_abp.finiteStateMachine import STATE_TOPIC, STATES, TRANSITION_TOPIC, TRANSITIONS,GOAL_TOPIC, POSSIBLE_GOALS, FREQUENCY 
"""
import cv_bridge
import cv2
import numpy as np

CAMERA_TOPIC = '/camera/image_raw'  # Topic por defecto para la cámara (puede ser modificado dinámicamente)
ERROR_TOPIC = '/visual_error' # float con componente X del centro de masas de la puerta detectada
HALLWAY_TOPIC = '/hallway' # bool indicando si se ha detectado puerta y trampilla (True) o no (False)
SHOW_CAMERA_FEED = False # for debug purposes
class Recon:
    """
    ## Clase a bajo nivel ##
    Encargada de todos los algoritmos de visión y segmentación (OpenCV).
    """
    def __init__(self):
        # Rango de color morado en HSV
        self.color = {
            "purple": {"lower": np.array([125, 50, 50]), "upper": np.array([155, 255, 255])},
            "orange": {"lower": np.array([0, 100, 40]), "upper": np.array([25, 255, 255])},
            "green": {"lower": np.array([40, 50, 50]), "upper": np.array([80, 255, 255])},
            "yellow": {"lower": np.array([20, 100, 100]), "upper": np.array([30, 255, 255])},
            "red": {"lower": np.array([0, 150, 50]), "upper": np.array([10, 255, 255])},
            "blue": {"lower": np.array([90, 50, 50]), "upper": np.array([130, 255, 255])}
        }
        self.frame = None
        self.there_is_hallway = False
        self.trampilla_detectada = False # flag para activar transicion de estado
        self.error = None
        self.goal = None # direccion objetivo en grados para el controlador basado en laser
        self.goal_identified = False # flag indicando si el objetivo ha sido identificado

        # Umbrales configurables para la geometría de la puerta
        self.umbral_y_diff = 80        # Diferencia máxima en Y para considerar que son las dos columnas de la puerta
        self.umbral_dist_max = 300     # Distancia máxima desde el centro de masas global al centro de cada columna

    def get_error(self):
        return self.error

    def get_goal(self):
        return self.goal

    def get_hallway_status(self):
        return self.there_is_hallway
    
    def get_goal_identified(self):
        #print(f"Goal identificado?: {self.goal_identified} - Error actual: {self.error}")
        return self.goal_identified
    
    def is_already_in_hallway(self):
        return not (self.trampilla_detectada or self.there_is_hallway)


    def set_frame(self, frame):
        """Almacena y redimensiona el frame actual para reducir coste computacional."""
        if frame is None:
            self.frame = None
            return
        # Reducir la imagen a la mitad para aligerar el procesamiento
        self.frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)

    def detect_color(self, color_name):
        """Aplica los filtros de OpenCV y devuelve la máscara segmentada."""
        if self.frame is None:
            return None
        
        # 1. Aplicar Gaussian Blur para reducir el ruido
        # 1. Aplicar Gaussian Blur para reducir el ruido (kernel reducido para mayor rendimiento)
        blurred = cv2.GaussianBlur(self.frame, (5, 5), 0)
        
        # 2. Cambiar al espacio de color HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # 3. Umbralización para obtener la máscara binaria
        mask = cv2.inRange(hsv, self.color[color_name]["lower"], self.color[color_name]["upper"])
        
        return mask
    
    def detect_hallway_entrance(self, mask):
        """Lógica de procesamiento de la imagen segmentada para detectar puerta y trampilla."""
        if mask is None:
            return None
            
        # Convertimos la máscara a BGR para poder dibujar encima a color
        output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # --- 1. DETECCIÓN DE LA PUERTA MEDIANTE CONTORNOS ---
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        contornos_validos = []
        centroides = []
        
        # Filtrar ruido y calcular centros de masas
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 1000:  # Filtrar manchas pequeñas
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    contornos_validos.append(c)
                    centroides.append((cx, cy))

        puerta_detectada = False
        self.trampilla_detectada = False
        centro_puerta = None

        # Kernel de Prewitt para detectar bordes horizontales (gradiente en Y)
        prewitt_y = np.array([[ 1,  1,  1],
                              [ 0,  0,  0],
                              [-1, -1, -1]], dtype=np.float32)

        #print(f"Numero de contornos detectados: {len(contornos)} - Contornos válidos: {len(contornos_validos)}")
        if len(contornos_validos) == 1:
            # Caso A: Un único contorno cerrado (Puede ser la U completa o un falso positivo de columna)
            c = contornos_validos[0]
            x, y, w, h = cv2.boundingRect(c)
            
            # Extraemos la región superior (20% superior) para comprobar si existe el travesaño
            roi_top_y_start = y
            roi_top_y_end = y + int(h * 0.2)
            
            # Dibujamos la ROI de comprobación del travesaño en cian
            cv2.rectangle(output_img, (x, roi_top_y_start), (x + w, roi_top_y_end), (255, 255, 0), 2)
            
            # Aplicamos Prewitt sobre esa zona superior
            grad_y_top = cv2.filter2D(mask[roi_top_y_start:roi_top_y_end, x:x+w].astype(np.float32), -1, prewitt_y)
            suma_gradiente_top = np.sum(np.abs(grad_y_top))
            
            # Umbral de travesaño: 50,000 es robusto para evitar columnas sueltas (ajustable según distancia/resolución)
            if suma_gradiente_top > 50000:
                puerta_detectada = True
                centro_puerta = centroides[0]
            
        elif len(contornos_validos) >= 2:
            # Caso B: Dos columnas y puede haber o no trampilla
            # ordenar centroides segun componente Y (priorizar columnas)
            centroides = sorted(centroides, key=lambda c: c[1])
            c1, c2 = centroides[0], centroides[1]
            
            # dibujar en amarillo los dos centros de masas detectados
            cv2.circle(output_img, c1, 10, (0, 255, 255), -1)
            cv2.circle(output_img, c2, 10, (0, 255, 255), -1)  

            diff_y = abs(c1[1] - c2[1])
            
            # Centro de masas conjunto de las dos columnas
            cx_sistema = int((c1[0] + c2[0]) / 2)
            cy_sistema = int((c1[1] + c2[1]) / 2)
            
            # Distancia del centro del sistema a los centros individuales
            dist1 = np.sqrt((cx_sistema - c1[0])**2 + (cy_sistema - c1[1])**2)
            dist2 = np.sqrt((cx_sistema - c2[0])**2 + (cy_sistema - c2[1])**2)
            
            # Validar que los centros verticales son invariantes y la distancia no supera el umbral
            vertical_center = diff_y < self.umbral_y_diff
            close_threshold = dist1 < self.umbral_dist_max and dist2 < self.umbral_dist_max
            """
            if not vertical_center:
                print(f"Falso positivo descartado (Diferencia Y demasiado grande: {diff_y}px)")
            if not close_threshold:
                print(f"Falso positivo descartado (Centros demasiado alejados: dist1={dist1:.2f}px, dist2={dist2:.2f}px)")
            """
            if vertical_center and close_threshold:
                puerta_detectada = True
                centro_puerta = (cx_sistema, cy_sistema)

        # --- 2. DETECCIÓN DE TRAMPILLA, LEYENDA Y DIBUJADO GRAFICO ---
        
        # Añadir leyenda permanente en la esquina superior izquierda
        cv2.rectangle(output_img, (10, 10), (220, 105), (0, 0, 0), -1)
        cv2.rectangle(output_img, (10, 10), (220, 105), (255, 255, 255), 1) # Borde blanco
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Leyenda 1: door_center
        cv2.circle(output_img, (30, 35), 8, (0, 0, 255), -1)
        cv2.putText(output_img, 'door_center', (50, 40), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        # Leyenda 2: roi hatch
        cv2.rectangle(output_img, (22, 55), (38, 70), (0, 255, 0), 2)
        cv2.putText(output_img, 'roi hatch', (50, 68), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        # Leyenda 3: roi top bar
        cv2.rectangle(output_img, (22, 80), (38, 95), (255, 255, 0), 2)
        cv2.putText(output_img, 'roi top bar', (50, 93), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        # Caso cuando el robot esta cerca de la puerta y puede haber trampilla
        posible_trampilla = (len(contornos_validos) >= 2) 

        if puerta_detectada or posible_trampilla:
            # Colorear el centro de masas en la imagen (círculo rojo)
            cv2.circle(output_img, centro_puerta, 15, (0, 0, 255), -1)

            # Obtener el Bounding Box global de la puerta detectada
            puntos_todos = np.vstack(contornos_validos)
            x, y, w, h = cv2.boundingRect(puntos_todos)
            
            # Extraer la región de interés (ROI): el 20% inferior del recuadro
            roi_y_start = y + int(h * 0.8)
            roi_y_end = y + h
            
            # Dibujar el rectángulo de la ROI de la trampilla en verde
            cv2.rectangle(output_img, (x, roi_y_start), (x + w, roi_y_end), (0, 255, 0), 2)
            
            # Aplicar el filtro de Prewitt sobre la ROI de la máscara binaria original
            grad_y_bottom = cv2.filter2D(mask[roi_y_start:roi_y_end, x:x+w].astype(np.float32), -1, prewitt_y)
            suma_gradiente_bottom = np.sum(np.abs(grad_y_bottom))
            
            # Detección final de la trampilla (umbral calibrable)
            if suma_gradiente_bottom > 250000:
                self.trampilla_detectada = True

        # --- 3. DIBUJAR ESTADO DE LA DETECCIÓN EN PANTALLA ---
        _, anchura = output_img.shape[:2]
        
        if puerta_detectada:
            #print("Puerta detectada.")
            # Texto verde indicando puerta detectada
            cv2.putText(output_img, 'PUERTA DETECTADA', (anchura - 280, 40), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        if self.trampilla_detectada:
            # Texto amarillo indicando la trampilla debajo del anterior
            cv2.putText(output_img, '+ TRAMPILLA DETECTADA', (anchura - 280, 75), font, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        if not puerta_detectada and not self.trampilla_detectada:
            # Texto rojo si no hay nada detectado
            cv2.putText(output_img, 'BUSCANDO...', (anchura - 180, 40), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        # actualizar el estado del pasillo (hallway) y el error 
        self.there_is_hallway = (puerta_detectada and self.trampilla_detectada)
        self.error = None  # valor por defecto si no se detecta puerta
        
        # relajar caso de que haya trampilla pero este muy cerca de la puerta
        if (puerta_detectada or self.trampilla_detectada) and centro_puerta is not None:
            img_width = output_img.shape[1]  # ancho de la imagen
            self.error = float(centro_puerta[0] - img_width / 2)  # componente X del error (diferencia con el centro de la imagen)


        return output_img

    def detect_target(self, goal: str):
        """Lógica de procesamiento para detectar objetivos y devolver imagen con anotaciones."""
        if goal not in POSSIBLE_GOALS:
            #print("El goal no se mapea bien")
            self.goal_identified = False
            self.error = None
            return self.frame.copy() if self.frame is not None else None
        
        mask = self.detect_color(goal)
        if self.frame is None:
            self.goal_identified = False
            self.error = None
            return None

        output_img = self.frame.copy()
        self.goal_identified = False

        if mask is None or np.all(mask == 0):
            self.error = None
            return output_img

        # Detectar contornos del objetivo
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filtrar contornos por área para eliminar ruido
        contornos_validos = [c for c in contornos if cv2.contourArea(c) > 500]
        cv2.drawContours(output_img, contornos_validos, -1, (255, 100, 0), 2)
                
        if len(contornos_validos) > 0:
            # Ordenar por mayor superficie para obtener el objetivo más cercano
            target = max(contornos_validos, key=cv2.contourArea)
            
            img_width = self.frame.shape[1]

            # Hallar bordes del objetivo (bounding box) y dibujarlo en verde
            x, y, w, h = cv2.boundingRect(target)
            cv2.rectangle(output_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Dibujar el centroide del objetivo en rojo
            M = cv2.moments(target)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(output_img, (cx, cy), 7, (0, 0, 255), -1)
                
                # Calcular error en componente X (diferencia con el centro de la imagen)
                self.error = float(cx - img_width / 2)
                self.goal_identified = True
            else:
                self.error = None
        else:
            self.error = None
            
        #print(f"Objetivos detectados: {len(contornos_validos)} - Goal identificado: {self.goal_identified}")
        # Añadir texto informativo a la imagen
        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.goal_identified:
            cv2.putText(output_img, f'Target Identified: {goal}', (10, 30), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(output_img, f'Error X: {self.error:.2f} px', (10, 60), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(output_img, f'Searching for: {goal}', (10, 30), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        return output_img

        
        
class CameraProcessor(Node):
    """
    ## Clase a alto nivel ##
    Administra el nodo y la suscripción a los topics de ROS2.
    """
    def __init__(self):
        super().__init__('camera_processor')
        
        # Obtener el namespace dinámicamente
        self.robot_id = self.get_namespace()
        if self.robot_id == '/':
            self.robot_id = '/robot_0'  # Default si no hay namespace
        
        self.recon = Recon() # A CameraProcessor uses a Recon para el procesamiento de imagen
        self.bridge = cv_bridge.CvBridge()
        self.image_subscription = None
        # Variable para almacenar el último frame procesado de forma segura
        self.latest_mask = None
        self.result = None
        self.dynamic_camera_feed = SHOW_CAMERA_FEED
        self.fsm_st = None
        self.current_goal = None
        self.target_approach_sent = False


        # Empleamos el setter en el constructor con el topic por defecto
        # Intentar suscribirse con reintentos
        try:
            self.setCameraTopic(self.robot_id + CAMERA_TOPIC)
        except Exception as e:
            self.get_logger().error(f'Error crítico en suscripción a cámara: {e}')
            raise
        
        # subscriptor a la fsm
        self.fsm_st = STATES[0]  # Inicializar con estado por defecto (WANDER)
        self.create_subscription(String, self.robot_id + STATE_TOPIC, self.fsm_callback, 10)
        self.get_logger().info(f'Suscrito a FSM state: {self.robot_id + STATE_TOPIC}')
        
        # subscriptor al goal
        self.create_subscription(String, self.robot_id + GOAL_TOPIC, self.goal_callback, 10)
        self.get_logger().info(f'Suscrito a goal topic: {self.robot_id + GOAL_TOPIC}')


        # publishers para los topics de error y hallway
        self.error_publisher_ = self.create_publisher(Float32, self.robot_id + ERROR_TOPIC, 10)
        self.hallway_publisher_ = self.create_publisher(Bool, self.robot_id + HALLWAY_TOPIC, 10)
        self.fsm_transition_publisher_ = self.create_publisher(String, self.robot_id + TRANSITION_TOPIC, 10)
        #self.processed_image_publisher_ = self.create_publisher(Image, self.robot_id + '/processed_image', 10)
        self.goal_reached_publisher_ = self.create_publisher(Bool, self.robot_id + '/goal_reached', 10)
        
        
        # CREAMOS UN TIMER PARA LA INTERFAZ GRÁFICA Y PUBLICACIÓN
        timer_period = 1.0 / FREQUENCY  # segundos
        self.display_timer = self.create_timer(timer_period, self.display_callback)

        if self.dynamic_camera_feed:
            # Si está habilitado, creamos la ventana de OpenCV
            self.window_name = "Procesamiento de Puerta/Trampilla"
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('Nodo CameraProcessor inicializado y listo.')

    def fsm_callback(self, msg: String):
        """Callback para recibir el estado actual de la FSM."""
        if msg is not None and msg.data in STATES:
            # Si el estado cambia, reseteamos los flags de transición para que puedan volver a enviarse
            if self.fsm_st != msg.data:
                self.target_approach_sent = False
                self.target_approach_counter = 0
            self.fsm_st = msg.data
    
    def goal_callback(self, msg: String):
        """Callback para recibir el objetivo actual."""
        if msg is not None:
            self.current_goal = msg.data
   
    def setCameraTopic(self, topic: str):
        """
        Setter para el topic de la cámara. 
        Crea (o recrea) la suscripción al nuevo topic.
        """
        self.camera_topic = topic
        
        if self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
            
        self.image_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Suscrito a la cámara en el topic: "{self.camera_topic}"')

    def image_callback(self, msg: Image):
        """Se ejecuta cada vez que llega una imagen. Solo procesa, NO dibuja."""
        try:
            # Validar que el estado FSM esté inicializado
            if self.fsm_st is None:
                self.get_logger().warn('FSM state no está disponible aún')
                return
            
            # Convertir mensaje a OpenCV y procesar
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.recon.set_frame(cv_image)
            
            # Computar la lógica y obtener la imagen final con el dibujado
            if self.fsm_st in [STATES[0], STATES[1]]:  # WANDER, APPROACH_DOOR
                purple_mask = self.recon.detect_color("purple")
                self.result = self.recon.detect_hallway_entrance(purple_mask)
            
            elif self.fsm_st in [STATES[2], STATES[3]]:  # NAVIGATING_HALLWAY, APPROACH_TARGET
                if self.current_goal is not None:
                    self.result = self.recon.detect_target(self.current_goal)
                else:
                    self.result = cv_image
            
            else:
                # Para otros estados o si es None, simplemente mostramos la imagen cruda
                self.result = cv_image

            # Publicar los resultados en los topics correspondientes
            error_msg = Float32()
            hallway_msg = Bool()
            trans_msg = String()

            error_msg.data = self.recon.get_error() if self.recon.get_error() is not None else 0.0
            hallway_msg.data = self.recon.get_hallway_status()            
            
            # Lanzar transiciones si se dan las condiciones 
            if self.fsm_st == STATES[1] and self.recon.is_already_in_hallway(): # Envia transicion de estado a la FSM
                trans_msg.data = TRANSITIONS[1]
                self.fsm_transition_publisher_.publish(trans_msg)
        
            # Publisher de realimentacion para el controlador PD (frecuencia de imagen)
            self.error_publisher_.publish(error_msg)
            self.hallway_publisher_.publish(hallway_msg)
            
            # Transición robusta a APPROACH_TARGET
            if self.fsm_st == STATES[2] and not self.target_approach_sent:
                if self.recon.get_goal_identified():
                    self.get_logger().info(f'Goal "{self.current_goal}" identificado.')
                    trans_msg.data = TRANSITIONS[2] # TARGET_APPROACH
                    self.fsm_transition_publisher_.publish(trans_msg)
               

        except Exception as e:
            self.get_logger().error(f'Error en image_callback: {e}')

    def display_callback(self):
        """Publica y muestra la imagen procesada en una ventana si SHOW_CAMERA_FEED es True."""
        if self.result is not None and self.dynamic_camera_feed:
            try:
                # Publicar la imagen procesada para RViz - Debug purposes
                #processed_img_msg = self.bridge.cv2_to_imgmsg(self.result, "bgr8")
                #self.processed_image_publisher_.publish(processed_img_msg)

                # Mostrar en ventana de OpenCV
                cv2.imshow(self.window_name, self.result)
                cv2.waitKey(1)
            except cv_bridge.CvBridgeError as e:
                self.get_logger().error(f'Error al convertir/publicar imagen: {e}')

    def goal_publish_callback(self):
        """Se ejecuta a baja frecuencia (1 Hz) para publicar el goal calculado."""
        if self.fsm_st == STATES[2]:  # Solo publicar goal en estado NAVIGATING_HALLWAY
            goal_msg = Float32()
            goal_msg.data = self.last_goal_value
            self.goal_publisher_.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = CameraProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Interrupción detectada. Cerrando nodo...')
    finally:
        # Limpieza de ventanas de OpenCV
        cv2.destroyAllWindows()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""
if __name__ == '__main__':
    main()
"""