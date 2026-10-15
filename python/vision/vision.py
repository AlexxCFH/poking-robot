# Pipeline de vision para localizar globos por color.
# resize -> gaussian blur -> HSV -> inRange -> mascara binaria -> contornos -> momentos -> cx -> error
from dataclasses import dataclass, field
from typing import Optional

import cv2 as cv
import numpy as np

# tamano al que se reduce cada frame antes de procesarlo (ancho, alto)
PROCESS_SIZE = (320, 180)  # 16:9, igual que la Logitech (1280x720)
BLUR_KERNEL = (5, 5)
# area minima (en px del frame reducido) para considerar un contorno un globo
MIN_AREA = 300
# filtros de forma: un globo es compacto y no mucho mas ancho que alto; las lineas y marcas del
# suelo (muy anchas y finas) y las manchas irregulares se descartan
MIN_ASPECT = 0.5  # alto / ancho minimo del bounding box
MIN_FILL = 0.3  # area del contorno / area del bounding box minima
MIN_SOLIDITY = 0.5  # area del contorno / area de su casco convexo minima (descarta manchas irregulares)
# region de interes: solo se busca en esta fraccion superior del frame. Los globos estan sobre
# la pared del campo y el suelo cercano da falsos positivos (marcas, lineas, puntos naranjas)
ROI_BOTTOM = 0.65
# CLAHE sobre el canal V para que cambie menos el resultado con la iluminacion
CLAHE = cv.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))

# kernel de erosion (elipse pequena) para quitar manchas pequenas de la mascara
ERODE_KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
# kernel de cierre para unir los trozos del globo que parten los reflejos blancos
CLOSE_KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))

# rangos HSV por color (OpenCV: H 0-179, S 0-255, V 0-255); cada color es una lista de (lower, upper)
# calibrados con las fotos de data/ tomadas desde el robot. Los globos rojo y verde salen muy
# claros (sobreexpuestos, S baja), por eso la saturacion minima es baja. El suelo es beige
# (H~22), las zonas amarillas H~25 y los cuadros del suelo rojo/rosa H~176 / ~140.
# contador de pinchazos: se cuenta un globo cuando desaparece estando cerca y centrado
POP_MIN_AREA = 1500  # area (px del frame reducido) a partir de la que el globo esta "cerca"
POP_MAX_ERROR = 60  # |error| maximo (px) para considerar que el robot lo tiene de frente
POP_LOST_FRAMES = 5  # frames seguidos sin ver el globo para darlo por pinchado
MAX_BALLOONS = 3  # globos por equipo

COLOR_RANGES = {
    "red": [((0, 45, 100), (13, 255, 255)), ((172, 150, 100), (179, 255, 255))],
    "green": [((44, 40, 200), (80, 255, 255))],
    "blue": [((85, 100, 60), (115, 255, 255))],
}


@dataclass
class Detection:
    """Resultado de procesar un frame. cx, cy y error son None si no hay globo."""
    found: bool
    cx: Optional[int] = None
    cy: Optional[int] = None
    area: float = 0.0
    error: Optional[float] = None  # image_width/2 - cx (positivo = globo a la izquierda)
    contour: Optional[np.ndarray] = field(default=None, repr=False)  # contorno del globo, para dibujarlo


class ColorSegmenter:
    """Pasos de segmentacion: resize, blur, HSV, rango de color y mascara binaria."""

    def __init__(self, color, size=PROCESS_SIZE, ranges=None):
        self.size = size
        self.ranges = ranges if ranges is not None else COLOR_RANGES
        self.set_color(color)

    def set_color(self, color):
        if color not in self.ranges:
            raise ValueError(f"Color '{color}' no soportado. Opciones: {list(self.ranges)}")
        self.color = color

    def resize(self, frame):
        return cv.resize(frame, self.size, interpolation=cv.INTER_AREA)

    def to_hsv(self, frame):
        blur = cv.GaussianBlur(frame, BLUR_KERNEL, 0)
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
        hsv[:, :, 2] = CLAHE.apply(hsv[:, :, 2])
        return hsv

    def mask(self, hsv):
        binary = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.ranges[self.color]:
            binary = cv.bitwise_or(
                binary, cv.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            )
        # cierre: une los trozos de un mismo globo; despues se rellenan los huecos de los contornos
        binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, CLOSE_KERNEL)
        contours, _ = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(binary, contours, -1, 255, cv.FILLED)
        binary = cv.erode(binary, ERODE_KERNEL)
        binary[int(binary.shape[0] * ROI_BOTTOM):, :] = 0  # fuera de la region de interes
        return binary

    def segment(self, frame):
        """Devuelve (frame_reducido, mascara_binaria)."""
        small = self.resize(frame)
        return small, self.mask(self.to_hsv(small))


class BalloonDetector:
    """Localiza el globo mas grande de un color y calcula el error horizontal respecto al centro."""

    def __init__(self, color, min_area=MIN_AREA, size=PROCESS_SIZE):
        self.segmenter = ColorSegmenter(color, size)
        self.min_area = min_area

    def set_color(self, color):
        self.segmenter.set_color(color)

    def is_balloon(self, contour):
        """Filtra por area y forma para descartar manchas, lineas y marcas del suelo."""
        area = cv.contourArea(contour)
        if area < self.min_area:
            return False
        _, _, w, h = cv.boundingRect(contour)
        hull_area = cv.contourArea(cv.convexHull(contour))
        return (h / w >= MIN_ASPECT and area / (w * h) >= MIN_FILL
                and area / hull_area >= MIN_SOLIDITY)

    def largest_contour(self, mask):
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        valid = [c for c in contours if self.is_balloon(c)]
        return max(valid, key=cv.contourArea) if valid else None

    @staticmethod
    def centroid(contour):
        """Centroide a partir de los momentos: cx = m10/m00, cy = m01/m00."""
        m = cv.moments(contour)
        if m["m00"] == 0:
            return None
        return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])

    def detect(self, frame):
        """Procesa un frame BGR. Devuelve (Detection, frame_reducido, mascara)."""
        small, mask = self.segmenter.segment(frame)
        contour = self.largest_contour(mask)
        if contour is None:
            return Detection(False), small, mask
        center = self.centroid(contour)
        if center is None:
            return Detection(False), small, mask
        cx, cy = center
        width = small.shape[1]
        error = width / 2 - cx
        return Detection(True, cx, cy, cv.contourArea(contour), error, contour), small, mask

    @staticmethod
    def draw(small, det, popped=None):
        """Dibuja el centroide y la linea central sobre el frame reducido (para depurar)."""
        out = small.copy()
        h, w = out.shape[:2]
        cv.line(out, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        if det.found:
            cv.drawContours(out, [det.contour], -1, (255, 100, 0), 2)
            cv.circle(out, (det.cx, det.cy), 6, (0, 0, 255), -1)
            cv.putText(out, f"error: {det.error:.1f}", (5, 20),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if popped is not None:
            cv.putText(out, f"pinchados: {popped}", (5, h - 8),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return out


def configure_camera(cam, exposure=None, width=1280, height=720):
    """Desactiva exposicion y balance de blancos automaticos para que los globos no salgan lavados.

    Los valores de CAP_PROP_AUTO_EXPOSURE y CAP_PROP_EXPOSURE dependen del backend (V4L2 en
    Linux/placa, DirectShow/MSMF en Windows): 0.25 = manual en V4L2 y 0.75 = automatico; en
    DirectShow el valor de exposicion suele ser negativo (log2 de segundos, p. ej. -6).
    Si exposure es None no se toca el valor, solo se pasa a manual. Devuelve los valores que
    la camara informa tras aplicarlos, para comprobar que se aceptaron.
    """
    cam.set(cv.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cam.set(cv.CAP_PROP_AUTO_WB, 0)
    if exposure is not None:
        cam.set(cv.CAP_PROP_EXPOSURE, exposure)
    return {
        "auto_exposure": cam.get(cv.CAP_PROP_AUTO_EXPOSURE),
        "exposure": cam.get(cv.CAP_PROP_EXPOSURE),
        "auto_wb": cam.get(cv.CAP_PROP_AUTO_WB),
        "size": (cam.get(cv.CAP_PROP_FRAME_WIDTH), cam.get(cv.CAP_PROP_FRAME_HEIGHT)),
    }


class PopCounter:
    """Cuenta globos pinchados.

    Un globo se da por pinchado cuando, tras verlo cerca (area grande) y de frente (error
    pequeno) en el ultimo frame, deja de detectarse durante POP_LOST_FRAMES frames seguidos.
    Si desaparece estando lejos o descentrado (salio de plano al girar) no se cuenta.
    """

    def __init__(self, min_area=POP_MIN_AREA, max_error=POP_MAX_ERROR,
                 lost_frames=POP_LOST_FRAMES, max_count=MAX_BALLOONS):
        self.min_area = min_area
        self.max_error = max_error
        self.lost_frames = lost_frames
        self.max_count = max_count
        self.count = 0
        self._armed = False  # ultimo frame con globo: cerca y de frente
        self._lost = 0

    @property
    def done(self):
        return self.count >= self.max_count

    def reset(self):
        self.count = 0
        self._armed = False
        self._lost = 0

    def update(self, det):
        """Procesa la deteccion de un frame. Devuelve True si en este frame se cuenta un pinchazo."""
        if self.done:
            return False
        if det.found:
            self._lost = 0
            self._armed = det.area >= self.min_area and abs(det.error) <= self.max_error
            return False
        if not self._armed:
            return False
        self._lost += 1
        if self._lost < self.lost_frames:
            return False
        self.count += 1
        self._armed = False
        self._lost = 0
        return True


if __name__ == "__main__":
    # prueba rapida: python vision.py [color] [imagen_o_carpeta]
    # sin ruta usa la webcam; con ruta recorre las imagenes (cualquier tecla = siguiente, q = salir)
    import glob
    import os
    import sys

    detector = BalloonDetector(sys.argv[1] if len(sys.argv) > 1 else "red")
    if len(sys.argv) > 2 and sys.argv[2]:
        path = sys.argv[2]
        files = sorted(glob.glob(os.path.join(path, "*.jpg"))) if os.path.isdir(path) else [path]
        for f in files:
            frame = cv.imread(f)
            if frame is None:
                print("No se pudo leer", f)
                continue
            det, small, mask = detector.detect(frame)
            print(os.path.basename(f), det)
            cv.imshow("deteccion", BalloonDetector.draw(small, det))
            cv.imshow("mascara", mask)
            if cv.waitKey(0) & 0xFF == ord("q"):
                break
        cv.destroyAllWindows()
        sys.exit()

    counter = PopCounter()
    cam = cv.VideoCapture(0)
    # python vision.py red "" -6  -> tercer argumento opcional: exposicion manual
    print("camara:", configure_camera(cam, float(sys.argv[3]) if len(sys.argv) > 3 else None))
    while cam.isOpened():
        ok, frame = cam.read()
        if not ok:
            break
        det, small, mask = detector.detect(frame)
        if counter.update(det):
            print("globo pinchado! total:", counter.count)
        cv.imshow("deteccion", BalloonDetector.draw(small, det, counter.count))
        cv.imshow("mascara", mask)
        if cv.waitKey(1) & 0xFF == ord("q"):
            break
    cam.release()
    cv.destroyAllWindows()
