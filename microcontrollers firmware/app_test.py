import sys
import os
import json
import subprocess
import re
import threading
import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QSplitter, QPlainTextEdit, QMenuBar, QToolBar, QFileDialog,
                             QMessageBox, QComboBox, QLabel, QPushButton, QListWidget,
                             QDialog, QFormLayout, QSpinBox, QDialogButtonBox, QTabWidget,
                             QStatusBar, QStyle, QSizePolicy, QGraphicsView, QGraphicsScene,
                             QGraphicsPixmapItem, QGraphicsTextItem, QLineEdit, QGraphicsItem, QButtonGroup)
from PyQt6.QtCore import Qt, QProcess, QThread, pyqtSignal, QSize, QRect, QTimer
from PyQt6.QtGui import (QTextCharFormat, QColor, QFont, QPalette, QIcon, QSyntaxHighlighter,
                         QTextDocument, QPainter, QPixmap, QPen, QBrush)

# --- КОНФИГУРАЦИЯ ---
APP_NAME = "PyDuino IDE"
CONFIG_DIR = os.path.expanduser("~/.config/pyduino_ide")
CONFIG_PATH = os.path.join(CONFIG_DIR, "config.json")

# --- НАСТРОЙКИ ПО УМОЛЧАНИЮ ---
DEFAULT_CONFIG = {
    "theme": {
        "bg_r": 45, "bg_g": 45, "bg_b": 45,
        "fg_r": 220, "fg_g": 220, "fg_b": 220,
        "accent_r": 0, "accent_g": 153, "accent_b": 255
    },
    "last_board_fqbn": "",
    "last_port": "",
    "last_file_path": ""
}

# --- КОНФИГУРАЦИЯ ПЛАТЫ ---
# Смещение всей схемы по оси X (положительное значение - вправо, отрицательное - влево)
PIN_LAYOUT_OFFSET_X = 25  # <-- Измените это значение для сдвига

# --- ФУНКЦИИ ДЛЯ РАБОТЫ С КОНФИГУРАЦИЕЙ ---
def load_config():
    os.makedirs(CONFIG_DIR, exist_ok=True)
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, 'r') as f:
                return {**DEFAULT_CONFIG, **json.load(f)}
        except (json.JSONDecodeError, IOError):
            return DEFAULT_CONFIG
    return DEFAULT_CONFIG

def save_config(config):
    with open(CONFIG_PATH, 'w') as f:
        json.dump(config, f, indent=4)

# --- ПОДСВЕТКА СИНТАКСИСА ---
class ArduinoHighlighter(QSyntaxHighlighter):
    def __init__(self, document: QTextDocument):
        super().__init__(document)
        self._rules = []
        self._setup_formats()
        self._setup_rules()

    def _setup_formats(self):
        self.keyword_format = QTextCharFormat()
        self.keyword_format.setForeground(QColor(0, 153, 255)) # Accent color
        self.keyword_format.setFontWeight(QFont.Weight.Bold)

        self.type_format = QTextCharFormat()
        self.type_format.setForeground(QColor(255, 120, 0))

        self.function_format = QTextCharFormat()
        self.function_format.setForeground(QColor(120, 255, 120))

        self.string_format = QTextCharFormat()
        self.string_format.setForeground(QColor(255, 255, 120))

        self.comment_format = QTextCharFormat()
        self.comment_format.setForeground(QColor(150, 150, 150))
        self.comment_format.setFontItalic(True)

        self.number_format = QTextCharFormat()
        self.number_format.setForeground(QColor(255, 80, 255))

    def _setup_rules(self):
        keywords = [
            "break", "case", "class", "continue", "default", "do", "else", "enum", "for",
            "goto", "if", "return", "switch", "while", "struct", "typedef", "union",
            "const", "static", "volatile", "extern", "long", "signed", "unsigned",
            "true", "false", "null", "new", "delete", "public", "private", "protected"
        ]

        types = [
            "void", "boolean", "byte", "char", "int", "word", "long", "float", "double",
            "String", "array", "size_t", "uint8_t", "uint16_t", "uint32_t", "int8_t"
        ]

        functions = [ # Часто используемые функции Arduino
            "pinMode", "digitalWrite", "digitalRead", "analogRead", "analogWrite", "delay",
            "delayMicroseconds", "millis", "micros", "Serial.begin", "Serial.print", "Serial.println",
            "Serial.available", "Serial.read", "Serial.write", "setup", "loop"
        ]

        self._rules = []

        for word in keywords:
            pattern = re.compile(rf'\b{word}\b')
            self._rules.append((pattern, self.keyword_format))

        for word in types:
            pattern = re.compile(rf'\b{word}\b')
            self._rules.append((pattern, self.type_format))

        for word in functions:
            pattern = re.compile(rf'\b{word}\b')
            self._rules.append((pattern, self.function_format))

        self._rules.append((re.compile(r'//.*'), self.comment_format))
        self._rules.append((re.compile(r'/\*.*?\*/', re.DOTALL), self.comment_format))
        self._rules.append((re.compile(r'"[^"\\]*(\\.[^"\\]*)*"'), self.string_format))
        self._rules.append((re.compile(r"'[^'\\]*(\\.[^'\\]*)*'"), self.string_format))
        self._rules.append((re.compile(r'\b[0-9]+\b'), self.number_format))

    def highlightBlock(self, text: str):
        for pattern, format in self._rules:
            for match in pattern.finditer(text):
                self.setFormat(match.start(), match.end() - match.start(), format)

# --- РЕДАКТОР КОДА С НОМЕРАМИ СТРОК ---
class CodeEditor(QPlainTextEdit):
    def __init__(self):
        super().__init__()
        self._line_number_area = QWidget(self)
        self._line_number_area.setFixedWidth(50)
        self.blockCountChanged.connect(self._update_line_number_area_width)
        self.updateRequest.connect(self._update_line_number_area)
        self._update_line_number_area_width(0)
        self.setFont(QFont("Consolas, Monaco, monospace", 12))

    def _update_line_number_area_width(self, _):
        width = 10 + self.fontMetrics().horizontalAdvance('9') * len(str(self.blockCount()))
        self._line_number_area.setFixedWidth(width)

    def _update_line_number_area(self, rect, dy):
        if dy:
            self._line_number_area.scroll(0, dy)
        else:
            self._line_number_area.update(0, rect.y(), self._line_number_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self._update_line_number_area_width(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self._line_number_area.setGeometry(QRect(cr.left(), cr.top(), self._line_number_area.width(), cr.height()))

    def line_number_area_paint_event(self, event): # ИСПРАВЛЕНО: убрано подчеркивание, т.к. это не переопределение
        painter = QPainter(self._line_number_area)
        painter.fillRect(event.rect(), QColor(60, 60, 60))
        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = self.blockBoundingGeometry(block).translated(self.contentOffset()).top()
        bottom = top + self.blockBoundingRect(block).height()

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                number = str(block_number + 1)
                painter.setPen(QColor(180, 180, 180))
                painter.drawText(0, int(top), self._line_number_area.width() - 5, self.fontMetrics().height(),
                                 Qt.AlignmentFlag.AlignRight, number)
            block = block.next()
            top = bottom
            bottom = top + self.blockBoundingRect(block).height()
            block_number += 1

# --- ДИАЛОГ НАСТРОЕК ---
class SettingsDialog(QDialog):
    def __init__(self, parent=None, current_theme=None):
        super().__init__(parent)
        self.setWindowTitle("Настройки темы")
        self.setModal(True)
        self.current_theme = current_theme or DEFAULT_CONFIG["theme"]
        layout = QFormLayout(self)

        self.bg_r_spin = QSpinBox(); self.bg_r_spin.setRange(0, 255)
        self.bg_g_spin = QSpinBox(); self.bg_g_spin.setRange(0, 255)
        self.bg_b_spin = QSpinBox(); self.bg_b_spin.setRange(0, 255)
        layout.addRow("Цвет фона (R):", self.bg_r_spin)
        layout.addRow("Цвет фона (G):", self.bg_g_spin)
        layout.addRow("Цвет фона (B):", self.bg_b_spin)

        self.fg_r_spin = QSpinBox(); self.fg_r_spin.setRange(0, 255)
        self.fg_g_spin = QSpinBox(); self.fg_g_spin.setRange(0, 255)
        self.fg_b_spin = QSpinBox(); self.fg_b_spin.setRange(0, 255)
        layout.addRow("Цвет текста (R):", self.fg_r_spin)
        layout.addRow("Цвет текста (G):", self.fg_g_spin)
        layout.addRow("Цвет текста (B):", self.fg_b_spin)

        self.accent_r_spin = QSpinBox(); self.accent_r_spin.setRange(0, 255)
        self.accent_g_spin = QSpinBox(); self.accent_g_spin.setRange(0, 255)
        self.accent_b_spin = QSpinBox(); self.accent_b_spin.setRange(0, 255)
        layout.addRow("Цвет акцента (R):", self.accent_r_spin)
        layout.addRow("Цвет акцента (G):", self.accent_g_spin)
        layout.addRow("Цвет акцента (B):", self.accent_b_spin)

        self.buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        layout.addRow(self.buttons)

        self._load_current_theme()

    def _load_current_theme(self):
        self.bg_r_spin.setValue(self.current_theme.get("bg_r", 45))
        self.bg_g_spin.setValue(self.current_theme.get("bg_g", 45))
        self.bg_b_spin.setValue(self.current_theme.get("bg_b", 45))
        self.fg_r_spin.setValue(self.current_theme.get("fg_r", 220))
        self.fg_g_spin.setValue(self.current_theme.get("fg_g", 220))
        self.fg_b_spin.setValue(self.current_theme.get("fg_b", 220))
        self.accent_r_spin.setValue(self.current_theme.get("accent_r", 0))
        self.accent_g_spin.setValue(self.current_theme.get("accent_g", 153))
        self.accent_b_spin.setValue(self.current_theme.get("accent_b", 255))

    def get_theme(self):
        return {
            "bg_r": self.bg_r_spin.value(), "bg_g": self.bg_g_spin.value(), "bg_b": self.bg_b_spin.value(),
            "fg_r": self.fg_r_spin.value(), "fg_g": self.fg_g_spin.value(), "fg_b": self.fg_b_spin.value(),
            "accent_r": self.accent_r_spin.value(), "accent_g": self.accent_g_spin.value(), "accent_b": self.accent_b_spin.value()
        }

# --- ПОТОК ДЛЯ МОНИТОРА ПОРТА ---
class SerialMonitorThread(QThread):
    data_received = pyqtSignal(bytes)

    def __init__(self, port, baud=9600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self._is_running = True

    def run(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.1)
            while self._is_running:
                try:
                    if self.serial_conn.in_waiting > 0:
                        data = self.serial_conn.read(self.serial_conn.in_waiting)
                        if data:
                            self.data_received.emit(data)
                except serial.SerialException as e:
                    print(f"Ошибка чтения с порта: {e}")
                    break
        except serial.SerialException as e:
            print(f"Не удалось открыть порт {self.port}: {e}")
        finally:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()

    def stop(self):
        self._is_running = False
        if not self.wait(1000):
            self.terminate()

# --- ОТДЕЛЬНОЕ ОКНО ДЛЯ МОНИТОРА ПОРТА ---
class SerialMonitorWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Монитор порта")
        self.setGeometry(150, 150, 600, 400)
        self.serial_thread = None
        # Добавь это:
        self.autoscroll_enabled = True
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.monitor_display = QPlainTextEdit()
        self.monitor_display.setReadOnly(True)
        self.monitor_display.setFont(QFont("Consolas, Monaco, monospace", 10))
        layout.addWidget(self.monitor_display)

        # Добавляем кнопку автоскролла
        self.autoscroll_btn = QPushButton("Выключить AutoScroll (Space)")
        self.autoscroll_btn.clicked.connect(self.toggle_autoscroll)
        layout.addWidget(self.autoscroll_btn)

        # Добавляем кнопку очистки
        clear_btn = QPushButton("Очистить")
        clear_btn.clicked.connect(self.monitor_display.clear)
        layout.addWidget(clear_btn)

        # Установить фокус на виджет с выводом для клавиатурных событий
        self.monitor_display.setFocus()

    def start_monitoring(self, port, baud=9600):
        if self.serial_thread and self.serial_thread.isRunning():
            self.stop_monitoring()

        self.monitor_display.clear()
        self.statusBar().showMessage(f"Монитор порта открыт на {port}")
        self.serial_thread = SerialMonitorThread(port, baud)
        self.serial_thread.data_received.connect(self._on_serial_data)
        self.serial_thread.start()

    def stop_monitoring(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread = None
        self.statusBar().showMessage("Монитор порта закрыт")

    def _on_serial_data(self, data):
        try:
            text = data.decode('utf-8', errors='replace')
            text = text.replace('\r\n', '\n').replace('\r', '\n')
            self.monitor_display.insertPlainText(text)
            if self.autoscroll_enabled:  # Добавим проверку автоскролла
                self.monitor_display.verticalScrollBar().setValue(self.monitor_display.verticalScrollBar().maximum())
        except Exception as e:
            print(f"Error decoding serial  {e}")

    def toggle_autoscroll(self):
        self.autoscroll_enabled = not self.autoscroll_enabled
        if self.autoscroll_enabled:
            self.autoscroll_btn.setText("Выключить AutoScroll (Space)")
        else:
            self.autoscroll_btn.setText("Включить AutoScroll (Space)")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Space:
            self.toggle_autoscroll()
        super().keyPressEvent(event)

    def closeEvent(self, event):
        self.stop_monitoring()
        event.accept()

# --- ОКНО С ИЗОБРАЖЕНИЕМ ПЛАТЫ ---
class BoardViewWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent  # Ссылка на главное окно для доступа к редактору
        self.setWindowTitle("Схема платы")
        # Увеличиваем размеры окна, чтобы плата помещалась целиком
        self.setGeometry(200, 200, 600, 700) # Было: 500, 600
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Создаем BoardView и передаем ему главное окно
        self.board_view_widget = BoardView(parent=self.parent)
        layout.addWidget(self.board_view_widget)

# --- ВИДЖЕТ СХЕМЫ ПЛАТЫ ---
# --- ВИДЖЕТ СХЕМЫ ПЛАТЫ (ИСПРАВЛЕННАЯ ВЕРСИЯ) ---
class BoardView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        # Убираем эти строки, теперь они в главном окне
        # self.pin_definitions = {}
        # self.pin_modes = {}
        self.pin_mode_button_groups = {}
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.graphics_view = QGraphicsView()
        self.graphics_scene = QGraphicsScene()
        self.graphics_view.setScene(self.graphics_scene)
        self.update_code_btn = QPushButton("Обновить #define в коде")
        self.update_code_btn.clicked.connect(self.update_code)
        layout.addWidget(self.graphics_view)
        layout.addWidget(self.update_code_btn)
        self.load_board_image()
        self.add_pins()

    def load_board_image(self):
        if not self.parent or not hasattr(self.parent, 'config'):
            bg_color = (45, 45, 45)
            accent_color = (0, 153, 255)
        else:
            bg_color = self.parent.config["theme"]["bg_r"], self.parent.config["theme"]["bg_g"], self.parent.config["theme"]["bg_b"]
            accent_color = self.parent.config["theme"]["accent_r"], self.parent.config["theme"]["accent_g"], self.parent.config["theme"]["accent_b"]
        image_width = 550
        image_height = 550
        pixmap = QPixmap(image_width, image_height)
        pixmap.fill(QColor(*accent_color))
        self.board_image = QGraphicsPixmapItem(pixmap)
        self.graphics_scene.addItem(self.board_image)
        self.graphics_scene.setSceneRect(0, 0, image_width, image_height)
        self.graphics_view.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.graphics_view.setFixedSize(image_width, image_height)
        self.graphics_view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.graphics_view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    def add_pins(self):
        if not self.parent or not hasattr(self.parent, 'config'):
            fg_color = (220, 220, 220)
            bg_color = (45, 45, 45)
        else:
            fg_color = self.parent.config["theme"]["fg_r"], self.parent.config["theme"]["fg_g"], self.parent.config["theme"]["fg_b"]
            bg_color = self.parent.config["theme"]["bg_r"], self.parent.config["theme"]["bg_g"], self.parent.config["theme"]["bg_b"]
        digital_pins_with_pwm = ["D3", "D5", "D6", "D9", "D10", "D11"]
        digital_pins = []
        for i in range(14):
            pin_name = f"D{i}"
            if pin_name in digital_pins_with_pwm:
                pin_name = f"~{pin_name}"
            digital_pins.append(pin_name)
        analog_pins = [f"A{i}" for i in range(6)]
        analog_x = 25 + PIN_LAYOUT_OFFSET_X
        analog_start_y = 320
        for i, pin_name in enumerate(reversed(analog_pins)):
            y = analog_start_y + i * 35
            self.add_pin_item(pin_name, analog_x, y, fg_color, bg_color)
        digital_x = 250 + PIN_LAYOUT_OFFSET_X
        digital_start_y = 490
        for i, pin_name in enumerate(digital_pins):
            y = digital_start_y - i * 35
            self.add_pin_item(pin_name, digital_x, y, fg_color, bg_color)

    def on_pin_name_changed(self, pin_name, new_text):
        new_name = new_text.strip()
        clean_pin_name = pin_name.lstrip("~")
        if new_name:
            self.parent.pin_definitions[clean_pin_name] = new_name
        else:
            self.parent.pin_definitions.pop(clean_pin_name, None)

    def add_pin_item(self, pin_name, x, y, fg_color, bg_color):
        text_item = QGraphicsTextItem(pin_name)
        text_item.setDefaultTextColor(QColor(*fg_color))
        text_item.setPos(x, y)
        self.graphics_scene.addItem(text_item)

        input_field = QLineEdit()
        input_field.setPlaceholderText("Имя пина")
        input_field.setMaximumWidth(80)
        input_field.setStyleSheet(f"""
            QLineEdit {{
                background-color: rgba({bg_color[0]}, {bg_color[1]}, {bg_color[2]}, 150);
                color: rgb({fg_color[0]}, {fg_color[1]}, {fg_color[2]});
                border: 1px solid rgb({fg_color[0]}, {fg_color[1]}, {fg_color[2]});
                padding: 2px;
                border-radius: 3px;
            }}
        """)
        
        # --- Создание кнопок и их стилей ---
        btn_style = f"""
            QPushButton {{
                background-color: rgba({bg_color[0]}, {bg_color[1]}, {bg_color[2]}, 150);
                color: rgb({fg_color[0]}, {fg_color[1]}, {fg_color[2]});
                border: 1px solid rgb({fg_color[0]}, {fg_color[1]}, {fg_color[2]});
                padding: 1px;
                border-radius: 3px;
            }}
            QPushButton:checked {{
                background-color: rgb(0, 153, 255);
                color: rgb(255, 255, 255);
            }}
        """
        btn_o = QPushButton("O")
        btn_o.setFixedSize(25, 20)
        btn_o.setStyleSheet(btn_style)
        btn_o.setCheckable(True)
        btn_o.mode = "OUTPUT" # Сохраняем режим в самом объекте кнопки

        btn_i = QPushButton("I")
        btn_i.setFixedSize(25, 20)
        btn_i.setStyleSheet(btn_style)
        btn_i.setCheckable(True)
        btn_i.mode = "INPUT"

        btn_ip = QPushButton("IP")
        btn_ip.setFixedSize(35, 20)
        btn_ip.setStyleSheet(btn_style)
        btn_ip.setCheckable(True)
        btn_ip.mode = "INPUT_PULLUP"

        # --- НОВАЯ ЛОГИКА С QButtonGroup ---
        clean_pin_name = pin_name.lstrip("~")
        
        button_group = QButtonGroup(self)
        button_group.setExclusive(True) # Гарантирует, что только одна кнопка может быть нажата
        self.pin_mode_button_groups[clean_pin_name] = button_group
        
        button_group.addButton(btn_o)
        button_group.addButton(btn_i)
        button_group.addButton(btn_ip)
        
        # Подключаем сигнал от группы к нашему новому обработчику
        button_group.buttonClicked.connect(lambda button, pn=clean_pin_name: self.on_pin_mode_group_clicked(pn, button))
        # --- КОНЕЦ НОВОЙ ЛОГИКИ ---

        proxy_input = self.graphics_scene.addWidget(input_field)
        proxy_input.setPos(x + 50, y)

        proxy_btn_o = self.graphics_scene.addWidget(btn_o)
        proxy_btn_i = self.graphics_scene.addWidget(btn_i)
        proxy_btn_ip = self.graphics_scene.addWidget(btn_ip)

        btn_start_x = x + 120 + PIN_LAYOUT_OFFSET_X
        proxy_btn_o.setPos(btn_start_x, y)
        proxy_btn_i.setPos(btn_start_x + 25, y)
        proxy_btn_ip.setPos(btn_start_x + 50, y)

        input_field.textChanged.connect(lambda text, pn=pin_name: self.on_pin_name_changed(pn, text))
        clean_pin_name = pin_name.lstrip("~")
        
        # Загружаем имя пина
        if clean_pin_name in self.parent.pin_definitions:
            input_field.setText(self.parent.pin_definitions[clean_pin_name])
            
        # Загружаем режим пина
        if clean_pin_name in self.parent.pin_modes:
            mode = self.parent.pin_modes[clean_pin_name]
            if mode == "OUTPUT":
                btn_o.setChecked(True)
            elif mode == "INPUT":
                btn_i.setChecked(True)
            elif mode == "INPUT_PULLUP":
                btn_ip.setChecked(True)
        # -----------------------------------------

        # Сохраняем ссылки на элементы для дальнейшего доступа
        setattr(self, f"input_{pin_name}", input_field)
        setattr(self, f"input_{pin_name}", input_field)
        setattr(self, f"btn_o_{pin_name}", btn_o)
        setattr(self, f"btn_i_{pin_name}", btn_i)
        setattr(self, f"btn_ip_{pin_name}", btn_ip)

        # Начальное состояние
        btn_o.setChecked(False)
        btn_i.setChecked(False)
        btn_ip.setChecked(False)
        self.parent.pin_modes[clean_pin_name] = None

        if pin_name.startswith('A'):
            btn_o.setVisible(False)
            btn_ip.setVisible(False)
            btn_i.setVisible(True)

    def on_pin_mode_group_clicked(self, pin_name, clicked_button):
        new_mode = clicked_button.mode
        old_mode = self.parent.pin_modes.get(pin_name)

        if new_mode == old_mode:
            self.parent.pin_modes[pin_name] = None
            button_group = self.pin_mode_button_groups[pin_name]
            button_group.setExclusive(False)
            clicked_button.setChecked(False)
            button_group.setExclusive(True)
        else:
            self.parent.pin_modes[pin_name] = new_mode
    def update_code(self):
        if not self.parent or not hasattr(self.parent, 'editor'):
            print("Ошибка: Нет доступа к редактору кода.")
            return

        current_code = self.parent.editor.toPlainText()
        lines = current_code.split('\n')
        new_lines = []
        in_block_to_remove = False
        for line in lines:
            stripped_line = line.strip()
            if stripped_line.startswith("// --- AUTO-GENERATED"):
                in_block_to_remove = True
                continue
            if in_block_to_remove:
                if stripped_line.startswith("// --- END AUTO-GENERATED"):
                    in_block_to_remove = False
                continue
            new_lines.append(line)

        clean_code = '\n'.join(new_lines).strip()
        lines = clean_code.split('\n')

        define_block_lines = []
        pins_with_names = {pin: name for pin, name in self.parent.pin_definitions.items() if name.strip()}
        if pins_with_names:
            define_block_lines.append("// --- AUTO-GENERATED DEFINES ---")
            for pin, name in pins_with_names.items():
                clean_pin = pin.lstrip("~")
                define_block_lines.append(f"#define {name} {clean_pin}")
            define_block_lines.append("// --- END AUTO-GENERATED DEFINES ---")

        pinMode_block_lines = []
        pinMode_calls = []
        for pin, mode in self.parent.pin_modes.items():
            if mode is not None:
                pin_name_to_use = self.parent.pin_definitions.get(pin, pin)
                pinMode_calls.append(f"  pinMode({pin_name_to_use}, {mode});")

        if pinMode_calls:
            pinMode_block_lines.append("  // --- AUTO-GENERATED PINMODES ---")
            pinMode_block_lines.extend(pinMode_calls)
            pinMode_block_lines.append("  // --- END AUTO-GENERATED PINMODES ---")

        final_lines = []
        if define_block_lines:
            final_lines.extend(define_block_lines)
            final_lines.append("")

        in_setup = False
        setup_found_and_handled = False
        for line in lines:
            if "void setup()" in line and not setup_found_and_handled:
                in_setup = True
                final_lines.append(line)
                if '{' in line:
                    if pinMode_block_lines:
                        final_lines.extend(pinMode_block_lines)
                    setup_found_and_handled = True
                continue
            if in_setup and not setup_found_and_handled and '{' in line:
                final_lines.append(line)
                if pinMode_block_lines:
                    final_lines.extend(pinMode_block_lines)
                setup_found_and_handled = True
                continue
            final_lines.append(line)

        final_code = '\n'.join(final_lines)
        self.parent.editor.setPlainText(final_code)

# --- ГЛАВНОЕ ОКНО ПРИЛОЖЕНИЯ ---
class PyDuinoIDE(QMainWindow):
    def __init__(self):
        super().__init__()
        self.config = load_config()
        self.autoscroll_enabled = True
        self.current_file_path = self.config.get("last_file_path", "")
        self.serial_thread = None
        self.serial_monitor_window = None
        self.board_view_window = None
        self._uploading_after_compile = False

        # Добавляем хранение состояния пинов
        self.pin_definitions = self.config.get("pin_definitions", {})
        self.pin_modes = self.config.get("pin_modes", {})

        self.setWindowTitle(APP_NAME)
        self.setGeometry(100, 100, 1200, 800)

        self._setup_ui()
        self._apply_theme(self.config["theme"])
        self._detect_ports()

        # --- Таймер для отслеживания портов ---
        self.port_timer = QTimer(self)
        self.port_timer.timeout.connect(self._detect_ports)
        self.port_timer.start(2000)  # Проверять каждые 2 секунды

        if self.current_file_path and os.path.exists(self.current_file_path):
            self._open_file(self.current_file_path)

    def _close_serial_monitor(self):
        if self.serial_monitor_window and self.serial_monitor_window.isVisible():
            self.serial_monitor_window.close()

    def on_tab_changed(self, index):
        if index == self.monitor_tab_index:
            port = self.port_combo.currentData()
            if not port or self.port_combo.currentText() == "Порты не найдены":
                QMessageBox.warning(self, "Внимание", "Порт для монитора не выбран.")
                # Возвращаемся на вкладку "Вывод"
                self.output_tabs.setCurrentIndex(0)
                return

            if self.serial_monitor_window is None:
                self.serial_monitor_window = SerialMonitorWindow(self)

            if self.serial_monitor_window.isVisible():
                self.serial_monitor_window.close()
            else:
                self.serial_monitor_window.start_monitoring(port)
                self.serial_monitor_window.show()
                self.serial_monitor_window.raise_()
                self.serial_monitor_window.activateWindow()

    def _setup_ui(self):
        # --- Центральный виджет ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # --- Основной сплиттер (редактор и боковая панель) ---
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(main_splitter)

        # --- Левая панель (редактор + консоль вывода) ---
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_widget.setContentsMargins(0, 0, 0, 0)

        self.editor = CodeEditor()
        self.highlighter = ArduinoHighlighter(self.editor.document())
        left_layout.addWidget(self.editor)

        # --- Консоль вывода ---
        self.output_console = QPlainTextEdit()
        self.output_console.setReadOnly(True)
        self.output_console.setFont(QFont("Consolas, Monaco, monospace", 10))
        # Устанавливаем минимальный размер для консоли, чтобы избежать исчезновения
        self.output_console.setMinimumHeight(100)
        # Устанавливаем политику размера
        self.output_console.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_layout.addWidget(self.output_console)

        main_splitter.addWidget(left_widget)
        main_splitter.setSizes([800, 400])

        # --- Правая панель (инструменты) ---
        right_panel = QWidget()
        right_panel.setMaximumWidth(350)
        right_layout = QVBoxLayout(right_panel)

        # Выбор платы
        right_layout.addWidget(QLabel("Плата:"))
        self.board_combo = QComboBox()
        # Добавляем платы с их FQBN (Fully Qualified Board Name)
        self.board_combo.addItem("Arduino Uno", "arduino:avr:uno")
        self.board_combo.addItem("Arduino Nano", "arduino:avr:nano")
        self.board_combo.addItem("ESP32", "esp32:esp32:esp32")
        self.board_combo.addItem("ESP32-S3", "esp32:esp32:esp32s3")
        self.board_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        right_layout.addWidget(self.board_combo)

        # Выбор порта
        right_layout.addWidget(QLabel("Порт:"))
        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        right_layout.addWidget(self.port_combo)

        right_layout.addSpacing(20)

        # Кнопки действий
        self.compile_btn = QPushButton("Компилировать (Ctrl+R)")
        self.compile_btn.clicked.connect(self.compile_sketch)
        right_layout.addWidget(self.compile_btn)

        self.upload_btn = QPushButton("Загрузить (Ctrl+U)")
        self.upload_btn.clicked.connect(self.upload_sketch)
        right_layout.addWidget(self.upload_btn)

        # Кнопка монитора порта
        self.monitor_btn = QPushButton("Открыть монитор порта")
        self.monitor_btn.clicked.connect(self.open_serial_monitor)
        right_layout.addWidget(self.monitor_btn)

        # Кнопка схемы платы
        self.board_view_btn = QPushButton("Открыть схему платы")
        self.board_view_btn.clicked.connect(self.open_board_view)
        right_layout.addWidget(self.board_view_btn)

        right_layout.addStretch()

        main_splitter.addWidget(right_panel)

        # --- Меню и тулбар ---
        self._create_menu_bar()
        self._create_toolbar()

        # --- Статусбар ---
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Готово")

    def open_board_view(self):
        if self.board_view_window is None:
            self.board_view_window = BoardViewWindow(parent=self)
        
        self.board_view_window.show()
        self.board_view_window.raise_()
        self.board_view_window.activateWindow()

    def open_serial_monitor(self):
        port = self.port_combo.currentData()
        if not port or self.port_combo.currentText() == "Порты не найдены":
            QMessageBox.warning(self, "Внимание", "Порт для монитора не выбран.")
            return

        if self.serial_monitor_window is None or not self.serial_monitor_window.isVisible():
            self.serial_monitor_window = SerialMonitorWindow(self)
            self.serial_monitor_window.start_monitoring(port)
            self.serial_monitor_window.show()
            self.serial_monitor_window.raise_()
            self.serial_monitor_window.activateWindow()
            # Установить фокус после открытия окна
            self.serial_monitor_window.setFocus()
        else:
            self.serial_monitor_window.close()

    def _create_menu_bar(self):
        menubar = self.menuBar()
        # Файл
        file_menu = menubar.addMenu("Файл")
        new_action = file_menu.addAction("Новый", self.new_file)
        new_action.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_FileIcon))
        new_action.setShortcut("Ctrl+N")
        open_action = file_menu.addAction("Открыть...", self.open_file_dialog)
        open_action.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton))
        open_action.setShortcut("Ctrl+O")
        save_action = file_menu.addAction("Сохранить", self.save_file)
        save_action.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogSaveButton))
        save_action.setShortcut("Ctrl+S")
        save_as_action = file_menu.addAction("Сохранить как...", self.save_file_as)
        file_menu.addSeparator()
        exit_action = file_menu.addAction("Выход", self.close)

        # Инструменты
        tools_menu = menubar.addMenu("Инструменты")
        compile_action = tools_menu.addAction("Компилировать", self.compile_sketch)
        compile_action.setShortcut("Ctrl+R")
        upload_action = tools_menu.addAction("Загрузить", self.upload_sketch)
        upload_action.setShortcut("Ctrl+U")
        board_view_action = tools_menu.addAction("Схема платы", self.open_board_view)

        # Настройки
        settings_menu = menubar.addMenu("Настройки")
        theme_action = settings_menu.addAction("Цветовая тема...", self.open_settings_dialog)

    def _create_toolbar(self):
        toolbar = QToolBar("Главная")
        self.addToolBar(toolbar)
        toolbar.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)

        new_action = toolbar.addAction(self.style().standardIcon(QStyle.StandardPixmap.SP_FileIcon), "Новый")
        new_action.triggered.connect(self.new_file)
        open_action = toolbar.addAction(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogOpenButton), "Открыть")
        open_action.triggered.connect(self.open_file_dialog)
        save_action = toolbar.addAction(self.style().standardIcon(QStyle.StandardPixmap.SP_DialogSaveButton), "Сохранить")
        save_action.triggered.connect(self.save_file)

        toolbar.addSeparator()

        compile_action = toolbar.addAction(self.style().standardIcon(QStyle.StandardPixmap.SP_MediaPlay), "Компилировать")
        compile_action.triggered.connect(self.compile_sketch)
        upload_action = toolbar.addAction(self.style().standardIcon(QStyle.StandardPixmap.SP_ArrowUp), "Загрузить")
        upload_action.triggered.connect(self.upload_sketch)

        # Добавляем кнопку монитора на тулбар
        monitor_action = toolbar.addAction(self.style().standardIcon(QStyle.StandardPixmap.SP_ComputerIcon), "Монитор")
        monitor_action.triggered.connect(self.open_serial_monitor)

        # Добавляем кнопку схемы платы на тулбар
        board_view_action = toolbar.addAction(self.style().standardIcon(QStyle.StandardPixmap.SP_DesktopIcon), "Плата") # Используйте подходящую иконку
        board_view_action.triggered.connect(self.open_board_view)

    def _apply_theme(self, theme):
        palette = self.palette()
        bg_color = QColor(theme["bg_r"], theme["bg_g"], theme["bg_b"])
        fg_color = QColor(theme["fg_r"], theme["fg_g"], theme["fg_b"])
        accent_color = QColor(theme["accent_r"], theme["accent_g"], theme["accent_b"])

        palette.setColor(QPalette.ColorRole.Window, bg_color)
        palette.setColor(QPalette.ColorRole.WindowText, fg_color)
        palette.setColor(QPalette.ColorRole.Base, bg_color.darker(120))
        palette.setColor(QPalette.ColorRole.AlternateBase, bg_color.darker(130))
        palette.setColor(QPalette.ColorRole.ToolTipBase, fg_color)
        palette.setColor(QPalette.ColorRole.ToolTipText, fg_color)
        palette.setColor(QPalette.ColorRole.Text, fg_color)
        palette.setColor(QPalette.ColorRole.Button, bg_color.darker(110))
        palette.setColor(QPalette.ColorRole.ButtonText, fg_color)
        palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
        palette.setColor(QPalette.ColorRole.Link, accent_color)
        palette.setColor(QPalette.ColorRole.Highlight, accent_color)
        palette.setColor(QPalette.ColorRole.HighlightedText, Qt.GlobalColor.black)
        self.setPalette(palette)
        self.editor.setPalette(palette)
        self.output_console.setPalette(palette)

        # Стили для виджетов, которые не всегда подчиняются палитре
        style_sheet = f"""
            QMenuBar, QToolBar, QStatusBar {{
                background-color: rgb({theme['bg_r']}, {theme['bg_g']}, {theme['bg_b']});
                color: rgb({theme['fg_r']}, {theme['fg_g']}, {theme['fg_b']});
                border: none;
            }}
            QTabWidget::pane {{
                border: 1px solid rgb({theme['bg_r']-20}, {theme['bg_g']-20}, {theme['bg_b']-20});
                background-color: rgb({theme['bg_r']}, {theme['bg_g']}, {theme['bg_b']});
            }}
            QTabBar::tab {{
                background-color: rgb({theme['bg_r']+10}, {theme['bg_g']+10}, {theme['bg_b']+10});
                color: rgb({theme['fg_r']}, {theme['fg_g']}, {theme['fg_b']});
                padding: 8px;
                border: 1px solid rgb({theme['bg_r']-20}, {theme['bg_g']-20}, {theme['bg_b']-20});
            }}
            QTabBar::tab:selected {{
                background-color: rgb({theme['accent_r']}, {theme['accent_g']}, {theme['accent_b']});
            }}
            QComboBox, QPushButton {{
                background-color: rgb({theme['bg_r']+10}, {theme['bg_g']+10}, {theme['bg_b']+10});
                color: rgb({theme['fg_r']}, {theme['fg_g']}, {theme['fg_b']});
                border: 1px solid rgb({theme['bg_r']-20}, {theme['bg_g']-20}, {theme['bg_b']-20});
                padding: 5px;
                border-radius: 3px;
            }}
            QComboBox::drop-down, QPushButton::pressed {{
                background-color: rgb({theme['accent_r']}, {theme['accent_g']}, {theme['accent_b']});
            }}
            QComboBox QAbstractItemView {{
                background-color: rgb({theme['bg_r']}, {theme['bg_g']}, {theme['bg_b']});
                color: rgb({theme['fg_r']}, {theme['fg_g']}, {theme['fg_b']});
                selection-background-color: rgb({theme['accent_r']}, {theme['accent_g']}, {theme['accent_b']});
            }}
        """
        self.setStyleSheet(style_sheet)

        # Обновляем цвет акцента в подсветке синтаксиса
        self.highlighter.keyword_format.setForeground(accent_color)
        self.highlighter.rehighlight()

    # --- ЛОГИКА РАБОТЫ С ФАЙЛАМИ ---
    def new_file(self):
        sketch_template = """void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
"""
        self.editor.setPlainText(sketch_template)
        self.current_file_path = ""
        self.setWindowTitle(f"{APP_NAME} - Новый файл")

    def open_file_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Открыть скетч", "", "Arduino Files (*.ino *.c *.cpp *.h);;All Files (*)")
        if path:
            self._open_file(path)

    def _open_file(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                self.editor.setPlainText(f.read())
            self.current_file_path = path
            self.setWindowTitle(f"{APP_NAME} - {os.path.basename(path)}")
            self.status_bar.showMessage(f"Открыт: {path}")
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Не удалось открыть файл:\n{e}")

    def save_file(self):
        if not self.current_file_path:
            self.save_file_as()
            return
        self._save_to_path(self.current_file_path)

    def save_file_as(self):
        path, _ = QFileDialog.getSaveFileName(self, "Сохранить скетч", "", "Arduino Files (*.ino);;All Files (*)")
        if path:
            self._save_to_path(path)

    def _save_to_path(self, path):
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(self.editor.toPlainText())
            self.current_file_path = path
            self.setWindowTitle(f"{APP_NAME} - {os.path.basename(path)}")
            self.status_bar.showMessage(f"Сохранено: {path}")
        except Exception as e:
            QMessageBox.critical(self, "Ошибка", f"Не удалось сохранить файл:\n{e}")

    # --- ЛОГИКА РАБОТЫ С ARDUINO-CLI ---
        self.status_bar.showMessage("Порт выбран.")

    def _run_arduino_command(self, command_args, on_finish_callback):
        sketch_dir = self._get_sketch_folder()
        if not sketch_dir:
            return

        # Очищаем консоль только при загрузке
        if command_args[0] == "upload":
            self.output_console.clear()

        self.compile_btn.setEnabled(False)
        self.upload_btn.setEnabled(False)

        self.process = QProcess(self)
        full_command = ["arduino-cli"] + command_args + [sketch_dir]
        self.output_console.appendPlainText(f">>> {' '.join(full_command)}\n")
        self.status_bar.showMessage(f"Выполняется: {' '.join(command_args)}...")

        # Сохраняем колбэк, чтобы вызвать его позже
        self._on_finish_callback = on_finish_callback

        self.process.readyReadStandardOutput.connect(self._handle_process_output)
        self.process.readyReadStandardError.connect(self._handle_process_error)
        # Подключаемся к слоту, а не к лямбде
        self.process.finished.connect(self._on_process_finished)

        self.process.start(full_command[0], full_command[1:])

    def _on_process_finished(self, exit_code, exit_status):
        """Слот, который вызывается по завершении процесса."""
        self.compile_btn.setEnabled(True)
        self.upload_btn.setEnabled(True)
        # Вызываем нужный колбэк (компиляции или загрузки)
        if hasattr(self, '_on_finish_callback') and self._on_finish_callback:
            self._on_finish_callback(exit_code, exit_status)

    def _handle_process_output(self):
        data = self.process.readAllStandardOutput().data().decode('utf-8', errors='replace')
        # Фильтруем прогресс-бары
        lines = data.split('\n')
        filtered_lines = []
        for line in lines:
            line = line.strip()
            # Пропускаем строки с прогресс-баром
            if line:
                filtered_lines.append(line)
        if filtered_lines:
            filtered_data = '\n'.join(filtered_lines) + '\n'
            self.output_console.insertPlainText(filtered_data)
            if self.autoscroll_enabled:  # Добавим проверку автоскролла
                self.output_console.verticalScrollBar().setValue(self.output_console.verticalScrollBar().maximum())

    def _handle_process_error(self):
        data = self.process.readAllStandardError().data().decode('utf-8', errors='replace')
        # Фильтруем прогресс-бары и неинформативные avrdude сообщения
        lines = data.split('\n')
        filtered_lines = []
        info_lines = []  # Для сообщений avrdude, которые не являются ошибками
        for line in lines:
            line = line.strip()
            # Пропускаем строки с прогресс-баром
            if line:
                # Отдельно обрабатываем avrdude сообщения
                if ('avrdude:' in line and
                    ('AVR device initialized' in line or 'Device signature' in line or
                    'reading input file' in line or 'writing flash' in line or
                    'bytes of flash written' in line or 'done. Thank you' in line)):
                    info_lines.append(line)
                elif not ('avrdude:' in line or 'Firmware Version:' in line or 'Vtarget' in line or
                        'Varef' in line or 'Oscillator' in line or 'SCK period' in line or
                        'New upload port:' in line or 'Hardware Version:' in line):
                    filtered_lines.append(line)

        # Выводим реальные ошибки без префикса
        if filtered_lines:
            error_data = '\n'.join(filtered_lines) + '\n'
            self.output_console.appendPlainText(error_data)
            if self.autoscroll_enabled:
                self.output_console.verticalScrollBar().setValue(self.output_console.verticalScrollBar().maximum())

        # Выводим информационные avrdude сообщения без префикса
        if info_lines:
            info_data = '\n'.join(info_lines) + '\n'
            self.output_console.appendPlainText(info_data)
            if self.autoscroll_enabled:
                self.output_console.verticalScrollBar().setValue(self.output_console.verticalScrollBar().maximum())

    def _detect_ports(self):
        # Получаем текущий список портов из combo box
        current_ports_in_combo = {self.port_combo.itemData(i) for i in range(self.port_combo.count())}

        # Находим новые порты
        ports = serial.tools.list_ports.comports()
        new_ports = {port.device for port in ports if port.device.startswith("/dev/ttyACM") or port.device.startswith("/dev/ttyUSB")}

        # Сравниваем наборы портов
        if current_ports_in_combo == new_ports:
            return # Ничего не изменилось, выходим

        # Порты изменились, обновляем список
        currently_selected_port_data = self.port_combo.currentData()
        self.port_combo.blockSignals(True) # Блокируем сигналы, чтобы не вызывать лишних событий
        self.port_combo.clear()

        if not new_ports:
            self.port_combo.addItem("Порты не найдены")
            self.status_bar.showMessage("Порты ACM/USB не обнаружены.")
        else:
            sorted_ports = sorted(list(new_ports)) # Сортируем для порядка
            for port in sorted_ports:
                self.port_combo.addItem(port, port)

            # Восстанавливаем выбор: сначала пытаемся вернуть то, что было выбрано
            index_to_select = self.port_combo.findData(currently_selected_port_data)
            if index_to_select == -1: # Если старого порта нет, пробуем из конфига
                last_port = self.config.get("last_port")
                index_to_select = self.port_combo.findData(last_port)
            if index_to_select != -1:
                self.port_combo.setCurrentIndex(index_to_select)
            self.status_bar.showMessage("Список портов обновлен.")

        self.port_combo.blockSignals(False) # Возвращаем сигналы

    def _get_sketch_folder(self):
        if not self.current_file_path:
            QMessageBox.warning(self, "Внимание", "Сначала сохраните скетч.")
            return None
        # Arduino CLI требует папку с именем файла
        sketch_dir = os.path.dirname(self.current_file_path)
        if os.path.basename(self.current_file_path) != os.path.basename(sketch_dir) + ".ino":
             QMessageBox.warning(self, "Внимание", "Файл .ino должен находиться в папке с таким же именем.")
             return None
        return sketch_dir

    def compile_sketch(self):
        fqbn = self.board_combo.currentData()
        if not fqbn or self.board_combo.currentText() == "Платы не найдены":
            QMessageBox.critical(self, "Ошибка", "Плата не выбрана.")
            return

        command = ["compile", "--fqbn", fqbn]
        self._run_arduino_command(command, self._on_compile_finished)

    def _on_compile_finished(self, exit_code, exit_status):
        # Если компиляция была частью процесса загрузки
        if self._uploading_after_compile:
            self._uploading_after_compile = False # Сбрасываем флаг
            if exit_code == 0:
                # Компиляция успешна, запускаем ЗАГРУЗКУ
                self.status_bar.showMessage("Компиляция успешна. Начинаю загрузку...")
                fqbn = self.board_combo.currentData()
                port = self.port_combo.currentData()
                # Убираем --verbose, как в вашем рабочем скрипте
                upload_command = ["upload", "--fqbn", fqbn, "--port", port]
                self._run_arduino_command(upload_command, self._on_upload_finished)
            else:
                # Компиляция не удалась, прерываем процесс загрузки
                self.status_bar.showMessage("Ошибка компиляции. Загрузка отменена.")
                self.compile_btn.setEnabled(True)
                self.upload_btn.setEnabled(True)
        else:
            # Это была обычная компиляция, просто разблокируем кнопки
            self.compile_btn.setEnabled(True)
            self.upload_btn.setEnabled(True)
            if exit_code == 0:
                self.status_bar.showMessage("Компиляция успешна!")
            else:
                self.status_bar.showMessage("Ошибка компиляции.")

    def upload_sketch(self):
        # 1. Автоматически сохраняем файл
        if not self.current_file_path:
            QMessageBox.warning(self, "Внимание", "Сначала сохраните скетч.")
            return
        self.save_file()

        # 2. Автоматически закрываем монитор порта
        self._close_serial_monitor()

        fqbn = self.board_combo.currentData()
        port = self.port_combo.currentData()
        if not fqbn or self.board_combo.currentText() == "Платы не найдены":
            QMessageBox.critical(self, "Ошибка", "Плата не выбрана.")
            return
        if not port or self.port_combo.currentText() == "Порты не найдены":
            QMessageBox.critical(self, "Ошибка", "Порт не выбран.")
            return

        self._uploading_after_compile = True
        command = ["compile", "--fqbn", fqbn]
        self._run_arduino_command(command, self._on_compile_finished)

    def _on_upload_finished(self, exit_code, exit_status):
        self.compile_btn.setEnabled(True)
        self.upload_btn.setEnabled(True)
        if exit_code == 0:
            self.status_bar.showMessage("Загрузка успешна!")
        else:
            self.status_bar.showMessage("Ошибка загрузки.")

    # --- МОНИТОР ПОРТА ---
    def _on_serial_data(self, data):
        try:
            text = data.decode('utf-8', errors='replace')
            # Нормализуем переносы строк для корректного отображения
            text = text.replace('\r\n', '\n').replace('\r', '\n')
            self.serial_monitor.insertPlainText(text)
            if self.autoscroll_enabled:  # Добавим проверку автоскролла
                self.serial_monitor.verticalScrollBar().setValue(self.serial_monitor.verticalScrollBar().maximum())
        except Exception as e:
            print(f"Error decoding serial  {e}")

    # --- НАСТРОЙКИ И ЗАКРЫТИЕ ---
    def open_settings_dialog(self):
        dialog = SettingsDialog(self, self.config.get("theme"))
        if dialog.exec() == QDialog.DialogCode.Accepted:
            new_theme = dialog.get_theme()
            self.config["theme"] = new_theme
            self._apply_theme(new_theme)
            save_config(self.config)
            self.status_bar.showMessage("Тема обновлена.")

    def closeEvent(self, event):
        self._close_serial_monitor()
        
        # Сохраняем состояние пинов в конфиг
        self.config["pin_definitions"] = self.pin_definitions
        self.config["pin_modes"] = self.pin_modes
        
        self.config["last_board_fqbn"] = self.board_combo.currentData() or ""
        self.config["last_port"] = self.port_combo.currentData() or ""
        self.config["last_file_path"] = self.current_file_path
        save_config(self.config)
        event.accept()

# --- ЗАПУСК ПРИЛОЖЕНИЯ ---
if __name__ == '__main__':
    # Проверка наличия arduino-cli
    try:
        subprocess.run(["arduino-cli", "version"], check=True, capture_output=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        app = QApplication(sys.argv)
        QMessageBox.critical(None, "Ошибка запуска",
                            "Arduino CLI не найден!\n"
                            "Пожалуйста, установите его и добавьте в PATH.\n"
                            "Инструкции: https://arduino.github.io/arduino-cli/latest/installation/  ")
        sys.exit(1)

    app = QApplication(sys.argv)
    app.setStyle("Fusion") # Стиль для более современного вида
    window = PyDuinoIDE()
    window.show()
    sys.exit(app.exec())