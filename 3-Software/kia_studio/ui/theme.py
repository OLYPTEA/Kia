"""Kia Studio dark theme (QSS) — clean technical look (DJI/Unitree-style).

Cool neutral surfaces, a single restrained azure accent, monospaced data figures,
thin 1px borders, consistent 6px radii. The light 3D viewport reads as a focal lightbox.
Palette tokens:
  bg #0f1115 · surface #171a20 · sunken #12151a · border #262b33 · border-hi #313842
  text #e7eaef · muted #8a929c · faint #5b626c · accent #2d9bdb · ok #34d399 · danger #e5484d
"""

STYLESHEET = """
* { font-family: "Segoe UI", sans-serif; font-size: 13px; }

QMainWindow, QWidget { background: #0f1115; color: #e7eaef; }

/* ---- grouping ---- */
QGroupBox {
    border: 1px solid #222730; border-radius: 8px;
    margin-top: 16px; padding: 10px 10px 8px 10px; background: #14171d;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 12px; padding: 0 4px;
    color: #7f8893; font-size: 11px;
}

/* ---- telemetry / data figures ---- */
QLabel#tlmValue {
    font-family: "JetBrains Mono", "Consolas", monospace; font-size: 14px; color: #eef1f5;
}
QLabel#tlmValue[fault="true"] { color: #e5484d; font-weight: 600; }
QLabel#sectionLabel { color: #7f8893; font-size: 11px; }

/* ---- buttons ---- */
QComboBox, QPushButton {
    background: #1b1f26; border: 1px solid #2b313b; border-radius: 6px;
    padding: 6px 13px; color: #dce0e6;
}
QPushButton:hover, QComboBox:hover { background: #222732; border-color: #3a4250; }
QPushButton:pressed { background: #161a20; }
QPushButton:disabled, QComboBox:disabled { color: #5b626c; border-color: #20242b; background: #14171d; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView {
    background: #1b1f26; border: 1px solid #2b313b; color: #dce0e6;
    selection-background-color: #2d9bdb; selection-color: #ffffff; outline: 0;
}

QPushButton#connectBtn, QPushButton#runBtn {
    background: #2d9bdb; border: 1px solid #2d9bdb; color: #ffffff; font-weight: 600;
}
QPushButton#connectBtn:hover, QPushButton#runBtn:hover { background: #3aabec; border-color: #3aabec; }

QPushButton#stopBtn {
    background: #c0322c; border: 1px solid #d23b34; border-radius: 6px;
    color: #ffffff; font-weight: 700; padding: 7px 20px;
}
QPushButton#stopBtn:hover { background: #d23b34; }
QPushButton#stopBtn:disabled { background: #3a201f; border-color: #4a2a28; color: #9a7a78; }

/* ---- status pills ---- */
QLabel#connState { color: #8a929c; padding: 0 8px; }
QLabel#connState[ok="true"] { color: #34d399; font-weight: 600; }
QLabel#reachState { font-weight: 600; }
QLabel#reachState[ok="true"] { color: #34d399; }
QLabel#reachState[bad="true"] { color: #f59e6b; }

/* ---- sliders ---- */
QSlider::groove:horizontal { height: 4px; background: #232932; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #2d9bdb; border-radius: 2px; }
QSlider::handle:horizontal {
    background: #e7eaef; width: 14px; height: 14px; margin: -6px 0; border-radius: 7px;
    border: 2px solid #2d9bdb;
}
QSlider::handle:horizontal:hover { background: #ffffff; }

/* ---- tabs ---- */
QTabWidget::pane { border: 1px solid #222730; border-radius: 8px; top: -1px; }
QTabBar::tab {
    background: transparent; color: #8a929c; padding: 7px 16px; margin-right: 2px;
    border: none; border-bottom: 2px solid transparent;
}
QTabBar::tab:hover { color: #c4cad2; }
QTabBar::tab:selected { color: #eef1f5; border-bottom: 2px solid #2d9bdb; }

/* ---- inputs ---- */
QDoubleSpinBox, QSpinBox, QLineEdit {
    background: #12151a; border: 1px solid #2b313b; border-radius: 6px;
    padding: 4px 7px; color: #eef1f5;
    selection-background-color: #2d9bdb; selection-color: #ffffff;
}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus { border-color: #2d9bdb; }
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button,
QSpinBox::up-button, QSpinBox::down-button { width: 14px; border: none; background: transparent; }

QRadioButton, QCheckBox { color: #c4cad2; spacing: 6px; }
QCheckBox::indicator, QRadioButton::indicator { width: 15px; height: 15px; }
QCheckBox::indicator {
    border: 1px solid #3a4250; border-radius: 4px; background: #12151a;
}
QCheckBox::indicator:checked { background: #2d9bdb; border-color: #2d9bdb; }
QRadioButton::indicator { border: 1px solid #3a4250; border-radius: 8px; background: #12151a; }
QRadioButton::indicator:checked { background: #2d9bdb; border-color: #2d9bdb; }

/* ---- timeline table ---- */
QTableWidget {
    background: #12151a; alternate-background-color: #14181e;
    border: 1px solid #222730; border-radius: 8px; gridline-color: #1e232b;
    color: #dce0e6;
}
QTableWidget::item { padding: 3px 6px; }
QTableWidget::item:selected { background: #1d3a4d; color: #eef1f5; }
QHeaderView::section {
    background: #161a20; color: #7f8893; border: none;
    border-bottom: 1px solid #262b33; padding: 6px 8px; font-size: 11px;
}

/* ---- log ---- */
QPlainTextEdit#eventLog {
    background: #0c0e12; border: 1px solid #222730; border-radius: 8px;
    color: #9aa2ac; font-family: "JetBrains Mono", "Consolas", monospace; font-size: 12px;
}

/* ---- chrome ---- */
QStatusBar { background: #0c0e12; color: #7f8893; border-top: 1px solid #1c2128; }
QStatusBar::item { border: none; }
QSplitter::handle { background: #1c2128; }
QSplitter::handle:hover { background: #2d9bdb; }
QWidget#timeline { border-top: 1px solid #1c2128; background: #0f1115; }
QToolTip {
    background: #1b1f26; color: #e7eaef; border: 1px solid #2b313b;
    border-radius: 4px; padding: 4px 7px;
}

/* ---- scrollbars ---- */
QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }
QScrollBar::handle:vertical { background: #2b313b; border-radius: 5px; min-height: 28px; }
QScrollBar::handle:vertical:hover { background: #3a4250; }
QScrollBar:horizontal { background: transparent; height: 10px; margin: 2px; }
QScrollBar::handle:horizontal { background: #2b313b; border-radius: 5px; min-width: 28px; }
QScrollBar::handle:horizontal:hover { background: #3a4250; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; width: 0; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
"""
