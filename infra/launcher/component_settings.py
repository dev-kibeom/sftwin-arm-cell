"""Launcher tabs for viewing defaults and editing one run's component settings."""
import yaml
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QDoubleSpinBox, QFormLayout, QLabel, QScrollArea,
                               QSpinBox, QTabWidget, QVBoxLayout, QWidget)

from arm_cell_bringup.runtime import component_experiment_settings


class ComponentSettings(QTabWidget):
    def __init__(self, tool_panel):
        super().__init__()
        self.forms = {}
        self.edits = {}
        self.profile = None
        self.error = None
        for section, title in (('simulation', '시뮬레이션'), ('motion', '모션'), ('vision', '비전')):
            panel = QWidget()
            layout = QVBoxLayout(panel)
            self.forms[section] = QFormLayout()
            layout.addLayout(self.forms[section])
            if section == 'simulation':
                layout.addWidget(tool_panel)
            layout.addStretch()
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(panel)
            self.addTab(scroll, title)
        self.load_profile(None)

    def load_profile(self, profile):
        self.profile = profile
        self.edits.clear()
        self.error = None
        try:
            settings = component_experiment_settings(profile) if profile is not None else {}
        except (KeyError, ValueError, OSError, yaml.YAMLError) as exc:
            self.error = f'설정 파일을 읽을 수 없습니다. {exc}'
            settings = {}

        for section, form in self.forms.items():
            while form.rowCount():
                form.removeRow(0)
            if section not in settings:
                if self.error:
                    text = '설정을 불러오지 못했습니다.'
                elif profile is None:
                    text = '실행할 프로파일을 선택하세요.'
                else:
                    text = '선택한 프로파일에 이 컴포넌트가 없습니다.'
                form.addRow(QLabel(text))
                continue
            source, fields = settings[section]
            source_label = QLabel(str(source))
            source_label.setTextFormat(Qt.PlainText)
            source_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            source_label.setWordWrap(True)
            form.addRow('기본값 파일', source_label)
            if not fields:
                form.addRow(QLabel('이 프로파일에서 조정할 수 있는 항목이 없습니다.'))
            for key, (field, default) in fields.items():
                if not field.accepts(default):
                    value = QLabel(f'{default} (실험 편집 범위 밖)')
                    value.setTextFormat(Qt.PlainText)
                    form.addRow(field.label, value)
                    continue
                edit = QSpinBox() if field.integer else QDoubleSpinBox()
                if not field.integer:
                    edit.setDecimals(6)
                edit.setRange(field.minimum, field.maximum)
                edit.setValue(default)
                edit.setObjectName(f'{section}.{key}')
                edit.setToolTip(f'{key}\nYAML 기본값: {default}')
                initial = edit.value()
                edit.valueChanged.connect(lambda value, widget=edit, initial=initial: widget.setStyleSheet(
                    'font-weight: bold; color: #b06000;' if value != initial else ''))
                form.addRow(field.label, edit)
                self.edits[section, key] = (edit, initial)

    def overrides(self):
        if self.error:
            raise ValueError(self.error)
        edits = {}
        for (section, key), (edit, initial) in self.edits.items():
            if edit.value() != initial:
                edits.setdefault(section, {})[key] = edit.value()
        return edits
