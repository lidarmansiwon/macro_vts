#!/usr/bin/env python3

from rqt_gui_py.plugin import Plugin
from macro_vts.vts_widget import VTSWidget  # UI 클래스 import

class MacroVTS(Plugin):
    def __init__(self, context):
        super(MacroVTS, self).__init__(context)
        self.setObjectName('MacroVTS')

        # Create widget instance and pass ROS 2 node
        self._widget = VTSWidget(context.node)

        # Multi-instance 대응 (rqt에서 같은 plugin을 여러 번 열 때)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + f' ({context.serial_number()})')

        # Add to main rqt window
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        """Shutdown logic (unsubscribe, stop timers, etc)"""
        if hasattr(self._widget, 'shutdown_widget'):
            self._widget.shutdown_widget()
