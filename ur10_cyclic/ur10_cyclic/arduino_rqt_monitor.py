#!/usr/bin/env python3
from rqt_gui_py.plugin import Plugin
from ur10_cyclic.arduino_rqt_monitor_widget import ArduinoRqtMonitorWidget


class ArduinoRqtMonitor(Plugin):
    def __init__(self, plugin_context):
        super(ArduinoRqtMonitor, self).__init__(plugin_context)
        self._plugin_context = plugin_context

        self.mainwidget = ArduinoRqtMonitorWidget(self, plugin_context)

        plugin_context.add_widget(self.mainwidget)

    def get_widget(self):
        return self.mainwidget

    def shutdown_plugin(self):
        self.mainwidget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        self.mainwidget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self.mainwidget.restore_settings(plugin_settings, instance_settings)
