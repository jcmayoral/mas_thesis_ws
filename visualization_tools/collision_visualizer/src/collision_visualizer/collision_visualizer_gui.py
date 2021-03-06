import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import PyQt5.QtCore
#from python_qt_binding.QtGui import QLabel, QFont, QHBoxLayout, QVBoxLayout
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from collision_visualizer.ledwidget import LedWidget

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        #print type(context)
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        #if not args.quiet:
        #    print 'arguments: ', args
        #    print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        #ui_file = os.path.join(rospkg.RosPack().get_path('collision_visualizer'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        #loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        #if context.serial_number() > 1:
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        #context.add_widget(self._widget)
        layout = QVBoxLayout(self._widget)
        layout.setAlignment(PyQt5.QtCore.Qt.AlignJustify)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSizeConstraint(1)


        for i in range(4):
            led = LedWidget()
            led.setObjectName("collision_state_" + str(i))
            led.initializeSubscriber(i)
            led.setAlignment(PyQt5.QtCore.Qt.AlignCenter)

            #text = QLabel("sensing_source_" + str(i))
            #text.setObjectName("label_"+str(i))
            #text.setGeometry(10, 10, 100, 100);
            #text.resize(100,100)
            #text.setAlignment(PyQt5.QtCore.Qt.AlignCenter)
            #font = QFont()
            #font.setPointSize(20)
            #text.setFont(font)
            #context.add_widget(text)
            #context.add_widget(led)

            ###Work but disorder
            #context.add_widget(text)
            #context.add_widget(led)
            layout.addWidget(led)
            #layout.addWidget(text)

        #context.add_widget(self._widget)
        #context.remove_widget(self._widget)

        #text.setText("experiment")
        context.add_widget(self._widget)

        #context.add_widget(gridLayout)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self.trigger_configuration()

    def shutdown_plugin(self):
        pass

        # TODO unregister all publishers here
    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        #print "save ", plugin_settings,instance_settings
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        #print "restore ", plugin_settings.all_keys(),instance_settings
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        #print "trigger", type(self)
        pass
