import os
import rospy
import rospkg
from roslaunch.substitution_args import resolve_args
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String
from interbotix_xs_sdk.srv import TorqueEnable, TorqueEnableRequest


def substitute_xml_args(param):
    # substitute string
    if isinstance(param, str):
        param = resolve_args(param)
        return param

    # For every key in the dictionary (not performing deepcopy!)
    if isinstance(param, dict):
        for key in param:
            # If the value is of type `(Ordered)dict`, then recurse with the value
            if isinstance(param[key], dict):
                substitute_xml_args(param[key])
            # Otherwise, add the element to the result
            elif isinstance(param[key], str):
                param[key] = resolve_args(param[key])


class EagerxCalibrationRqt(Plugin):
    def __init__(self, context):
        super(EagerxCalibrationRqt, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("EagerxCalibrationRqt")

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser

        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print("arguments: ", args)
            print("unknowns: ", unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(
            rospkg.RosPack().get_path("interbotix_calibration_rqt"), "resource", "interbotix_calibration.ui"
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName("EagerxCalibrationRqtUi")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (" (%d)" % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.event_publisher = rospy.Publisher("interbotix_calibration/event_in", String, queue_size=1, latch=True)
        self.torque_service = rospy.ServiceProxy("torque_enable", TorqueEnable)

        # Switch controller service
        self._switch_controller_service = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)

        # Calibration Buttons
        self._widget.calibrateButton.clicked[bool].connect(self.handle_calibrate)
        self._widget.saveButton.clicked[bool].connect(self.handle_save)
        self._widget.calculateButton.clicked[bool].connect(self.handle_calculate)

        # Manipulator Buttons
        self._widget.homeButton.clicked[bool].connect(self.handle_home)
        self._widget.uprightButton.clicked[bool].connect(self.handle_upright)
        self._widget.sleepButton.clicked[bool].connect(self.handle_sleep)
        self._widget.upButton.clicked[bool].connect(self.handle_up)
        self._widget.stopButton.clicked[bool].connect(self.handle_stop)
        self._widget.releaseButton.clicked[bool].connect(self.handle_release)

        # Gripper Buttons
        self._widget.openButton.clicked[bool].connect(self.handle_open)
        self._widget.closeButton.clicked[bool].connect(self.handle_close)

        # Torque Buttons
        self._widget.enableButton.clicked[bool].connect(self.handle_enable)
        self._widget.disableButton.clicked[bool].connect(self.handle_disable)

    def shutdown_plugin(self):
        self.event_publisher.unregister()
        self.torque_service.close()

    def handle_calibrate(self):
        msg = String("calibrate")
        self.event_publisher.publish(msg)

    def handle_save(self):
        msg = String("save")
        self.event_publisher.publish(msg)

    def handle_calculate(self):
        msg = String("calculate")
        self.event_publisher.publish(msg)

    def handle_home(self):
        msg = String("home")
        self.event_publisher.publish(msg)

    def handle_upright(self):
        msg = String("upright")
        self.event_publisher.publish(msg)

    def handle_sleep(self):
        msg = String("sleep")
        self.event_publisher.publish(msg)

    def handle_up(self):
        msg = String("up")
        self.event_publisher.publish(msg)

    def handle_stop(self):
        request = SwitchControllerRequest()
        request.stop_controllers = ["arm_controller", "gripper_controller"]
        request.strictness = 2
        request.start_asap = True
        self._switch_controller_service(request)

    def handle_release(self):
        request = SwitchControllerRequest()
        request.start_controllers = ["arm_controller", "gripper_controller"]
        request.strictness = 1
        request.start_asap = True
        self._switch_controller_service(request)

    def handle_open(self):
        msg = String("open")
        self.event_publisher.publish(msg)

    def handle_close(self):
        msg = String("close")
        self.event_publisher.publish(msg)

    def handle_enable(self):
        req = TorqueEnableRequest()
        req.cmd_type = "group"
        req.name = "arm"
        req.enable = True
        self.torque_service.call(req)

    def handle_disable(self):
        req = TorqueEnableRequest()
        req.cmd_type = "group"
        req.name = "arm"
        req.enable = False
        self.torque_service.call(req)
