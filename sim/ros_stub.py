"""Minimal rclpy / std_msgs / sensor_msgs / cv_bridge stub.

Just enough so the real jetson_driver.{perception_node, planner_node,
control_node} modules import and instantiate.  No threading, no DDS, no
actual pub/sub — instead, every Node stores its publishers/subscriptions
in dicts that the simulator drives directly.

Usage:
    import ros_stub as _  # patches sys.modules
    from jetson_driver.planner_node import PlannerNode
    p = PlannerNode()
    # inject a message:
    p._trigger_cb(StubString('go'))
    # tick the timer:
    p._tick()
    # read latest publication:
    latest = SimBus.get('/behavior')
"""

import sys
import time as _time
import types


# ---------------------------------------------------------------------------
# Message types — simple data wrappers.
# ---------------------------------------------------------------------------
class _Msg:
    def __init__(self, data=None):
        self.data = data
class Float32(_Msg): pass
class Bool(_Msg): pass
class String(_Msg): pass
class Int32(_Msg): pass
class Float32MultiArray:
    def __init__(self, data=None):
        self.data = data if data is not None else []
class Image:
    def __init__(self):
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=0, nanosec=0),
            frame_id='')


# ---------------------------------------------------------------------------
# Central bus — records last published message per topic.
# ---------------------------------------------------------------------------
class SimBus:
    _latest = {}     # topic -> last message
    _history = {}    # topic -> list of messages

    @classmethod
    def reset(cls):
        cls._latest.clear()
        cls._history.clear()

    @classmethod
    def get(cls, topic):
        return cls._latest.get(topic)

    @classmethod
    def get_value(cls, topic):
        m = cls._latest.get(topic)
        return None if m is None else m.data

    @classmethod
    def history(cls, topic):
        return list(cls._history.get(topic, []))

    @classmethod
    def _record(cls, topic, msg):
        cls._latest[topic] = msg
        cls._history.setdefault(topic, []).append(msg)


# ---------------------------------------------------------------------------
# Publisher / subscription / parameter / logger / clock / timer.
# ---------------------------------------------------------------------------
class _StubPublisher:
    def __init__(self, topic):
        self.topic = topic
    def publish(self, msg):
        SimBus._record(self.topic, msg)

class _StubSubscription:
    def __init__(self, topic, cb):
        self.topic = topic
        self.cb = cb

class _StubParam:
    def __init__(self, value):
        self.value = value

class _StubLogger:
    def __init__(self, name):
        self.name = name
        self.records = []
    def _log(self, level, msg):
        self.records.append((level, msg))
    def info(self, msg):  self._log('INFO',  msg)
    def warn(self, msg):  self._log('WARN',  msg)
    def warning(self, msg): self._log('WARN', msg)
    def error(self, msg): self._log('ERROR', msg)
    def debug(self, msg): self._log('DEBUG', msg)

class _StubClock:
    def now(self):
        return types.SimpleNamespace(
            to_msg=lambda: types.SimpleNamespace(sec=0, nanosec=0))

class _StubTimer:
    def __init__(self, period, cb):
        self.period = period
        self.cb = cb

# ---------------------------------------------------------------------------
# Node base class — what every jetson_driver node inherits.
# ---------------------------------------------------------------------------
class Node:
    def __init__(self, name):
        self._name = name
        self._params = {}
        self._logger = _StubLogger(name)
        self._clock = _StubClock()
        self.publishers_by_topic = {}
        self.subs_by_topic = {}
        self.timers = []

    def declare_parameter(self, name, default):
        if name not in self._params:
            self._params[name] = _StubParam(default)
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    def set_parameter(self, name, value):
        """Test helper — not in real rclpy."""
        self._params[name] = _StubParam(value)

    def create_publisher(self, msg_type, topic, qos):
        p = _StubPublisher(topic)
        self.publishers_by_topic[topic] = p
        return p

    def create_subscription(self, msg_type, topic, cb, qos):
        s = _StubSubscription(topic, cb)
        self.subs_by_topic[topic] = s
        return s

    def create_timer(self, period, cb):
        t = _StubTimer(period, cb)
        self.timers.append(t)
        return t

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


# ---------------------------------------------------------------------------
# QoS placeholder
# ---------------------------------------------------------------------------
class _QosSensorData:
    pass
qos_profile_sensor_data = _QosSensorData()


# ---------------------------------------------------------------------------
# Patch sys.modules so `import rclpy`, `from std_msgs.msg import ...` work
# without the real installs.
# ---------------------------------------------------------------------------
def _install():
    rclpy_mod = types.ModuleType('rclpy')
    rclpy_mod.init = lambda args=None: None
    rclpy_mod.spin = lambda node: None
    rclpy_mod.shutdown = lambda: None
    rclpy_mod.ok = lambda: True

    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = Node
    rclpy_mod.node = rclpy_node

    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.qos_profile_sensor_data = qos_profile_sensor_data
    rclpy_mod.qos = rclpy_qos

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Float32 = Float32
    std_msgs_msg.Bool = Bool
    std_msgs_msg.String = String
    std_msgs_msg.Int32 = Int32
    std_msgs_msg.Float32MultiArray = Float32MultiArray
    std_msgs.msg = std_msgs_msg

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.Image = Image
    sensor_msgs.msg = sensor_msgs_msg

    # cv_bridge stub — just enough that PerceptionNode constructs.
    cv_bridge = types.ModuleType('cv_bridge')
    class CvBridge:
        def imgmsg_to_cv2(self, msg, desired_encoding='bgr8'):
            return getattr(msg, '_frame', None)
        def cv2_to_imgmsg(self, frame, encoding='bgr8'):
            img = Image()
            img._frame = frame
            return img
    cv_bridge.CvBridge = CvBridge

    sys.modules['rclpy'] = rclpy_mod
    sys.modules['rclpy.node'] = rclpy_node
    sys.modules['rclpy.qos'] = rclpy_qos
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = sensor_msgs_msg
    sys.modules['cv_bridge'] = cv_bridge

_install()
