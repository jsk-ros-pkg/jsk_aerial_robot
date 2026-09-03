#!/usr/bin/env python
"""Send raw sinusoidal servo targets and monitor ServoStates update rates."""

from __future__ import print_function

import math
import threading
import time
from collections import deque

import rospy

from spinal.msg import ServoControlCmd, ServoStates


# =============================================================================
# User configuration
# =============================================================================

# bridge.launch exposes relative servo topic names. Set this to the namespace in
# which bridge.launch is started (for example "/dragon"), or leave it empty for
# the root topics /servo/target_states and /servo/states.
ROBOT_NAMESPACE = ""
COMMAND_TOPIC = "servo/target_states"
STATE_TOPIC = "servo/states"

# Safety switch: inspect state statistics only until this is explicitly enabled.
SEND_COMMANDS = True

# ServoControlCmd.angles uses raw int16 servo units, not radians.
# Each dictionary controls one servo. Multiple entries are sent in one message.
SERVO_TARGETS = [
    {
        "index": index,
        "center": 2048,
        "amplitude": 2000,
        "phase_deg": 0.0,
    }
    for index in range(4)
]

# Target waveform and command publishing frequency.
SINE_FREQUENCY_HZ = 0.25
COMMAND_PUBLISH_RATE_HZ = 200.0

# 0.0 means run until Ctrl+C. When command output is enabled, the script sends
# one final center command on exit if RETURN_TO_CENTER_ON_EXIT is True.
TEST_DURATION_SEC = 0.0
RETURN_TO_CENTER_ON_EXIT = True

# Statistics are computed over a sliding receive-time window and printed at the
# interval below. An empty list reports every servo index observed in the topic.
STATS_WINDOW_SEC = 10.0
REPORT_INTERVAL_SEC = 5.0
REPORT_SERVO_INDICES = []

# Check actual value transitions independently of the fixed-rate ServoStates
# publication. ServoState calls position "angle" in the ROS message.
STATE_CHANGE_FIELDS = ("angle", "velocity", "load")

# Mark a field UNCHANGED after no value transition has been observed for this
# many seconds. Set to 0.0 to disable the UNCHANGED marker.
UNCHANGED_WARNING_SEC = 2.0

PUBLISHER_QUEUE_SIZE = 1
SUBSCRIBER_QUEUE_SIZE = 100

# =============================================================================
# End of user configuration
# =============================================================================


STATE_FIELDS = ("angle", "velocity", "temp", "load", "error")
STATE_FIELD_LABELS = {
    "angle": "position",
    "velocity": "velocity",
    "load": "load",
}
INT16_MIN = -32768
INT16_MAX = 32767


def resolve_topic(namespace, topic):
    """Resolve a relative topic beneath namespace while preserving absolutes."""
    if topic.startswith("/"):
        return topic
    parts = [part.strip("/") for part in (namespace, topic) if part.strip("/")]
    return "/" + "/".join(parts)


def event_rate(event_times):
    """Estimate event frequency from the first and last event in a window."""
    if len(event_times) < 2:
        return 0.0
    elapsed = event_times[-1] - event_times[0]
    return float(len(event_times) - 1) / elapsed if elapsed > 0.0 else 0.0


def value_change_rate(change_times, sample_times):
    """Return value transitions per second over a servo's observation span."""
    if not sample_times:
        return 0.0
    elapsed = sample_times[-1] - sample_times[0]
    if elapsed <= 0.0:
        return 0.0
    return float(len(change_times)) / elapsed


class ServoRateMonitor(object):
    def __init__(self, state_topic):
        self._lock = threading.Lock()
        self._topic_times = deque()
        self._source_stamps = deque()
        self._servos = {}
        self._subscriber = rospy.Subscriber(
            state_topic,
            ServoStates,
            self._callback,
            queue_size=SUBSCRIBER_QUEUE_SIZE,
        )

    @staticmethod
    def _new_servo_record():
        return {
            "sample_times": deque(),
            "first_sample_time": None,
            "change_times": dict((field, deque()) for field in STATE_CHANGE_FIELDS),
            "last_change_time": {},
            "latest": {},
        }

    @staticmethod
    def _prune_deque(values, oldest_time, time_getter=None):
        if time_getter is None:
            time_getter = lambda item: item
        while values and time_getter(values[0]) < oldest_time:
            values.popleft()

    def _prune_locked(self, now):
        oldest_time = now - STATS_WINDOW_SEC
        self._prune_deque(self._topic_times, oldest_time)
        self._prune_deque(
            self._source_stamps, oldest_time, time_getter=lambda item: item[0]
        )
        for record in self._servos.values():
            self._prune_deque(record["sample_times"], oldest_time)
            for changes in record["change_times"].values():
                self._prune_deque(changes, oldest_time)

    def _callback(self, msg):
        now = time.time()
        with self._lock:
            self._topic_times.append(now)
            self._source_stamps.append((now, msg.stamp.to_sec()))

            indices_in_message = set()
            for servo in msg.servos:
                index = int(servo.index)
                if index in indices_in_message:
                    rospy.logwarn_throttle(
                        5.0, "duplicate servo index %d in %s", index, STATE_TOPIC
                    )
                    continue
                indices_in_message.add(index)

                record = self._servos.setdefault(index, self._new_servo_record())
                record["sample_times"].append(now)
                if record["first_sample_time"] is None:
                    record["first_sample_time"] = now
                for field in STATE_FIELDS:
                    value = getattr(servo, field)
                    if (
                        field in STATE_CHANGE_FIELDS
                        and field in record["latest"]
                        and value != record["latest"][field]
                    ):
                        record["change_times"][field].append(now)
                        record["last_change_time"][field] = now
                    record["latest"][field] = value

            self._prune_locked(now)

    @staticmethod
    def _source_stamp_rate(source_stamps):
        # Consecutive duplicate or zero stamps are not independent source ticks.
        distinct = []
        for _, stamp in source_stamps:
            if stamp <= 0.0:
                continue
            if not distinct or stamp != distinct[-1]:
                distinct.append(stamp)
        if len(distinct) < 2 or distinct[-1] <= distinct[0]:
            return 0.0
        return float(len(distinct) - 1) / (distinct[-1] - distinct[0])

    def report(self, state_topic):
        now = time.time()
        with self._lock:
            self._prune_locked(now)
            topic_times = list(self._topic_times)
            source_stamps = list(self._source_stamps)
            servo_snapshot = {}
            for index, record in self._servos.items():
                servo_snapshot[index] = {
                    "sample_times": list(record["sample_times"]),
                    "change_times": dict(
                        (field, list(times))
                        for field, times in record["change_times"].items()
                    ),
                    "first_sample_time": record["first_sample_time"],
                    "last_change_time": dict(record["last_change_time"]),
                    "latest": dict(record["latest"]),
                }

        rospy.loginfo(
            "%s: receive %.2f Hz, source stamp %.2f Hz, %d messages in %.1f s window",
            state_topic,
            event_rate(topic_times),
            self._source_stamp_rate(source_stamps),
            len(topic_times),
            STATS_WINDOW_SEC,
        )

        configured_indices = set(REPORT_SERVO_INDICES)
        indices = sorted(configured_indices or servo_snapshot.keys())
        if not indices:
            rospy.logwarn("no servo state has been received from %s", state_topic)
            return

        rospy.loginfo(
            " idx | sample_Hz age_s | actual state changes in %.1f s window",
            STATS_WINDOW_SEC,
        )
        for index in indices:
            record = servo_snapshot.get(index)
            if record is None or not record["sample_times"]:
                rospy.loginfo("%4d | no samples in current window", index)
                continue

            sample_times = record["sample_times"]
            rospy.loginfo(
                "%4d | %9.2f %5.2f | temp=%s error=%s",
                index,
                event_rate(sample_times),
                max(0.0, now - sample_times[-1]),
                record["latest"].get("temp", "-"),
                record["latest"].get("error", "-"),
            )
            for field in STATE_CHANGE_FIELDS:
                changes = record["change_times"][field]
                changed_in_window = bool(changes)
                last_change_time = record["last_change_time"].get(field)
                if last_change_time is None:
                    last_change_age = now - record["first_sample_time"]
                    age_prefix = ">"
                else:
                    last_change_age = now - last_change_time
                    age_prefix = ""
                unchanged = (
                    UNCHANGED_WARNING_SEC > 0.0
                    and last_change_age >= UNCHANGED_WARNING_SEC
                )
                rospy.loginfo(
                    "     %-8s changed=%-3s count=%4d actual_change=%6.2f Hz "
                    "last_change=%s%.2f s value=%s%s",
                    STATE_FIELD_LABELS.get(field, field),
                    "YES" if changed_in_window else "NO",
                    len(changes),
                    value_change_rate(changes, sample_times),
                    age_prefix,
                    last_change_age,
                    record["latest"].get(field, "-"),
                    " UNCHANGED" if unchanged else "",
                )


class ServoSineTest(object):
    def __init__(self):
        self.command_topic = resolve_topic(ROBOT_NAMESPACE, COMMAND_TOPIC)
        self.state_topic = resolve_topic(ROBOT_NAMESPACE, STATE_TOPIC)
        self.targets = self._validate_configuration()
        self.publisher = rospy.Publisher(
            self.command_topic,
            ServoControlCmd,
            queue_size=PUBLISHER_QUEUE_SIZE,
        )
        self.monitor = ServoRateMonitor(self.state_topic)
        self._last_report_time = time.time()
        self._sent_command = False
        rospy.on_shutdown(self._on_shutdown)

    @staticmethod
    def _validate_configuration():
        if COMMAND_PUBLISH_RATE_HZ <= 0.0:
            raise ValueError("COMMAND_PUBLISH_RATE_HZ must be positive")
        if SINE_FREQUENCY_HZ < 0.0:
            raise ValueError("SINE_FREQUENCY_HZ cannot be negative")
        if STATS_WINDOW_SEC <= 0.0 or REPORT_INTERVAL_SEC <= 0.0:
            raise ValueError("statistics window and report interval must be positive")
        if UNCHANGED_WARNING_SEC < 0.0:
            raise ValueError("UNCHANGED_WARNING_SEC cannot be negative")
        if TEST_DURATION_SEC < 0.0:
            raise ValueError("TEST_DURATION_SEC cannot be negative")
        invalid_fields = set(STATE_CHANGE_FIELDS) - set(STATE_FIELDS)
        if invalid_fields:
            raise ValueError(
                "unknown STATE_CHANGE_FIELDS: {}".format(sorted(invalid_fields))
            )

        targets = []
        used_indices = set()
        for target in SERVO_TARGETS:
            index = int(target["index"])
            center = float(target["center"])
            amplitude = float(target["amplitude"])
            phase_rad = math.radians(float(target.get("phase_deg", 0.0)))

            if index < 0 or index > 255:
                raise ValueError("servo index {} is outside uint8 range".format(index))
            if index in used_indices:
                raise ValueError("servo index {} is configured twice".format(index))
            if amplitude < 0.0:
                raise ValueError("servo {} amplitude cannot be negative".format(index))
            if center - amplitude < INT16_MIN or center + amplitude > INT16_MAX:
                raise ValueError(
                    "servo {} sine target exceeds int16 range".format(index)
                )

            used_indices.add(index)
            targets.append((index, center, amplitude, phase_rad))

        if SEND_COMMANDS and not targets:
            raise ValueError("SERVO_TARGETS cannot be empty when SEND_COMMANDS is True")
        return targets

    def _publish_at_elapsed_time(self, elapsed):
        msg = ServoControlCmd()
        msg.index = [target[0] for target in self.targets]
        msg.angles = [
            int(
                round(
                    target[1]
                    + target[2]
                    * math.sin(
                        2.0 * math.pi * SINE_FREQUENCY_HZ * elapsed + target[3]
                    )
                )
            )
            for target in self.targets
        ]
        self.publisher.publish(msg)
        self._sent_command = True

    def _on_shutdown(self):
        if not (
            SEND_COMMANDS and RETURN_TO_CENTER_ON_EXIT and self._sent_command
        ):
            return
        msg = ServoControlCmd()
        msg.index = [target[0] for target in self.targets]
        msg.angles = [int(round(target[1])) for target in self.targets]
        self.publisher.publish(msg)
        rospy.loginfo("sent final center target to %s", self.command_topic)
        rospy.sleep(0.05)

    def run(self):
        rospy.loginfo("monitoring servo states on %s", self.state_topic)
        rospy.logwarn(
            "sample_Hz measures fixed-rate message entries; position/velocity/load "
            "actual_change measures real value transitions in the statistics window"
        )
        if SEND_COMMANDS:
            rospy.logwarn(
                "COMMAND OUTPUT ENABLED: publishing raw int16 targets to %s at %.2f Hz",
                self.command_topic,
                COMMAND_PUBLISH_RATE_HZ,
            )
        else:
            rospy.logwarn(
                "command output disabled; set SEND_COMMANDS = True at the top of the script "
                "after checking index, center, and amplitude"
            )

        rate = rospy.Rate(COMMAND_PUBLISH_RATE_HZ)
        start_time = time.time()
        while not rospy.is_shutdown():
            now = time.time()
            elapsed = now - start_time
            if TEST_DURATION_SEC > 0.0 and elapsed >= TEST_DURATION_SEC:
                break
            if SEND_COMMANDS:
                self._publish_at_elapsed_time(elapsed)
            if now - self._last_report_time >= REPORT_INTERVAL_SEC:
                self.monitor.report(self.state_topic)
                self._last_report_time = now
            rate.sleep()


def main():
    rospy.init_node("servo_sine_test")
    try:
        ServoSineTest().run()
    except (KeyError, TypeError, ValueError) as error:
        rospy.logfatal("invalid servo_sine_test configuration: %s", error)


if __name__ == "__main__":
    main()
