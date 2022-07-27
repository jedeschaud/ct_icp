#!/usr/bin/env python
import random

import rospy
import rostopic
import std_msgs
from std_msgs.msg import String

import wandb

# Import Custom message
from slam_roscore.msg import monitor_entry


# Callback logging to Wandb
def callback(entry):
    wandb.log({entry.key: entry.value})


subscribers = {}


# Subscribers
def register_subscribers(allowed_topics=None):
    pubs, subs = rostopic.get_topic_list()
    for topic in pubs:
        name = topic[0]
        dtype = topic[1]

        if name not in subscribers and dtype == "slam_roscore/monitor_entry":
            subscribers[name] = rospy.Subscriber(name, monitor_entry, callback, queue_size=5000)
            rospy.loginfo(f"Subscribing to topic {name}")


# Handle Commands
def handle_commands(command):
    rospy.loginfo(f"Received Command: {command}")
    if command.data == "update_subs":
        rospy.loginfo(f"Registering Subscribers")
        register_subscribers()
    return


# Main script
def main():
    wandb.init(project="slam-monitor", entity="ct_icp", config={"test": "test1"})
    topics = rospy.get_param("topics", "")
    if topics != "":
        try:
            topics = topics.split(",")
        except (RuntimeError, Exception):
            rospy.signal_shutdown()
            wandb.finish()
            raise RuntimeError(f"Cannot split the string `{topics}` into a list of topics")
    else:
        rospy.loginfo("No topics selected, will try to find all topics")

    rospy.init_node("monitor", anonymous=True)

    # Subscribe to any topic having type monitor_entry
    register_subscribers()
    rospy.Subscriber("/monitor/command", std_msgs.msg.String, handle_commands)
    rospy.Subscriber("/monitor/entry", monitor_entry, callback, queue_size=5000)

    rospy.spin()
    wandb.finish()


if __name__ == "__main__":
    main()
