#! /usr/bin/env python

import collections
import math
from mongodb_store.message_store import MessageStoreProxy
from landmarks_study.msg import Event
from landmarks_study.msg import Participant
from landmarks_study.msg import Task
import rospy

# Returns time in seconds of an event type.
def get_time(time_counter, event_type):
    return time_counter[event_type].to_sec() if event_type in time_counter else 0

def format_time(secs):
    minutes = int(math.floor(secs / 60))
    seconds = int(round((secs % 60)))
    return '{}m {}s'.format(minutes, seconds)

def process(participant, task):
    features = {}
    features['Participant name'] = participant.name
    features['Task name'] = task.name

    event_counter = collections.Counter()
    time_counter = {}
    prev_task = None
    prev_stamp = None
    for event in task.events:
        event_counter[event.type] += 1

        if prev_task is not None:
            duration = event.stamp - prev_stamp
            if prev_task in time_counter:
                time_counter[prev_task] += duration
            else:
                time_counter[prev_task] = duration

        prev_task = event.type
        prev_stamp = event.stamp

    load_time = get_time(time_counter, Event.LOAD)
    total_time = rospy.Duration(0)
    for v in time_counter.values():
        total_time += v
    total_time = total_time.to_sec() - load_time

    total_test_time = get_time(time_counter, Event.TEST) + get_time(time_counter, Event.FINISHED_TEST)

    features['# edits'] = event_counter[Event.EDIT]
    features['# tests'] = event_counter[Event.TEST]
    features['Total task time'] = format_time(total_time)
    features['Edit time'] = format_time(get_time(time_counter, Event.EDIT))
    features['Test time (user)'] = format_time(get_time(time_counter, Event.FINISHED_TEST))
    features['Test time (total)'] = format_time(total_test_time)
    return features

def main():
    rospy.init_node('data_analysis')
    participant_db = MessageStoreProxy(database='landmarks_study', collection='participants')
    participants = participant_db.query(Participant._type)
    data = []
    for p, meta in participants:
        for t in p.tasks:
            features = process(p, t)
            data.append(features)
    cols = ['Participant name', 'Task name', '# edits','# tests', 'Total task time', 'Edit time', 'Test time (user)', 'Test time (total)']
    print ','.join([c for c in cols if (c in data[0])])
    for d in data:
        features = [str(d[col]) for col in cols if col in d]
        print ','.join(features)

if __name__ == '__main__':
    main()
