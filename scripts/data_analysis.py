#! /usr/bin/env python

from mongodb_store.message_store import MessageStoreProxy
from landmarks_study.msg import Participant
import rospy

def main():
    rospy.init_node('data_analysis')
    participant_db = MessageStoreProxy(database='landmarks_study', collection='participants')
    ps = participant_db.query(Participant._type)
    print ps
    rospy.spin()

if __name__ == '__main__':
    main()
