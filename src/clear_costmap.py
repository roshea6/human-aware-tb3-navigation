#!/usr/bin/env python

import rospy
import rosservice
import time

if __name__ == "__main__":

    while(True):
        rosservice.call_service("/move_base/clear_costmaps", service_args=None)

        time.sleep(1)