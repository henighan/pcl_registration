#!/usr/bin/env python

import rospy
import tf
import sys

if __name__ == '__main__':
  if len(sys.argv) != 10:
    print 'multcam_transform_publisher error: expecting 9 arguments, \
        received %d' % len(sys.argv)
    print 'arguments should be as follows: '
    print 'transx transy transz rotr rotp roty frame_id child_frame_id \
        periodinmilliseconds'
    raise
  camtf = sys.argv[1:]
  rospy.init_node('camera_tf_broadcaster')
  br = tf.TransformBroadcaster()
  print " will broadcast every %f milliseconds" % float(sys.argv[9])
  while not rospy.is_shutdown():
    br.sendTransform((float(camtf[0]), float(camtf[1]), float(camtf[2])), 
      tf.transformations.quaternion_from_euler(float(camtf[3]),float(camtf[4]),float(camtf[5])),
      rospy.Time.now(),
      camtf[6],
      camtf[7])
    rospy.sleep(float(sys.argv[9])/1000)
