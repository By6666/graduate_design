#! /home/by/software/anaconda3/bin/python

import rospy
import pandas as pd
from eco_references.msg import EcoReferencesPoint
from eco_references.msg import EcoReferences


file_name = rospy.get_param('/eco_references/file_name')

'''
get eco references data from specific file
'''

eco_ref_data = pd.read_csv(file_name).to_numpy()

eco_ref = EcoReferences()

for point in eco_ref_data:
  ref_point = EcoReferencesPoint()
  ref_point.t = point[0]
  ref_point.s = point[1]
  ref_point.v = point[2]
  ref_point.a = point[3]
  # print('eco_ref_publish *************')


  eco_ref.eco_ref_path.append(ref_point)


def talker():
  rospy.init_node('eco_references')
  pub = rospy.Publisher('~eco_path', EcoReferences, queue_size=1)
  rate = rospy.Rate(10) #Hz

  while not rospy.is_shutdown():
    pub.publish(eco_ref)

    rate.sleep()


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSException:
    pass
