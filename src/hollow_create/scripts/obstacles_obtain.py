#! /home/by/software/anaconda3/bin/python

import json
import rospy
from hollow_create.msg import ObstacleObject
from hollow_create.msg import Obstacles
from geometry_msgs.msg import Point


file_name = rospy.get_param('/obstacles_create/obstacle_file_path')

'''
get json str from specific file
'''
with open(file_name, 'r') as f:
  obstacles_json = json.loads(json.load(f))


obstacles = Obstacles()

for obstacle in obstacles_json:
  obs_temp = ObstacleObject()

  # sign
  obs_temp.is_static = obstacle['is_static']
  obs_temp.label = obstacle['label']

  # obstacle size
  obs_temp.obs_box.x = obstacle['obs_box']['x']
  obs_temp.obs_box.y = obstacle['obs_box']['y']
  obs_temp.obs_box.z = obstacle['obs_box']['z']

  if obs_temp.is_static:
    obs_temp.center.x = 200
    obs_temp.center.y = 0.0
    obs_temp.center.z = 0.0
    obs_temp.delay_time = 0.0
  else:
    #yield case obstacle center
    obs_temp.center.x = 106.0
    obs_temp.center.y = 3.6
    obs_temp.center.z = 0.0
    obs_temp.delay_time = 25.0

  # # overtake case obstacle center
  # obs_temp.center.x = 36.0
  # obs_temp.center.y = 10.0
  # obs_temp.center.z = -180 / 57.3

  # # # path planning obstacle center point
  # obs_temp.center.x = 178.0260805534199
  # obs_temp.center.y = 492.6955370102078
  # obs_temp.center.z = 30


  # obstacle path
  if not obs_temp.is_static:
    for path_point in obstacle['path']:
      obs_temp.path.append(Point(path_point['x'], path_point['y'], path_point['heading']))

  obstacles.obstacles.append(obs_temp)


def talker():
  rospy.init_node('obstacles_create')
  pub = rospy.Publisher('~obstacles_info', Obstacles, queue_size=1)
  rate = rospy.Rate(1000) # Hz

  while not rospy.is_shutdown():
    pub.publish(obstacles)

    rate.sleep()


if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInitException:
    pass
      
  


