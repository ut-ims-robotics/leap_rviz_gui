graph [
  version 2
  directed 1
  bb "0,0,2061.1,227.45"
  compound "True"
  rank "same"
  rankdir "LR"
  ranksep 0.2
  node [
    id 0
    name "n___leap_motion__stereo_image"
    URL "__leap_motion__stereo_image"
    label "/leap_motion/stereo_image"
    tooltip "/leap_motion/stereo_image"
    graphics [
      x 654.22
      y 170.45
      w 214.481
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/leap_motion/stereo_image"
    ]
  ]
  node [
    id 1
    name "n___leap_motion__link2_broadcaster"
    URL "__leap_motion__link2_broadcaster"
    label "/leap_motion/link2_broadcaster"
    tooltip "/leap_motion/link2_broadcaster"
    graphics [
      x 654.22
      y 116.45
      w 245.678
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/leap_motion/link2_broadcaster"
    ]
  ]
  node [
    id 2
    name "n___leap_visualizer"
    URL "__leap_visualizer"
    label "/leap_visualizer"
    tooltip "/leap_visualizer"
    graphics [
      x 654.22
      y 55.448
      w 132.588
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/leap_visualizer"
    ]
  ]
  node [
    id 3
    name "n___visual_manager"
    URL "__visual_manager"
    label "/visual_manager"
    tooltip "/visual_manager"
    graphics [
      x 1096.6
      y 36.448
      w 139.09
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/visual_manager"
    ]
  ]
  node [
    id 4
    name "n___move_group"
    URL "__move_group"
    label "/move_group"
    tooltip "/move_group"
    graphics [
      x 1325
      y 93.448
      w 115.69
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/move_group"
    ]
  ]
  node [
    id 5
    name "n___leap_filter"
    URL "__leap_filter"
    label "/leap_filter"
    tooltip "/leap_filter"
    graphics [
      x 312.99
      y 25.448
      w 98.7912
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/leap_filter"
    ]
  ]
  node [
    id 6
    name "n___leap_driver"
    URL "__leap_driver"
    label "/leap_driver"
    tooltip "/leap_driver"
    graphics [
      x 53.295
      y 25.448
      w 106.589
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/leap_driver"
    ]
  ]
  node [
    id 7
    name "n___joint_state_publisher"
    URL "__joint_state_publisher"
    label "/joint_state_publisher"
    tooltip "/joint_state_publisher"
    graphics [
      x 1713.9
      y 55.448
      w 174.182
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/joint_state_publisher"
    ]
  ]
  node [
    id 8
    name "n___robot_state_publisher"
    URL "__robot_state_publisher"
    label "/robot_state_publisher"
    tooltip "/robot_state_publisher"
    graphics [
      x 1972.1
      y 55.448
      w 178.085
      H 36
      type "ellipse"
    ]
    LabelGraphics [
      text "/robot_state_publisher"
    ]
  ]
  edge [
    id 10
    source 1
    target 3
    URL "topic_3A__tf"
    label "/tf"
    lp "906.06,101.95"
    graphics [
      Line [
        point [ x 736.04 y 102.94 ]
        point [ x 754.74 y 99.986 ]
        point [ x 774.57 y 96.985 ]
        point [ x 793.06 y 94.448 ]
        point [ x 893.23 y 80.699 ]
        point [ x 921.23 y 93.977 ]
        point [ x 1019.1 y 68.448 ]
        point [ x 1030.6 y 65.438 ]
        point [ x 1042.7 y 61.03 ]
        point [ x 1053.7 y 56.442 ]
        point [ x 1063.2 y 52.395 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/tf"
    ]
  ]
  edge [
    id 11
    source 1
    target 4
    URL "topic_3A__tf"
    label "/tf"
    lp "1096.6,128.95"
    graphics [
      Line [
        point [ x 772.28 y 121.52 ]
        point [ x 894.27 y 125.22 ]
        point [ x 1090.8 y 126.69 ]
        point [ x 1259.1 y 107.45 ]
        point [ x 1262.4 y 107.08 ]
        point [ x 1265.6 y 106.63 ]
        point [ x 1268.9 y 106.11 ]
        point [ x 1278.8 y 104.39 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/tf"
    ]
  ]
  edge [
    id 1
    source 2
    target 3
    URL "topic_3A__leap_motion__visualization_marker_array"
    label "/leap_motion/visualization_marker_array"
    lp "906.06,56.948"
    graphics [
      Line [
        point [ x 719.73 y 52.665 ]
        point [ x 798.82 y 49.252 ]
        point [ x 932.77 y 43.473 ]
        point [ x 1017.6 y 39.814 ]
        point [ x 1027.7 y 39.379 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/leap_motion/visualization_marker_array"
    ]
  ]
  edge [
    id 2
    source 3
    target 3
    URL "topic_3A__tf"
    label "/tf"
    lp "1096.6,79.948"
    graphics [
      Line [
        point [ x 1075.2 y 53.858 ]
        point [ x 1071.2 y 63.536 ]
        point [ x 1078.3 y 72.448 ]
        point [ x 1096.6 y 72.448 ]
        point [ x 1108 y 72.448 ]
        point [ x 1115.1 y 68.966 ]
        point [ x 1117.8 y 63.993 ]
        point [ x 1118 y 53.858 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/tf"
    ]
  ]
  edge [
    id 3
    source 3
    target 4
    URL "topic_3A__place__cancel"
    label "/place/cancel"
    lp "1216.6,95.948"
    graphics [
      Line [
        point [ x 1127.7 y 52.63 ]
        point [ x 1141.5 y 59.401 ]
        point [ x 1158.3 y 66.768 ]
        point [ x 1174.1 y 71.448 ]
        point [ x 1201.3 y 79.475 ]
        point [ x 1232.2 y 84.643 ]
        point [ x 1258.8 y 87.935 ]
        point [ x 1268.7 y 89.108 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/place/cancel"
    ]
  ]
  edge [
    id 14
    source 4
    target 3
    URL "topic_3A__place__feedback"
    label "/place/feedback"
    lp "1216.6,63.948"
    graphics [
      Line [
        point [ x 1300.4 y 76.989 ]
        point [ x 1288.6 y 69.509 ]
        point [ x 1273.7 y 61.271 ]
        point [ x 1259.1 y 56.448 ]
        point [ x 1232.8 y 47.676 ]
        point [ x 1202.8 y 42.692 ]
        point [ x 1175.9 y 39.881 ]
        point [ x 1165.7 y 38.907 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/place/feedback"
    ]
  ]
  edge [
    id 15
    source 4
    target 7
    URL "topic_3A__move_group__fake_controller_joint_states"
    label "/move_group/fake_controller_joint_states"
    lp "1504.8,59.948"
    graphics [
      Line [
        point [ x 1346.1 y 76.505 ]
        point [ x 1358.3 y 67.44 ]
        point [ x 1374.5 y 57.19 ]
        point [ x 1390.8 y 52.448 ]
        point [ x 1472.3 y 28.846 ]
        point [ x 1570.4 y 34.272 ]
        point [ x 1636.7 y 42.61 ]
        point [ x 1646.7 y 43.913 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/move_group/fake_controller_joint_states"
    ]
  ]
  edge [
    id 4
    source 5
    target 2
    URL "topic_3A__leap_motion__leap_filtered"
    label "/leap_motion/leap_filtered"
    lp "442.88,50.948"
    graphics [
      Line [
        point [ x 361.16 y 29.619 ]
        point [ x 418.13 y 34.657 ]
        point [ x 514.56 y 43.186 ]
        point [ x 580.94 y 49.055 ]
        point [ x 590.91 y 49.938 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/leap_motion/leap_filtered"
    ]
  ]
  edge [
    id 5
    source 5
    target 3
    URL "topic_3A__leap_motion__leap_filtered"
    label "/leap_motion/leap_filtered"
    lp "654.22,20.948"
    graphics [
      Line [
        point [ x 361.77 y 21.929 ]
        point [ x 404.19 y 18.991 ]
        point [ x 467.87 y 15.037 ]
        point [ x 523.38 y 13.448 ]
        point [ x 639.64 y 10.119 ]
        point [ x 668.84 y 9.1497 ]
        point [ x 785.06 y 13.448 ]
        point [ x 865.79 y 16.433 ]
        point [ x 958.08 y 23.77 ]
        point [ x 1020.9 y 29.361 ]
        point [ x 1031.1 y 30.277 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/leap_motion/leap_filtered"
    ]
  ]
  edge [
    id 6
    source 6
    target 5
    URL "topic_3A__leap_motion__leap_device"
    label "/leap_motion/leap_device"
    lp "185.09,32.948"
    graphics [
      Line [
        point [ x 106.75 y 25.448 ]
        point [ x 149.35 y 25.448 ]
        point [ x 209.38 y 25.448 ]
        point [ x 253.51 y 25.448 ]
        point [ x 263.52 y 25.448 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/leap_motion/leap_device"
    ]
  ]
  edge [
    id 7
    source 7
    target 3
    URL "topic_3A__joint_states"
    label "/joint_states"
    lp "1325,35.948"
    graphics [
      Line [
        point [ x 1655.7 y 42.009 ]
        point [ x 1643.6 y 39.694 ]
        point [ x 1630.9 y 37.649 ]
        point [ x 1618.8 y 36.448 ]
        point [ x 1461.7 y 20.741 ]
        point [ x 1276.6 y 26.448 ]
        point [ x 1174.7 y 31.673 ]
        point [ x 1164.5 y 32.209 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/joint_states"
    ]
  ]
  edge [
    id 9
    source 7
    target 4
    URL "topic_3A__joint_states"
    label "/joint_states"
    lp "1504.8,97.948"
    graphics [
      Line [
        point [ x 1648 y 67.302 ]
        point [ x 1638.3 y 68.847 ]
        point [ x 1628.3 y 70.291 ]
        point [ x 1618.8 y 71.448 ]
        point [ x 1541.3 y 80.927 ]
        point [ x 1451.8 y 86.938 ]
        point [ x 1392.3 y 90.238 ]
        point [ x 1382.3 y 90.783 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/joint_states"
    ]
  ]
  edge [
    id 8
    source 7
    target 8
    URL "topic_3A__joint_states"
    label "/joint_states"
    lp "1842,62.948"
    graphics [
      Line [
        point [ x 1801.3 y 55.448 ]
        point [ x 1824.3 y 55.448 ]
        point [ x 1849.3 y 55.448 ]
        point [ x 1872.9 y 55.448 ]
        point [ x 1882.9 y 55.448 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/joint_states"
    ]
  ]
  edge [
    id 12
    source 8
    target 3
    URL "topic_3A__tf_static"
    label "/tf_static"
    lp "1504.8,14.948"
    graphics [
      Line [
        point [ x 1902.9 y 43.943 ]
        point [ x 1871.9 y 38.88 ]
        point [ x 1834.6 y 33.051 ]
        point [ x 1801 y 28.448 ]
        point [ x 1525 y -9.3593 ]
        point [ x 1450.9 y -8.0484 ]
        point [ x 1174.1 y 24.448 ]
        point [ x 1171.3 y 24.779 ]
        point [ x 1168.5 y 25.137 ]
        point [ x 1165.6 y 25.515 ]
        point [ x 1155.6 y 26.869 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/tf_static"
    ]
  ]
  edge [
    id 13
    source 8
    target 4
    URL "topic_3A__tf_static"
    label "/tf_static"
    lp "1713.9,122.95"
    graphics [
      Line [
        point [ x 1909.2 y 68.255 ]
        point [ x 1897.8 y 70.441 ]
        point [ x 1886.1 y 72.599 ]
        point [ x 1875 y 74.448 ]
        point [ x 1662.2 y 110 ]
        point [ x 1605 y 135.48 ]
        point [ x 1390.8 y 109.45 ]
        point [ x 1386.7 y 108.94 ]
        point [ x 1382.4 y 108.27 ]
        point [ x 1378.2 y 107.47 ]
        point [ x 1368.3 y 105.44 ]
      ]
      width 1
    ]
    LabelGraphics [
      text "/tf_static"
    ]
  ]
]
