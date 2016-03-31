# Introduction #

This stack implements algorithm for self-parking Lego NXT robot. Stack uses lejos firmware for control robots and lcp\_proxy for link robot with ROS

<a href='http://www.youtube.com/watch?feature=player_embedded&v=AZlVnxzhT9c' target='_blank'><img src='http://img.youtube.com/vi/AZlVnxzhT9c/0.jpg' width='425' height=344 /></a>

# Details #

Run by follow steps:
  * Run lcp\_proxy
`roslaunch lcp_proxy lcp_proxy.launch`
  * Run self-parking algorithm
`roslaunch nxt_carlike nxt_parking.launch`
  * Public message 'StartP'
`rostopic pub /parking std_msgs/String 'startP'`