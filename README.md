# SeExplorer

breadth first search begins from the current pose of robot and find the frontier betweens the unknowns and free space

then,we pick the centroid of the frontiers

![robot in costmap](http://upload-images.jianshu.io/upload_images/635586-078e70d6eee96cef.png?imageMogr2/auto-orient/strip%7CimageView2/2/w/575)

the arrow is pointing to the current of robot , the red region means unknown space and the blue are the frontiers that we found 

next ,we publish the centroid of the frontiers as the goal to the globalplanner
