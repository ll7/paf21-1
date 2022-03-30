
# Object Detection

## About
In object detection, vehicles and pedestrians are detected by evaluating semantic and depth images. 
This is done by the detection of contours with the corresponding masks on the semantic image. 
For this purpose, the depth image is converted into a local point cloud and the contours can be used
to determine the relative position of the objects. In addition to the detection the objects are 
tracked with the help of an object tracker. The object detection sends as output to the vehicle
control the information of the identifier, the object class and the relative position of each
to each detected object.

## Detection Techiques

**Procedure Sketch:**

1) Detect objects using the segmentation camera
   - Use appropriate masks for vehicles and pedestrians
2) Perform separate clustering for vehicles and pedestrians
   1) Extract the contours in the depth image around the detected objects
   2) Iterate over the contours
      1) Draw a rectangle around the contour
      2) Check if the rectangle is valid (minimum size, center in mask)
      3) Calculate the variance of the contour
         1) Under the limit the whole contour is one cluster
         2) Otherwise, split the contour
         3) Removal of outliers
         4) Using "Jenks natural breaks" optimization (cluster algorithm for 1D)
         5) Splitting the contour into two clusters using the break points
      4) Drawing new rectangles around the clusters
      5) Center of the rectangles as cluster centers
3) Determining the relative distance by converting the depth image into a local point cloud
4) Updating the objects of the object tracker
   1) Detect if object has already been detected in previous frame -> assign same identifier
      - Using Euclidean distance and a limit how far objects can move between frames
   2) For newly detected objects a new identifier is assigned
5) Output of a list of ObjectInfos to the VehicleControl
   - contains for every object: identifier, object class, relative position
