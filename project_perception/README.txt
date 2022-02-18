
Package to implement perception into the drone

We use a trained network on the images published by the drone camera to detect traffic signs in the image.

We use an estimator to estimate the signs pose. (NOT IMPLEMENTED YET)

The information from the network, (bboxes and type of sign) is then passed to the bbox node to visualize the bbox
in rviz and estimated pose is passed to sign_position node to publish transforms. 