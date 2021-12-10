# How it detects & tracking

1.1 Read raw image which is distorted by camera len.

1.2 Convert distorted to undistorted image by applying camera calibration matrix.

2.1 Do perspective warped (extrinsic).

3.1 Apply 2Dfilter to remove noise or smoothing the image.

3.2 Detect white & yellow lanes.

3.5 Do sliding windows and histogram.

3.4 Do polynomial fitting for both lanes.

3.6 Estimated middle lane value to send to the PD controller.
