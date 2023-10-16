# Extended-Kalman-Filter
###State Estimation of a Nano Quadrotor

### OBJECTIVE

To implement an Extended Kalman Filter (EKF) for state estimation by considering the acceleration and angular velocity from an onboard IMU sensor as control inputs and measurement from the pose and orientation provided by the Vicon motion capture system as ground truth.

### METHODOLOGY

In the KalmanFilt_Part1.m file, data is first loaded into the sampledData struct, sampledVicon and sampledTime files. Data from sampledVicon is then extracted which contains all the measurements that are required for the update step. We then run a for loop to update the values of the angular velocity and acceleration of the IMU from the sampledData file. We also calculate the sampling time dt in this for loop by calculating the difference between the current time and the previous time step. This data is necessary for the prediction step.

The goal of the prediction step is to calculate the estimated mean and co-variance which are given by the following formulas-
![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/5a9f4a7f-1954-413d-9873-47932018e17e)

In this calculation, we assume that there is no noise in the gyroscope and accelerometer readings from the IMU as well as no noise in the bias of the accelerometer and gyroscope.

The state matrix and process model for the prediction step are given by-
![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/ac8b410e-99f7-4016-8975-fb2ea4efdda9)

![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/a91c2efc-e652-49ce-980f-3a3d9a7b4c92)

Once we have the x_dot matrix, we can calculate the estimated mean and estimated covariance by using the parameters shown below and then plugging them into equations 1 and 2 mentioned above.

![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/c9d3b948-9ae3-417d-866f-d12e39bcdbd1)

The x, x_dot, At and Ut matrices have been calculated by using symbolic variables in a live script. The values for these symbolic variables have been obtained from the uPrev, angVel, and acc files.

Ft discretizes the values of At by multiplying it with the sampling time. Vt is the same as Ut which is the discretized noise. This matrix is the jacobian that has been obtained by partially differentiating x_dot with respect to the noise of the gyroscope, accelerometer, and their biases.

After calculating the estimated mean and covariance in the prediction step, we have the data required to calculate our present mean and covariance in the update step for the Extended Kalman Filter. 
![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/f736bb16-42a6-4bd3-9b1c-9a54753c49d7)

Kt is given by the following formula-
![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/fa4aa49c-b823-4705-8e65-de018b083547)

Here, Ct and Wt are linearized by calculating the Jacobian by partially differentiating z with respect to x and v (noise in the update step) respectively-

To implement the update step, we need to first calculate the value of z which is obtained from the observation model is given by-
![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/596c0b78-3e64-4629-aeb2-71f1e5701f86)

Since we are told that the measurement step is updated only by taking values of the position and orientation from the Vicon, we consider only the first two rows of the C matrix shown above to update the measurement. Lastly, we calculate the current mean and covariance as per equations 3 and 4 mentioned above.

This completes the implementation of the Extended Kalman Filter.

### RESULTS

The plots obtained by running 3 different datasets can be seen below. It can be observed that the actual values vs the predicted values are almost the same for the position and orientation in X, Y, and Z. However, there is a slight mismatch in the velocity graphs. This is because we only consider the position and orientation while excluding the velocity in the observation model to calculate the z matrix.

![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/5807389a-6662-42cb-9416-a2eb70c170dc)

![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/d0641db8-b643-4f3d-9ee8-d68a844950e0)

![image](https://github.com/RajatKirloskar/Extended-Kalman-Filter/assets/108690286/65f90ac5-aa07-42eb-84b3-4caa67b38743)





