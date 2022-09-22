# Fall-Detection

This is Python code for building machine learning model for detect if a package is thrown or slammed by the courier from time-series sensor data.
There are 11 sensor data obtained from IoT device that consist of altitude, gyroscope, and magnetic with axis x,y, and z in each sensor.
The data is preprocessed using mean and standard deviation seperately. Rolling window is also used to calculate the data with 1 window contains 5 data and shifted by 3 data.
As results from feature importance, only 10 of 22 features will be used as an input to the model which are standard deviation of aZ, aSqrt, gX, gY, and gZ and mean of gX, gY, gZ, aX, and aSqrt.
The algorithm used for this model is RandomForestClassifier from sklearn module. To find best parameters for the model we used Bayesian Search Cross Validation to optimize hyperparameters tuning.
