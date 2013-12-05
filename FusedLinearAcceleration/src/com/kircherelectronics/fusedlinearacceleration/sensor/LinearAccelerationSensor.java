package com.kircherelectronics.fusedlinearacceleration.sensor;

/*
 * Copyright 2013, Kaleb Kircher - Boki Software, Kircher Electronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Copyright (c) 2012 Paul Lawitzki
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

import java.util.ArrayList;

import android.annotation.TargetApi;
import android.hardware.SensorManager;
import android.os.Build;

import com.kircherelectronics.fusedlinearacceleration.filters.MeanFilter;
import com.kircherelectronics.fusedlinearacceleration.sensor.observer.AccelerationSensorObserver;
import com.kircherelectronics.fusedlinearacceleration.sensor.observer.GravitySensorObserver;
import com.kircherelectronics.fusedlinearacceleration.sensor.observer.LinearAccelerationSensorObserver;
import com.kircherelectronics.fusedlinearacceleration.sensor.observer.GyroscopeSensorObserver;
import com.kircherelectronics.fusedlinearacceleration.sensor.observer.MagneticSensorObserver;

/**
 * OrientationComplementaryFilter attempts to fuse magnetometer, gravity and
 * gyroscope sensors together to produce an accurate measurement of the rotation
 * of the device. The magnetometer and acceleration sensors are used to
 * determine the orientation of the device, but these readings are noisy and are
 * subject to the constraint that the device must not be accelerating. The
 * gyroscope is much more accurate and has a shorter response time, however it
 * experiences drift and has to be compensated periodically to remain accurate.
 * 
 * The gyroscope provides the angular rotation speeds for all three axes. To
 * find the orientation of the device, the rotation speeds must be integrated
 * over time. This can be accomplished by multiplying the angular speeds by the
 * time intervals between sensor updates. The calculation produces the rotation
 * increment. Integrating these values again produces the absolute orientation
 * of the device. Small errors are produced at each iteration causing the gyro
 * to drift away from the true orientation.
 * 
 * To eliminate both the drift and noise from the orientation, the gyro
 * measurements are applied only for orientation changes in short time
 * intervals. The magnetometer/acceleration fusion is used for long time
 * intervals. This is equivalent to low-pass filtering of the accelerometer and
 * magnetic field sensor signals and high-pass filtering of the gyroscope
 * signals.
 * 
 * Note: The fusion algorithm itself was written by Paul @
 * http://www.thousand-thoughts.com/2012/03/android-sensor-fusion-tutorial/ and
 * taken from his SensorFusion1.zip project. J.W. Alexandar Qiu has credit for
 * the transitions between 179° <–> -179° fix. I have optimized some of the code
 * and made it slightly easier to follow and read. I have also added the
 * trigonometric calculations required to calculated the gravity components of
 * the acceleration sensor and I have also changed the
 * SensorManager.getRotationMatrix() to use the gravity sensor instead of the
 * acceleration sensor.
 * 
 * @author Kaleb
 * @version %I%, %G%
 * @see http://web.mit.edu/scolton/www/filter.pdf
 * @see http 
 *      ://developer.android.com/reference/android/hardware/SensorEvent.html#
 *      values
 * @see http://www.thousand-thoughts.com/2012/03/android-sensor-fusion-tutorial/
 * 
 */
@TargetApi(Build.VERSION_CODES.GINGERBREAD)
public class LinearAccelerationSensor implements GyroscopeSensorObserver,
		AccelerationSensorObserver, MagneticSensorObserver,
		GravitySensorObserver
{
	private static final String tag = LinearAccelerationSensor.class
			.getSimpleName();

	public static final float FILTER_COEFFICIENT = 0.5f;

	public static final float EPSILON = 0.000000001f;

	// private static final float NS2S = 1.0f / 10000.0f;
	// Nano-second to second conversion
	private static final float NS2S = 1.0f / 1000000000.0f;

	// list to keep track of the observers
	private ArrayList<LinearAccelerationSensorObserver> observersAngularVelocity;

	private boolean hasOrientation = false;

	private float dT = 0;

	private float omegaMagnitude = 0;

	private float thetaOverTwo = 0;
	private float sinThetaOverTwo = 0;
	private float cosThetaOverTwo = 0;

	// The gravity components of the acceleration signal.
	private float[] components = new float[3];

	private float[] linearAcceleration = new float[]
	{ 0, 0, 0 };

	private float[] gravity = new float[]
	{ 0, 0, 0 };

	// angular speeds from gyro
	private float[] gyroscope = new float[3];

	// rotation matrix from gyro data
	private float[] gyroMatrix = new float[9];

	// orientation angles from gyro matrix
	private float[] gyroOrientation = new float[3];

	// magnetic field vector
	private float[] magnetic = new float[3];

	// accelerometer vector
	private float[] acceleration = new float[3];

	// orientation angles from accel and magnet
	private float[] orientation = new float[3];

	// final orientation angles from sensor fusion
	private float[] fusedOrientation = new float[3];

	// accelerometer and magnetometer based rotation matrix
	private float[] rotationMatrix = new float[9];

	private float[] absoluteFrameOrientation = new float[3];

	// copy the new gyro values into the gyro array
	// convert the raw gyro data into a rotation vector
	private float[] deltaVector = new float[4];

	// convert rotation vector into rotation matrix
	private float[] deltaMatrix = new float[9];

	private long timeStamp;

	private boolean initState = false;

	private MeanFilter meanFilterGravity;
	private MeanFilter meanFilterMagnetic;
	private MeanFilter meanFilterAcceleration;
	private MeanFilter meanFilterLinearAcceleration;

	/**
	 * Initialize a singleton instance.
	 * 
	 * @param gravitySubject
	 *            the gravity subject.
	 * @param gyroscopeSubject
	 *            the gyroscope subject.
	 * @param magneticSubject
	 *            the magnetic subject.
	 */
	public LinearAccelerationSensor()
	{
		super();

		observersAngularVelocity = new ArrayList<LinearAccelerationSensorObserver>();

		meanFilterGravity = new MeanFilter();
		meanFilterGravity.setWindowSize(10);

		meanFilterMagnetic = new MeanFilter();
		meanFilterMagnetic.setWindowSize(10);

		meanFilterAcceleration = new MeanFilter();
		meanFilterAcceleration.setWindowSize(10);

		meanFilterLinearAcceleration = new MeanFilter();
		meanFilterLinearAcceleration.setWindowSize(10);

		gyroOrientation[0] = 0.0f;
		gyroOrientation[1] = 0.0f;
		gyroOrientation[2] = 0.0f;

		// Initialize gyroMatrix with identity matrix
		gyroMatrix[0] = 1.0f;
		gyroMatrix[1] = 0.0f;
		gyroMatrix[2] = 0.0f;
		gyroMatrix[3] = 0.0f;
		gyroMatrix[4] = 1.0f;
		gyroMatrix[5] = 0.0f;
		gyroMatrix[6] = 0.0f;
		gyroMatrix[7] = 0.0f;
		gyroMatrix[8] = 1.0f;

	}

	public void notifyObservers()
	{
		for (LinearAccelerationSensorObserver g : observersAngularVelocity)
		{
			g.onLinearAccelerationSensorChanged(linearAcceleration, timeStamp);
		}
	}

	public void registerLinearAccelerationObserver(
			LinearAccelerationSensorObserver g)
	{
		observersAngularVelocity.add(g);
	}

	public void removeLinearAccelerationObserver(
			LinearAccelerationSensorObserver g)
	{
		int i = observersAngularVelocity.indexOf(g);
		if (i >= 0)
		{
			observersAngularVelocity.remove(i);
		}
	}

	/**
	 * Calculates orientation angles from accelerometer and magnetometer output.
	 */
	private void calculateOrientation()
	{
		if (SensorManager.getRotationMatrix(rotationMatrix, null, gravity,
				magnetic))
		{
			SensorManager.getOrientation(rotationMatrix, orientation);

			hasOrientation = true;
		}
	}

	/**
	 * Get the rotation matrix from the current orientation. Android Sensor
	 * Manager does not provide a method to transform the orientation into a
	 * rotation matrix, only the orientation from a rotation matrix. The basic
	 * rotations can be found in Wikipedia with the caveat that the rotations
	 * are *transposed* relative to what is required for this method.
	 * 
	 * @param The
	 *            device orientation.
	 * @return The rotation matrix from the orientation.
	 * 
	 * @see http://en.wikipedia.org/wiki/Rotation_matrix
	 */
	private float[] getRotationMatrixFromOrientation(float[] orientation)
	{
		float[] xM = new float[9];
		float[] yM = new float[9];
		float[] zM = new float[9];

		float sinX = (float) Math.sin(orientation[1]);
		float cosX = (float) Math.cos(orientation[1]);
		float sinY = (float) Math.sin(orientation[2]);
		float cosY = (float) Math.cos(orientation[2]);
		float sinZ = (float) Math.sin(orientation[0]);
		float cosZ = (float) Math.cos(orientation[0]);

		// rotation about x-axis (pitch)
		xM[0] = 1.0f;
		xM[1] = 0.0f;
		xM[2] = 0.0f;
		xM[3] = 0.0f;
		xM[4] = cosX;
		xM[5] = sinX;
		xM[6] = 0.0f;
		xM[7] = -sinX;
		xM[8] = cosX;

		// rotation about y-axis (roll)
		yM[0] = cosY;
		yM[1] = 0.0f;
		yM[2] = sinY;
		yM[3] = 0.0f;
		yM[4] = 1.0f;
		yM[5] = 0.0f;
		yM[6] = -sinY;
		yM[7] = 0.0f;
		yM[8] = cosY;

		// rotation about z-axis (azimuth)
		zM[0] = cosZ;
		zM[1] = sinZ;
		zM[2] = 0.0f;
		zM[3] = -sinZ;
		zM[4] = cosZ;
		zM[5] = 0.0f;
		zM[6] = 0.0f;
		zM[7] = 0.0f;
		zM[8] = 1.0f;

		// Build the composite rotation... rotation order is y, x, z (roll,
		// pitch, azimuth)
		float[] resultMatrix = matrixMultiplication(xM, yM);
		resultMatrix = matrixMultiplication(zM, resultMatrix);
		return resultMatrix;
	}

	/**
	 * Calculates a rotation vector from the gyroscope angular speed values.
	 * 
	 * @param gyroValues
	 * @param deltaRotationVector
	 * @param timeFactor
	 * @see http://developer.android
	 *      .com/reference/android/hardware/SensorEvent.html#values
	 */
	private void getRotationVectorFromGyro(float timeFactor)
	{

		// Calculate the angular speed of the sample
		omegaMagnitude = (float) Math.sqrt(Math.pow(gyroscope[0], 2)
				+ Math.pow(gyroscope[1], 2) + Math.pow(gyroscope[2], 2));

		// Normalize the rotation vector if it's big enough to get the axis
		if (omegaMagnitude > EPSILON)
		{
			gyroscope[0] /= omegaMagnitude;
			gyroscope[1] /= omegaMagnitude;
			gyroscope[2] /= omegaMagnitude;
		}

		// Integrate around this axis with the angular speed by the timestep
		// in order to get a delta rotation from this sample over the timestep
		// We will convert this axis-angle representation of the delta rotation
		// into a quaternion before turning it into the rotation matrix.
		thetaOverTwo = omegaMagnitude * timeFactor;
		sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
		cosThetaOverTwo = (float) Math.cos(thetaOverTwo);

		deltaVector[0] = sinThetaOverTwo * gyroscope[0];
		deltaVector[1] = sinThetaOverTwo * gyroscope[1];
		deltaVector[2] = sinThetaOverTwo * gyroscope[2];
		deltaVector[3] = cosThetaOverTwo;
	}

	/**
	 * Multiply A by B.
	 * 
	 * @param A
	 * @param B
	 * @return A*B
	 */
	private float[] matrixMultiplication(float[] A, float[] B)
	{
		float[] result = new float[9];

		result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
		result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
		result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

		result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
		result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
		result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

		result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
		result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
		result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

		return result;
	}

	@Override
	public void onMagneticSensorChanged(float[] magnetic, long timeStamp)
	{
		// Get a local copy of the raw magnetic values from the device sensor.
		System.arraycopy(magnetic, 0, this.magnetic, 0, magnetic.length);

		this.magnetic = meanFilterMagnetic.filterFloat(this.magnetic);
	}

	@Override
	public void onAccelerationSensorChanged(float[] acceleration, long timeStamp)
	{
		// Get a local copy of the raw magnetic values from the device sensor.
		System.arraycopy(acceleration, 0, this.acceleration, 0,
				acceleration.length);

		this.acceleration = meanFilterAcceleration
				.filterFloat(this.acceleration);
	}

	@Override
	public void onGravitySensorChanged(float[] gravity, long timeStamp)
	{
		// Get a local copy of the raw magnetic values from the device sensor.
		System.arraycopy(gravity, 0, this.gravity, 0, gravity.length);

		this.gravity = meanFilterGravity.filterFloat(this.gravity);

		calculateOrientation();
	}

	@Override
	public void onGyroscopeSensorChanged(float[] gyroscope, long timeStamp)
	{
		// don't start until first accelerometer/magnetometer orientation has
		// been acquired
		if (!hasOrientation)
		{
			return;
		}

		// Initialization of the gyroscope based rotation matrix
		if (!initState)
		{
			gyroMatrix = matrixMultiplication(gyroMatrix, rotationMatrix);
			initState = true;
		}

		if (this.timeStamp != 0)
		{
			dT = (timeStamp - this.timeStamp) * NS2S;

			System.arraycopy(gyroscope, 0, this.gyroscope, 0, 3);
			getRotationVectorFromGyro(dT / 2.0f);
		}

		// measurement done, save current time for next interval
		this.timeStamp = timeStamp;

		// Get the rotation matrix from the gyroscope
		SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

		// Apply the new rotation interval on the gyroscope based rotation
		// matrix to form a composite rotation matrix. The product of two
		// rotation matricies is a rotation matrix...
		// Multiplication of rotation matrices corresponds to composition of
		// rotations... Which in this case are the rotation matrix from the
		// fused orientation and the rotation matrix from the current gyroscope
		// outputs.
		gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

		// Get the gyroscope based orientation from the composite rotation
		// matrix. This orientation will be fused via complementary filter with
		// the orientation from the acceleration sensor and magnetic sensor.
		SensorManager.getOrientation(gyroMatrix, gyroOrientation);

		calculateFusedOrientation();
	}

	/**
	 * Calculate the fused orientation.
	 */
	private void calculateFusedOrientation()
	{
		float oneMinusCoeff = (1.0f - FILTER_COEFFICIENT);

		/*
		 * Fix for 179° <--> -179° transition problem: Check whether one of the
		 * two orientation angles (gyro or accMag) is negative while the other
		 * one is positive. If so, add 360° (2 * math.PI) to the negative value,
		 * perform the sensor fusion, and remove the 360° from the result if it
		 * is greater than 180°. This stabilizes the output in
		 * positive-to-negative-transition cases.
		 */

		// azimuth
		if (gyroOrientation[0] < -0.5 * Math.PI && orientation[0] > 0.0)
		{
			fusedOrientation[0] = (float) (FILTER_COEFFICIENT
					* (gyroOrientation[0] + 2.0 * Math.PI) + oneMinusCoeff
					* orientation[0]);
			fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else if (orientation[0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0)
		{
			fusedOrientation[0] = (float) (FILTER_COEFFICIENT
					* gyroOrientation[0] + oneMinusCoeff
					* (orientation[0] + 2.0 * Math.PI));
			fusedOrientation[0] -= (fusedOrientation[0] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else
		{
			fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0]
					+ oneMinusCoeff * orientation[0];
		}

		// pitch
		if (gyroOrientation[1] < -0.5 * Math.PI && orientation[1] > 0.0)
		{
			fusedOrientation[1] = (float) (FILTER_COEFFICIENT
					* (gyroOrientation[1] + 2.0 * Math.PI) + oneMinusCoeff
					* orientation[1]);
			fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else if (orientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0)
		{
			fusedOrientation[1] = (float) (FILTER_COEFFICIENT
					* gyroOrientation[1] + oneMinusCoeff
					* (orientation[1] + 2.0 * Math.PI));
			fusedOrientation[1] -= (fusedOrientation[1] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else
		{
			fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1]
					+ oneMinusCoeff * orientation[1];
		}

		// roll
		if (gyroOrientation[2] < -0.5 * Math.PI && orientation[2] > 0.0)
		{
			fusedOrientation[2] = (float) (FILTER_COEFFICIENT
					* (gyroOrientation[2] + 2.0 * Math.PI) + oneMinusCoeff
					* orientation[2]);
			fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else if (orientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0)
		{
			fusedOrientation[2] = (float) (FILTER_COEFFICIENT
					* gyroOrientation[2] + oneMinusCoeff
					* (orientation[2] + 2.0 * Math.PI));
			fusedOrientation[2] -= (fusedOrientation[2] > Math.PI) ? 2.0 * Math.PI
					: 0;
		}
		else
		{
			fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2]
					+ oneMinusCoeff * orientation[2];
		}

		// overwrite gyro matrix and orientation with fused orientation
		// to comensate gyro drift
		gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);

		System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);

		calculateLinearAcceleration();

		notifyObservers();
	}

	private void calculateLinearAcceleration()
	{
		System.arraycopy(gyroOrientation, 0, absoluteFrameOrientation, 0, 3);

		// values[0]: azimuth, rotation around the Z axis.
		// values[1]: pitch, rotation around the X axis.
		// values[2]: roll, rotation around the Y axis.

		// Find the gravity component of the X-axis
		// = g*-cos(pitch)*sin(roll);
		components[0] = (float) (SensorManager.GRAVITY_EARTH
				* -Math.cos(absoluteFrameOrientation[1]) * Math
				.sin(absoluteFrameOrientation[2]));

		// Find the gravity component of the Y-axis
		// = g*-sin(pitch);
		components[1] = (float) (SensorManager.GRAVITY_EARTH * -Math
				.sin(absoluteFrameOrientation[1]));

		// Find the gravity component of the Z-axis
		// = g*cos(pitch)*cos(roll);
		components[2] = (float) (SensorManager.GRAVITY_EARTH
				* Math.cos(absoluteFrameOrientation[1]) * Math
				.cos(absoluteFrameOrientation[2]));

		// Subtract the gravity component of the signal
		// from the input acceleration signal to get the
		// tilt compensated output.
		linearAcceleration[0] = (this.acceleration[0] - components[0]);
		linearAcceleration[1] = (this.acceleration[1] - components[1]);
		linearAcceleration[2] = (this.acceleration[2] - components[2]);

		this.linearAcceleration = meanFilterLinearAcceleration
				.filterFloat(this.linearAcceleration);
	}
}
