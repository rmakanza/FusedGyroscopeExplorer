package com.kircherelectronics.fusedgyroscopeexplorer.sensor;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.kircherelectronics.fusedgyroscopeexplorer.sensor.observer.GyroscopeSensorObserver;

/*
 * Fused Gyroscope Explorer
 * Copyright (C) 2013, Kaleb Kircher - Kircher Engineering, LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Gyroscope Sensor is a subject in an Observer Pattern for classes that need to
 * be provided with rotation measurements. Gyroscope Sensor implements
 * Sensor.TYPE_GYROSCOPE and provides methods for managing SensorEvents and
 * rotations.
 * 
 * Note that not all devices support Sensor.TYPE_GYROSCOPE. If
 * Sensor.TYPE_GYROSCOPE is supported, it can not be guaranteed that the
 * gyroscope sensors drift has been compensated for. Therefore, appropriate
 * algorithms should be applied to the sensor measurements to ensure stability
 * across devices.
 * 
 * @author Kaleb
 * @version %I%, %G%
 */
public class GyroscopeSensor implements SensorEventListener
{
	/*
	 * Developer Note: Quaternions are used for the internal representations of
	 * the rotations which prevents the polar anomalies associated with Gimbal
	 * lock when using Euler angles for the rotations.
	 */

	private static final String tag = GyroscopeSensor.class.getSimpleName();
	
	// Keep track of observers.
	private ArrayList<GyroscopeSensorObserver> observersGyroscope;

	// Keep track of the application mode. Vehicle Mode occurs when the device
	// is in the Landscape orientation and the sensors are rotated to face the
	// -Z-Axis (along the axis of the camera).
	private boolean vehicleMode = false;

	// We need the Context to register for Sensor Events.
	private Context context;

	// Keep a local copy of the rotation values that are copied from the
	// sensor event.
	private float[] gyroscope = new float[3];

	// The time stamp of the most recent Sensor Event.
	private long timeStamp = 0;

	// Quaternion data structures to rotate a matrix from the absolute Android
	// orientation to the orientation that the device is actually in. This is
	// needed because the the device sensors orientation is fixed in hardware.
	// Also remember the many algorithms require a NED orientation which is not
	// the same as the absolute Android orientation. Do not confuse this
	// rotation with a rotation into absolute earth frame!
	private Rotation yQuaternion;
	private Rotation xQuaternion;
	private Rotation rotationQuaternion;

	// We need the SensorManager to register for Sensor Events.
	private SensorManager sensorManager;

	// The vectors that will be rotated when the application is in Vehicle Mode.
	private Vector3D vIn;
	private Vector3D vOut;

	/**
	 * Initialize the state.
	 * 
	 * @param context
	 *            the Activities context.
	 */
	public GyroscopeSensor(Context context)
	{
		super();

		this.context = context;

		initQuaternionRotations();

		observersGyroscope = new ArrayList<GyroscopeSensorObserver>();

		sensorManager = (SensorManager) this.context
				.getSystemService(Context.SENSOR_SERVICE);
	}

	/**
	 * Register for Sensor.TYPE_GYROSCOPE measurements.
	 * 
	 * @param observer
	 *            The observer to be registered.
	 */
	public void registerGyroscopeObserver(GyroscopeSensorObserver observer)
	{
		if (observersGyroscope.size() == 0)
		{
			boolean enabled = sensorManager.registerListener(this,
					sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
					SensorManager.SENSOR_DELAY_FASTEST);
			
			if(!enabled)
			{
				showGyroscopeNotAvailableAlert();
			}
		}
		
		// Only register the observer if it is not already registered.
		int i = observersGyroscope.indexOf(observer);
		if (i == -1)
		{
			observersGyroscope.add(observer);
		}
	}

	/**
	 * Remove Sensor.TYPE_GYROSCOPE measurements.
	 * 
	 * @param observer
	 *            The observer to be removed.
	 */
	public void removeGyroscopeObserver(GyroscopeSensorObserver observer)
	{
		int i = observersGyroscope.indexOf(observer);
		if (i >= 0)
		{
			observersGyroscope.remove(i);
		}

		// If there are no observers, then don't listen for Sensor Events.
		if (observersGyroscope.size() == 0)
		{
			sensorManager.unregisterListener(this);
		}
	}


	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy)
	{
		// Do nothing.
	}

	@Override
	public void onSensorChanged(SensorEvent event)
	{
		if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
		{
			System.arraycopy(event.values, 0, this.gyroscope, 0,
					event.values.length);

			this.timeStamp = event.timestamp;

			if (vehicleMode)
			{
				this.gyroscope = quaternionToDeviceVehicleMode(this.gyroscope);
			}

			notifyGyroscopeObserver();
		}
	}

	/**
	 * Vehicle mode occurs when the device is put into the landscape
	 * orientation. On Android phones, the positive Y-Axis of the sensors faces
	 * towards the top of the device. In vehicle mode, we want the sensors to
	 * face the negative Z-Axis so it is aligned with the camera of the device.
	 * 
	 * @param vehicleMode
	 *            true if in vehicle mode.
	 */
	public void setVehicleMode(boolean vehicleMode)
	{
		this.vehicleMode = vehicleMode;
	}

	/**
	 * To avoid anomalies at the poles with Euler angles and Gimbal lock,
	 * quaternions are used instead.
	 */
	private void initQuaternionRotations()
	{
		// Rotate by 90 degrees or pi/2 radians.
		double rotation = Math.PI / 2;

		// Create the rotation around the x-axis
		Vector3D xV = new Vector3D(1, 0, 0);
		xQuaternion = new Rotation(xV, rotation);

		// Create the rotation around the y-axis
		Vector3D yV = new Vector3D(0, 1, 0);
		yQuaternion = new Rotation(yV, -rotation);

		// Create the composite rotation.
		rotationQuaternion = yQuaternion.applyTo(xQuaternion);
	}
	
	/**
	 * Notify observers with new measurements.
	 */
	private void notifyGyroscopeObserver()
	{
		for (GyroscopeSensorObserver a : observersGyroscope)
		{
			a.onGyroscopeSensorChanged(this.gyroscope, this.timeStamp);
		}
	}

	/**
	 * Orient the measurements from the absolute Android device rotation into
	 * the current device orientation. Note that the rotation is different based
	 * on the current rotation of the device relative to the absolute Android
	 * rotation. Do not confuse this with a rotation into absolute earth frame,
	 * or the NED orientation that the algorithm assumes.
	 * 
	 * @param measurements
	 *            the measurements referenced to the absolute Android
	 *            orientation.
	 * @return the measurements referenced to the current device rotation.
	 * 
	 * @see http 
	 *      ://developer.android.com/reference/android/hardware/SensorEvent.html
	 *      #values
	 */
	private float[] quaternionToDeviceVehicleMode(float[] matrix)
	{

		vIn = new Vector3D(matrix[0], matrix[1], matrix[2]);
		vOut = rotationQuaternion.applyTo(vIn);

		float[] rotation =
		{ (float) vOut.getX(), (float) vOut.getY(), (float) vOut.getZ() };

		return rotation;
	}
	
	
	private void showGyroscopeNotAvailableAlert()
	{
		AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(context);

		// set title
		alertDialogBuilder.setTitle("Gyroscope Not Available");

		// set dialog message
		alertDialogBuilder
				.setMessage(
						"Your device is not equipped with a gyroscope or it is not responding...")
				.setCancelable(false)
				.setNegativeButton("I'll look around...",
						new DialogInterface.OnClickListener()
						{
							public void onClick(DialogInterface dialog, int id)
							{
								// if this button is clicked, just close
								// the dialog box and do nothing
								dialog.cancel();
							}
						});

		// create alert dialog
		AlertDialog alertDialog = alertDialogBuilder.create();

		// show it
		alertDialog.show();
	}
}
