package com.kircherelectronics.fusedgyroscopeexplorer.sensor.observer;

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
 * An angular velocity sensor observer interface. Classes that need to observe
 * the angular velocity sensor for updates should do so with this interface.
 * 
 * @author Kaleb
 * @version %I%, %G%
 */
public interface FusedGyroscopeSensorObserver
{
	/**
	 * Notify observers when new angular velocity measurements are available.
	 * 
	 * values[0]: Rotation in radians around the x-axis 
	 * values[1]: Rotation in radians around the y-axis 
	 * values[2]: Rotation in radians around the z-axis
	 * 
	 * @param angularVelocity
	 *            the angular velocity of the device (x,y,z).
	 * @param timeStamp
	 *            the time stamp of the measurement.
	 */
	public void onAngularVelocitySensorChanged(float[] angularVelocity,
			long timeStamp);
}
