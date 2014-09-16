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
 * An magnetic sensor observer interface. Classes that need to observe the
 * magnetic sensor for updates should do so with this interface.
 * 
 * @author Kaleb
 * @version %I%, %G%
 */
public interface MagneticSensorObserver
{
	/**
	 * Notify observers when new magnetic measurements are available.
	 * 
	 * @param magnetic
	 *            the magnetic measurements (x, y, z).
	 * @param timeStamp
	 *            the time stamp of the measurement.
	 */
	public void onMagneticSensorChanged(float[] magnetic, long timeStamp);
}
