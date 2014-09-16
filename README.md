FusedLinearAcceleration
=======================

![](http://www.kircherelectronics.com/bundles/keweb/css/images/fused_linear_acceleration_phone_graphic.png?raw=true)

Fused Linear Acceleration provides a working code example and Android application that demonstrates how to fuse the acceleration, gyroscope and magnetic sensors to provide a reliable measurement of linear acceleration. Fused Linear Acceleration relies on the gyroscope and magnetic sensors to determine the tilt of the device so the gravity component of the acceleration can be subtracted from the signal. The linear acceleration measurement is not skewed by peroids of actual linear acceleration (i.e, accelerating in a vehicle) unlike a low-pass filter or complementary filter implementation. While this example is implemented with Android/Java, the jist of the algorithm can be applied to almost any hardware/language combination to determine linear acceleration.

Linear Acceleration:

An acceleromter can measure the static gravitation field of earth (like a tilt sensor) or it can measure measure linear acceleration (like accelerating in a vehicle), but it cannot measure both at the same time. When talking about linear acceleration in reference to an acceleration sensor, what we really mean is Linear Acceleration = Measured Acceleration - Gravity. The tricky part is determining what part of the signal is gravity.

The Problem:

It is difficult to sequester the gravity component of the signal from the linear acceleration. Some Android devices implement Sensor.TYPE_LINEAR_ACCELERATION and Sensor.TYPE_GRAVITY which perform the calculations for you. Most of these devices are new and equipped with a gyroscope. If you have and older device and do not have a gyroscope, you are going to face some limitations with Sensor.TYPE_ACCELERATION. Note that the implementations of Sensor.TYPE_LINEAR_ACCELERATION and Sensor.TYPE_GRAVITY tend to be poor and are skewed while the device is under peroids of true linear acceleration.

Sensor Fusions:

Sensor fusions take measurements from multiple sensors and fuse them together to create a better estimation than either sensor could do by itself. The most common type of sensor fusion to determine linear acceleration is an accelerometer and gyroscope, which measures the rotation of the device. If you know the rotation of the device and the acceleration of gravity, you can determine the tilt of the device and subtract that from the measured acceleration to get linear acceleration. However, not all devices have gyroscopes.

Most Android devices are equipped with a magnetic sensor which is also capable of measuring the tilt of the device and does not require the use of a complimentary filter or low-pass filter. This makes using the magnetic and acceleration sensor fusion ideal as it is not subject to being skewed by peroids of linear acceleration (unlike a complimentary filter) and it is extremely responsive (unlike a low-pass filter).

Fused Linear Acceleration uses the magnetic, gyroscope and acceleration sensors in an effort to create the most accurate and reliable estimation of linear acceleration possible.

Features:
* Reliably estimate linear acceleration for most applications
* Plot linear acceleration in real-time
* Log linear acceleration to a .CSV file
* Compare Fused Linear Acceleration to other implementations, especially those in the Android API

Useful Links:

* [Fused Linear Acceleration Homepage](http://www.kircherelectronics.com/fusedlinearacceleration/fusedlinearacceleration)
* [Fused Linear Acceleration Community](http://kircherelectronics.com/forum/viewforum.php?f=10)
* [Download Fused Linear Acceleration from Google Play](https://play.google.com/store/apps/details?id=com.kircherelectronics.fusedlinearacceleration)

Written by [Kircher Electronics](https://www.kircherelectronics.com)




