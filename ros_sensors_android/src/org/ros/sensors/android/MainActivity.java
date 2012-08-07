
package org.ros.sensors.android;


import android.os.Bundle;
import android.hardware.Camera;
import android.hardware.SensorManager;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.views.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.sensors.android.R;

/**
 * @author alexbuyval@gmail.com (Alex Buyval)
 */


public class MainActivity extends RosActivity {

	private int cameraId;
	private RosCameraPreviewView preview;
	private SensorsService SensorsNode;
	private SensorManager mSensorManager;

	public MainActivity() {
		super("ROS Sensors Android", "ROS Sensors Android");
	}

	@SuppressWarnings("unchecked")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		preview = (RosCameraPreviewView) findViewById(R.id.camera_preview);
		setupControl(this);
	}

	@Override
	protected void init(NodeMainExecutor nodeMainExecutor) {
		cameraId = 0;
	    preview.setCamera(Camera.open(cameraId));
		NodeConfiguration nodeConfiguration =  NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
		nodeConfiguration.setMasterUri(getMasterUri());
		nodeMainExecutor.execute(preview, nodeConfiguration);
		mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);
		SensorsNode = new SensorsService(mSensorManager);
		nodeMainExecutor.execute(SensorsNode, nodeConfiguration);

	}

	@Override
	protected void onResume() {
		super.onResume();
		
	}

	private void setupControl(final MainActivity mainActivity) {


	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
	}
}
