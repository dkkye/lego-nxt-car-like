
package org.ros.lejos.android;


import android.os.Bundle;
import android.hardware.Camera;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.views.RosCameraPreviewView;
import android.widget.Button;
import android.view.View;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.lejos.android.R;

/**
 * @author alexbuyval@gmail.com (Alex Buyval)
 */


public class MainActivity extends RosActivity {

	private int cameraId;
	private RosCameraPreviewView preview;
	private NXTNode nxt_node;

	public MainActivity() {
		super("Lejos ROS Android", "Lejos ROS Android");
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
		nxt_node = new NXTNode();
		nodeMainExecutor.execute(preview, nodeConfiguration);
		nodeMainExecutor.execute(nxt_node, nodeConfiguration);
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
