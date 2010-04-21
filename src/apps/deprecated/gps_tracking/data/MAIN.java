
/**
 * 
 * 
 * @author Hui Dai
 * 
 */
public class MAIN {

	public static void main(String[] args) {

		NetworkTopology topology = new NetworkTopology();
		SensorTopo sensorTopo = new SensorTopo();
		NetworkTopologyDisplay display = new NetworkTopologyDisplay(topology);
		SensorTopoDisplay sensorDisplay = new SensorTopoDisplay(sensorTopo);
		

		MainFrame mainFrame = new MainFrame(display, "Path Tracking 1.0");

		MainFrame sensorFrame = new MainFrame(sensorDisplay, "Sensor Data Displaying");

		topology.setObserver(display);
		sensorTopo.setObserver(sensorDisplay);

		if (args.length != 1) {
			System.err.println("usage: java SerialPortListener [port]");
			System.exit(-1);
		}

		System.out.println("\nSerialPortListenern started");
		SerialPortListener reader = new SerialPortListener(args[0], topology, sensorTopo);
		try {
			reader.printAllPorts();
			reader.open();
		} catch (Exception e) {
			e.printStackTrace();
		}

		try {
			reader.read();
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}
}
		
