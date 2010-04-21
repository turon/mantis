
import java.util.*;
import java.io.*;
import javax.comm.*;


public class SerialPortListener {

	//===   CONSTANTS   =========================================================

	private static final String CLASS_NAME = "SerialPortListener";
	private static final String VERSION = "v0.1";
	private static final int MSG_SIZE = 512; // 4 byte longitude, 4-byte latitude and 4 byte latitude, 1 byte start and 1 byte end

    private static int sensorCount = 0;
	//===   PRIVATE VARIABLES   =================================================

	CommPortIdentifier portId;
	SerialPort port;
        NetworkTopology topology;
        SensorTopo sensorTopo;
	String portName;
	InputStream in;
	OutputStream out;

    public SerialPortListener(String portName, NetworkTopology topology, SensorTopo sensorTopo) {
		this.portName = portName;
    		this.topology = topology;
    		this.sensorTopo = sensorTopo;
	}



	//===========================================================================

	public void open()
		throws NoSuchPortException, PortInUseException, IOException, UnsupportedCommOperationException {
		portId = CommPortIdentifier.getPortIdentifier(portName);
		port = (SerialPort) portId.open(CLASS_NAME, 0);
		in = port.getInputStream();
		out = port.getOutputStream();

		//port.setFlowControlMode(SerialPort.FLOWCONTROL_RTSCTS_IN);
		port.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
		port.disableReceiveFraming();
		printPortStatus();
		port.setSerialPortParams(
			19200,
			SerialPort.DATABITS_8,
			SerialPort.STOPBITS_1,
			SerialPort.PARITY_NONE);
		printPortStatus();
	}

	private void printPortStatus() {
		System.out.println("baud rate: " + port.getBaudRate());
		System.out.println("data bits: " + port.getDataBits());
		System.out.println("stop bits: " + port.getStopBits());
		System.out.println("parity:    " + port.getParity());
	}

	//===========================================================================

	/*
	 *  Get an enumeration of all of the comm ports 
	 *  on the machine
	 */

	public void printAllPorts() {
		Enumeration ports = CommPortIdentifier.getPortIdentifiers();

		if (ports == null) {
			System.out.println("No comm ports found!");
			return;
		}

		// print out all ports
		System.out.println("printing all ports...");
		while (ports.hasMoreElements()) {
			System.out.println("-  " + ((CommPortIdentifier) ports.nextElement()).getName());
		}
		System.out.println("done.");
	}

	public void read()  throws IOException {

		int loncount = 0;
		int latcount = 0;
		int index = 0;
		int sIndex = 0;
		byte[] packet = new byte[MSG_SIZE];
		byte[] sPacket = new byte[4];
		byte[] lon = new byte[4];
		byte[] lat = new byte[4];
		

		int source, target=-1;
		byte c;


		c = (byte)in.read();

		while (true) {
		    latcount = 0;
		    loncount = 0;
		    index = 0;
		    sIndex = 0;

		    while(c !='#'){
			c = (byte)in.read();
			if( (char)c == '#')  
			    break;
		    }

		    while( (char)(c = (byte)in.read()) != '$' ){
			packet[index++] = c;
		    	if ( index >= 13)    break;
		    }

		    while( (char)(c = (byte)in.read()) != '|' ){
			sPacket[sIndex++] = c;
		    	if ( sIndex >= 2)    break;
		    }

		    int value = (((packet[0] & 0xff) << 24) | ((packet[1] & 0xff) << 16) |  ((packet[2] & 0xff) << 8) | (packet[3] & 0xff));

		    long lonvalue = ((((packet[4] & 0xff) << 24) | ((packet[5] & 0xff) << 16) |  ((packet[6] & 0xff) << 8) | (packet[7] & 0xff)) ) & 0xffffffffL ;
		    float f2 = 180 * ( Float.intBitsToFloat(value)/(float)3.14);

		    float lon_f2 = 180 * (Float.intBitsToFloat((int)lonvalue)/(float)3.14);

		    System.out.println(f2);
		    System.out.println(lon_f2);

		    Byte sByte = new Byte(sPacket[0]);
		    float sData  = sByte.floatValue();

		    topology.connect(f2,lon_f2);
		    sensorTopo.putDot(sData, (float)sensorCount++);
		}//while true


	}



}	
