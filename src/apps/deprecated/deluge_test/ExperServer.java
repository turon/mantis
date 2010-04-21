/*
 * Created on May 1, 2005
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
import java.net.*;
import java.io.*;
import java.awt.*;
import javax.swing.*;
import java.awt.event.*;
import java.util.*;

/* This program automates experiments on the testbed for studying the Aqueduct protocol.
 * ExperServer makes sockets connections with a special shell called aqshell to collect data about sensor nodes.
 * For more information see the comments in the rest of this file and also take a look at the following:
 *  src/apps/deluge_test/aqshell.c  - the Aqueduct shell that this program connects to
 *  src/mos/net/aqueduct_shell.c    - runs on the sensor node sending data to aqshell and receiving commands
 *  src/mos/net/aqueduct_shell.h    - common header file for aqshell.c and aqueduct_shell.h
 *   
 * I apologize for abusing the Java inner classes feature and not putting things in separate files,
 * but this started out as a quick-and-dirty hack.
 * 
 * BTW, this code uses generic classes and varargs, you need JDK 1.5 to compile this.
 */
public class ExperServer
{
    static final int TESTBED_SIZE = 25;
    /* Aqueduct shell commands.  These are duplicated in src/mos/net/include/aqueduct_shell.h 
     * See that file for descriptions.
     */
	static final byte AQSHELL_MESSAGE = (byte)'#';
	static final byte AQSHELL_START = (byte)'0';
	static final byte AQSHELL_NEWVERSION = (byte)'1';
	static final byte AQSHELL_COMPLETEPAGE = (byte)'2';
	static final byte AQSHELL_COMPLETEPROG = (byte)'3';
	static final byte AQSHELL_CLEARSTATS = (byte)'4';
	static final byte AQSHELL_STARTSTATS = (byte)'5';
	static final byte AQSHELL_STOPSTATS = (byte)'6';
	static final byte AQSHELL_SAVESTATS = (byte)'7';
	static final byte AQSHELL_GETSTATS = (byte)'8';
	static final byte AQSHELL_GETSTATS_REPLY = (byte)'9';
	static final byte AQSHELL_SETVERSION = (byte)'A';
	static final byte AQSHELL_SETVERSION_REPLY = (byte)'B';
	static final byte AQSHELL_SUMMARY = (byte)'C';
	static final byte AQSHELL_PROFILE = (byte)'D';
	static final byte AQSHELL_REQUEST = (byte)'E';
	static final byte AQSHELL_DATA = (byte)'F';
	static final byte AQSHELL_GETID = (byte)'G';
	static final byte AQSHELL_GETID_REPLY = (byte)'H';
	static final byte AQSHELL_SUMMARY_SEND = (byte)'I';
	static final byte AQSHELL_PROFILE_SEND = (byte)'J';
	static final byte AQSHELL_REQUEST_SEND = (byte)'K';
	static final byte AQSHELL_DATA_SEND = (byte)'L';
	static final byte AQSHELL_CLOSE = (byte)'M';
	static final byte AQSHELL_SETLOG = (byte)'N';
	static final byte AQSHELL_NOOP = (byte)'O';
	static final byte AQSHELL_GETVERSION = (byte)'P';
	static final byte AQSHELL_SETIMAGESIZE = (byte)'Q';
	static final byte AQSHELL_SETCACHESIZE = (byte)'R';
	static final byte AQSHELL_ALLQUIET = (byte)'S';
	static final byte AQSHELL_NOTQUIET = (byte)'T';
    static final byte AQSHELL_STOPUPDATE = (byte)'W';
    static final byte AQSHELL_CACHEHIT = (byte)'X';
    static final byte AQSHELL_CACHEMISS = (byte)'Y';
    static final byte AQSHELL_CACHEHITFORWARD = (byte)'Z';
    static final byte AQSHELL_CACHEHITOLDFORWARD = (byte)'a';

    /* These bits control how a command packet is handled.
     * Acknowledging nodes copy these bits into their replies and set AQSHELL_F_ACK.
     */
	static final byte AQSHELL_F_ACK = 0x01;            // The acknowledging shell or node will set this bit when replying 
	static final byte AQSHELL_F_PLEASEACK = 0x02;      // The shell or node that receives this command should acknowledge
	static final byte AQSHELL_F_RESEND = 0x04;         // Unused
	static final byte AQSHELL_F_FORWARDREPLY = 0x08;   // This bit tells the shell to forward a node's ACK to ExperServer
	static final byte AQSHELL_F_SHELLCTL = 0x10;       // This packet is a command for the shell
	static final byte AQSHELL_F_NODECTL = 0x20;        // This packet is a command that should be forwarded to the node

	static final String HANDSHAKE = "aqshell Handshake";   // On creating a new connection both parties exchange this string
    static final int TEST_PORT = 58799;         // ExperServer listens for connections on this TCP port
    static final int SERIAL_PORT = 58800;       // Unused
    static final int USB_PORT = 58900;          // The aqshell process monitoring /dev/ttyUSB0 will listen on TCP port USB_PORT,
                                                // aqshell monitoring /dev/ttyUSB1 will listen on USB_PORT+1, etc.
    static final byte SYNC_BYTE_1 = (byte)'#';  // All packets between aqshell and ExperServer start with these three bytes
    static final byte SYNC_BYTE_2 = (byte)'3';
    static final byte SYNC_BYTE_3 = (byte)'C';
    
    ControlPanel cp;        // The GUI
    NodeSquare[] nodes;     // GUIs for each node
    ArrayList<ShellConnection> connections = new ArrayList<ShellConnection>();      // Open connections to aqshell processes
    HashSet<Integer> ports = new HashSet<Integer>();    // Ports we are connected to
    ShellConnection seedNode;   // Source for code updates
    Experiment expers = new Experiment();   // Runs experiments
    
    boolean logging = false;        // We are logging.  See src/apps/deluge_test/unlog.py.
    PrintWriter logStream = null;   // Stream we are logging to.
    String logName = null;          // Name of the log file.
    
    /* Use this like C printf. */
    synchronized void log(String format, Object... args)
    {
    	if (logging) {
    		logStream.printf(format, args);
    		logStream.flush();
    	}
    }
	
    /* Log packet counts in a format understood by unlog.py */
	synchronized void logStats(ShellConnection sc)
	{
		int[] tot = new int[5];
		log("Stats %d\n", sc.ui.node);
		log(" Node Summary Profile Request Data Total\n");
		log("----- ------- ------- ------- ---- -----\n");
        for (int r=0; r<sc.stats.length; r++)
        {
			int t = sc.stats[r][0] + sc.stats[r][1] + sc.stats[r][2] + sc.stats[r][3];
			log("%5d %7d %7d %7d %4d %5d\n", 
			    r, sc.stats[r][0], sc.stats[r][1], sc.stats[r][2], sc.stats[r][3], t);
			tot[0] += sc.stats[r][0];
			tot[1] += sc.stats[r][1];
			tot[2] += sc.stats[r][2];
			tot[3] += sc.stats[r][3];
			tot[4] += t;
        }
		log("----- ------- ------- ------- ---- -----\n");
		log("Total %7d %7d %7d %4d %5d\n", tot[0], tot[1], tot[2], tot[3], tot[4]);
	}
    
    /* Set whether we are logging. */
    synchronized void setLogging(boolean logging)
    {
    	this.logging = logging;
    	if (!logging) logStream.flush();
    }

    /* Set the stream we are logging to. */
    synchronized void setLogStream(PrintWriter stream)
    {
    	logStream = stream;
    }

    /* This function iterates over all the USB-to-serial convertors on the system and connects to the aqshell process. */
    void findShells()
    {
    	final String usbstart = "ttyUSB";
    	// Find all the USB-to-serial convertors
		File dev = new File("/dev");
    	String[] usbs = dev.list(new FilenameFilter() {
    		public boolean accept(File dir, String name) {
    			return name.startsWith(usbstart);
    		}
    	});
        // No convertors found
    	if (usbs == null) return;
    	
    	for (int i=0; i<usbs.length; i++)
    	{
    		try
			{
    			System.out.println("Found USB device "+usbs[i]);
                // Figure out the port aqshell is listening on
				int port = USB_PORT + Integer.parseInt(usbs[i].substring(usbstart.length()));
                // Are we already connected?
	    		if (ports.contains(port)) {
	    			System.out.println("Already connected to node on "+usbs[i]);
	    			continue;
	    		}

				Socket s = new Socket("localhost", port);
				ShellConnection sc = new ShellConnection(s);
                // Test for a good handshake
            	if (sc.doHandshake(false)) {
					System.out.println("Connected to "+s.getInetAddress().getHostName()+" on port "+port);
                    // sc.key contains the port number aqshell was listening on
                    // Don't try this port again
                    ports.add(sc.key);
                    // Add to our list of all connections
                    connections.add(sc);
                    // Start listening 
					sc.start();
            	}
			}
			catch (NumberFormatException e)
			{
				e.printStackTrace();
			}
			catch (ConnectException e)
			{
                // This is expected if no aqshell is listening on the port
				System.err.println(e.getMessage());
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
    	}
    	
    }
    
    /* Main ExperServer thread */
    void run()
    {
        // Show the GUI
        cp = new ControlPanel();
        JFrame f = new JFrame();
        f.getContentPane().add(cp);
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.pack();
        f.setVisible(true);
        
        try
        {
            // Look for existing aqshells that we can connect to 
        	findShells();
            
            ServerSocket ss = new ServerSocket(TEST_PORT);
            // Loop indefinitely and listen for aqshells that want to connect to us
	        while (true)
	        {
	            Socket s = ss.accept();
	            try {
	            	ShellConnection sc = new ShellConnection(s);
	            	if (sc.doHandshake(true)) {
						System.out.println("Connected to "+s.getInetAddress().getHostName()+" on port "+TEST_PORT);
                        // sc.key contains the port number aqshell would have been listening on if we had connected to it in findShells()
                        // Don't try this port again
	                    ports.add(sc.key);
                        // Add to our list of all connections
	                    connections.add(sc);
                        // Start listening 
						sc.start();
	            	}
	            } catch (IOException ioe) {
	                ioe.printStackTrace();
	            }
	        }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    
    public static void main(String[] args)
    {
        // Start the server
        ExperServer es = new ExperServer();
        es.run();
    }
    
    /* Bounds check and return a node's GUI if the ID is good. */
    NodeSquare getNodeUI(int node)
    {
        if (node<0 || node>=nodes.length) {
            System.err.println("Node index "+node+" out of range.");
            return null;
        }
        return nodes[node];
    }
    
    /* This class manages the connection with one aqshell process.
     * There is one aqshell process per node.
     */
    class ShellConnection implements Runnable
    {
        DataInputStream in;     // For sending and receiving on the socket
        DataOutputStream out;
        Socket sock;            // The socket connected to the aqshell
        int key;                // The port number that the aqshell listens on when it isn't connected
        NodeSquare ui;          // The GUI for this connection.  It is null until we receive a valid node ID.
        
        // Every packet begins with these 3 bytes
        byte[] syncbytes = { SYNC_BYTE_1, SYNC_BYTE_2, SYNC_BYTE_3 };
        // Packet for sending
        AqShellPacket spkt = new AqShellPacket();
        
        int[][] stats = new int[32][4];     // Aqueduct packet counts gathered by aqshell
        long versionTime;                   // Time when this node was given a new version number
        long finishTime;                    // Time when this node completed receiving the new version
        int lastAck;                        // Last command that was acknowledged
        int version;                        // Version number of node
        int size;                           // Size of image on node
        boolean allquiet = true;            // Haven't heard any requests or data from this node for 30 seconds
        boolean isTarget;                   // This node is interested in updates

        ShellConnection(Socket s) throws IOException
        {
        	in = new DataInputStream(new BufferedInputStream(s.getInputStream()));
        	out = new DataOutputStream(new BufferedOutputStream(s.getOutputStream()));
            sock = s;
        }
        
        /* Handshake to initiate a connection between aqshell and ExperServer
         * isServer - is set to true if this is the process that got the socket from an accept() call
         *              This process will go first in the handshaking
         *              TODO Actually, it's not necessary to take turns, both could send right away, then both could listen
         */
        boolean doHandshake(boolean isServer)
        {
        	try
    		{
    			if (isServer) {
                    // Server sends handshake string first
    			    out.writeUTF(HANDSHAKE);
    			    out.flush();
    			} else {
                    // We're not the server so the 'key' is the port that we connected to
                    // See findShells() to see how this value is used
    				key = sock.getPort();
    			}
                // Receive and verify the handshake from the other end
    			String hs = in.readUTF();
    			if (!hs.equals(HANDSHAKE))
    			    return false;
    			if (!isServer) {
                    // Client sends handshake string second
    			    out.writeUTF(HANDSHAKE);
    			    out.flush();
    			} else {
                    // If we're the server we have no idea what port the aqshell at the other end was listening on,
                    // so he sends it to us
    				key = in.readInt();
    			}
    			System.out.println("Good handshake.");
    			return true;
    		}
    		catch (IOException e)
    		{
    			e.printStackTrace();
    		}
    		return false;
        }
        
        /* Launch this connection's listening thread. */
        void start()
        {
            new Thread(this).start();
        }
        
        /* This thread listens for packets from aqshell */
        public void run()
        {
            // Packet for receiving
        	AqShellPacket rpkt = new AqShellPacket();
            try
            {
                // Ping the node so it will reply with its ID and version information
                sendNodeCommand(AQSHELL_GETVERSION);
                while (!sock.isClosed())
                {
                    // Look for the beginning of a packet
                    // TODO Since TCP is reliable, I suppose this is unnecessary
                	if (in.readUnsignedByte() != SYNC_BYTE_1) continue;
                	if (in.readUnsignedByte() != SYNC_BYTE_2) continue;
                	if (in.readUnsignedByte() != SYNC_BYTE_3) continue;
                	
                	rpkt.recv(in);     // Read the packet from the socket
                    
                    // If this packet came from a node, check its address and find the corresponding GUI
                	if ((rpkt.flags & AQSHELL_F_SHELLCTL)==0 && (ui == null || ui.node != rpkt.id)) {
                		NodeSquare ns = getNodeUI(rpkt.id);
                		if (ns != null) {
                			ui = ns;
                			ui.setConnection(this);
                		} else {
                            // We got a bad node address, print out the packet for debugging
                			rpkt.dump();
                		}
                	}

                	switch (rpkt.command)
                    {
            			case AQSHELL_CLEARSTATS:
            			case AQSHELL_STARTSTATS:
            			case AQSHELL_STOPSTATS:
            			case AQSHELL_SAVESTATS:
            			case AQSHELL_GETID:
            			case AQSHELL_MESSAGE:
            			case AQSHELL_START:
            			case AQSHELL_SUMMARY:
            			case AQSHELL_PROFILE:
            			case AQSHELL_REQUEST:
            			case AQSHELL_DATA:
            			case AQSHELL_NOOP:
                		case AQSHELL_GETID_REPLY:
            			case AQSHELL_SUMMARY_SEND:
            			case AQSHELL_PROFILE_SEND:
            			case AQSHELL_REQUEST_SEND: 
            			case AQSHELL_DATA_SEND:
            			case AQSHELL_SETCACHESIZE:
                            // There is no special handling for these types
            				break;
            			case AQSHELL_COMPLETEPAGE:
                            // Display the new page number
                            if (ui != null)
                                ui.showVersionPage(rpkt.data[0], rpkt.data[1]+1);
                            System.out.print((rpkt.data[1]+1)+" ");
            			case AQSHELL_ALLQUIET:
                            // This node has not transmitted any data or requests for 30 seconds
            				synchronized (this) {
            					if (!allquiet) {
            						allquiet = true;
                                    // Wake up any experiments that may be waiting
            						notify();
            					}
            				}
            				break;
            			case AQSHELL_NOTQUIET:
                            // This node has broken its silence by transmitting a request or data packet
            				synchronized (this) {
            					if (allquiet) {
            						allquiet = false;
                                    // Wake up any experiments that may be waiting
            						notify();
            					}
            				}
            				break;
            			case AQSHELL_NEWVERSION:
                            // This node may be beginning a new update
            				finishTime = 0;
                            if (ui != null) {
                				ui.setFinished(false);
                                ui.showVersionPage(rpkt.data[0], 0);
                            }
            				break;
            			case AQSHELL_COMPLETEPROG:
                            // This node has finished downloading the update
                            if (ui != null)
                                ui.setFinished(true);
                            if (isTarget) {
                				log("Finished %d\n", rpkt.id);
                				log("Time: %10d.%06d\n", rpkt.ts_sec, rpkt.ts_usec);
                            }
                            synchronized (this) {
                                finishTime = 1000L * rpkt.ts_sec + rpkt.ts_usec / 1000;
                                // Notify any experiment that might be listening for this node to finish
                                notify();
                            }
            				break;
            			case AQSHELL_GETSTATS:
            			case AQSHELL_GETSTATS_REPLY: {
            				if ((rpkt.flags & AQSHELL_F_ACK) != 0) {
            					// Final ACK  packet only, no data
								logStats(this);
            					break;
            				}
                            // Copy data into the stats array
            				DataInput di = rpkt.getDataInput();
            	            int row = di.readUnsignedShort();
            	            //System.out.println("Reading rows "+row+" to "+(row+4));
            	            for (int r=row; r<row+4 && r<stats.length; r++)
            	            {
            	                for (int i=0; i<4; i++)
            	                {
            	                    stats[r][i] = di.readUnsignedShort();
            	                }
            	            }
            				break;
            			}
                        case AQSHELL_STOPUPDATE:
                            version = rpkt.data[0];
                            size = rpkt.data[1];
                            if (ui != null)
                                ui.showVersionPage(rpkt.data[0], rpkt.data[1]);
                            break;
            			case AQSHELL_GETVERSION:
                            // Display the current version info for this node
                            version = rpkt.data[0];
                            size = rpkt.data[1];
                            if (ui != null)
                                ui.showVersionPage(rpkt.data[0], rpkt.data[1]);
            				break;
                        case AQSHELL_SETIMAGESIZE:
                            // This node is acknowledging that its image size was updated by aqshell
                            versionTime = 1000L * rpkt.ts_sec + rpkt.ts_usec / 1000;
                            version = rpkt.data[1];
                            size = rpkt.data[0];
                            if (ui != null) {
                                ui.setMessage("Size Ack");
                                ui.showVersionPage(rpkt.data[1], rpkt.data[0]);
                            }
                            log("Version acknowledge %d\n", rpkt.id);
                            log("Time: %10d.%06d\n", rpkt.ts_sec, rpkt.ts_usec);
                            break;
            			case AQSHELL_SETVERSION:
            			case AQSHELL_SETVERSION_REPLY:
                            // This node is acknowledging that its version number was updated by aqshell
            				versionTime = 1000L * rpkt.ts_sec + rpkt.ts_usec / 1000;
                            version = rpkt.data[0];
                            size = rpkt.data[1];
                            if (ui != null) {
                				ui.setMessage("Version Ack");
                                ui.showVersionPage(rpkt.data[0], rpkt.data[1]);
                            }
            				log("Version acknowledge %d\n", rpkt.id);
            				log("Time: %10d.%06d\n", rpkt.ts_sec, rpkt.ts_usec);
            				break;
            			case AQSHELL_CLOSE:
                            // The aqshell process was interrupted, so it is telling us to close the connection
                            // It is cleaner if the client closes a TCP connection first
            				System.out.println("Received AQSHELL_CLOSE");
            				sock.close();
            				break;
                        default:
                            System.err.println("Unrecognized type: "+Integer.toHexString(rpkt.command));
                    }

                	if ((rpkt.flags & AQSHELL_F_ACK) != 0) {
                    	synchronized (this) {
                            // Save the command if this was an ACK
	                		lastAck = rpkt.command;
                            // Notify any experiment that might be listening
	                		notify();
	                	}
                	}
                }
            }
            catch (EOFException e)
            {
                System.err.println(e);
            	//e.printStackTrace();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        	try
			{
                // Close the socket so aqshell knows we disconnected
                // TODO if we are the server, and aqshell is still connected, aqshell should close the socket first
				sock.close();
			}
			catch (IOException e1)
			{
				e1.printStackTrace();
			}
            // Detach from GUI
            if (ui != null)
            	ui.setConnection(null);
            ui = null;
            // Remove ourselves from the list of connections
            connections.remove(this);
            // And ports
            ports.remove(key);
        }

        /* Send a command to the node to change its version number. */
		public void setVersion(int i)
		{
            try
			{
            	out.write(syncbytes);
            	AqShellPacket spkt = new AqShellPacket(AQSHELL_SETVERSION, 
													   AQSHELL_F_PLEASEACK|AQSHELL_F_NODECTL|AQSHELL_F_FORWARDREPLY,
													   -1, 0, 1);
            	spkt.data[0] = (byte)i;
            	spkt.send(out);
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
		}

        /* Send a command to the node to change its size and version.
         * We have to change the version number at the same time so updates don't occur from two different images. */
		public void setImageSize(int size, int version)
		{
            try
			{
            	out.write(syncbytes);
            	AqShellPacket spkt = new AqShellPacket(AQSHELL_SETIMAGESIZE, 
													   AQSHELL_F_PLEASEACK|AQSHELL_F_NODECTL|AQSHELL_F_FORWARDREPLY,
													   -1, 0, 2);
            	spkt.data[0] = (byte)size;
            	spkt.data[1] = (byte)version;
            	spkt.send(out);
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
		}

        /* Send a command without any extra data to the node. */
        public void sendNodeCommand(byte command)
		{
            try
			{
            	out.write(syncbytes);
            	AqShellPacket spkt = new AqShellPacket(command, 
													   AQSHELL_F_PLEASEACK|AQSHELL_F_NODECTL|AQSHELL_F_FORWARDREPLY,
													   -1, 0, 0);
            	spkt.send(out);
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
		}

        /* Send a command without any extra data to aqshell. */
		public void sendShellCommand(byte command)
		{
            try
			{
            	out.write(syncbytes);
            	AqShellPacket spkt = new AqShellPacket(command, 
													   AQSHELL_F_PLEASEACK|AQSHELL_F_SHELLCTL,
													   -1, 0, 0);
            	spkt.send(out);
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
		}

        /* Set the ACK command.  Used to clear the value by setting it to -1. */
		public synchronized void setAck(int i)
		{
			lastAck = i;
		}

		/* Returns the last command that was acknowledged. */
		public synchronized int getAck()
		{
			return lastAck;
		}

        /* Query a node for its version number and keep resending the query every five seconds. */
		public int getVersion() throws InterruptedException
		{
			synchronized (this) {
				lastAck = -1;
				while (lastAck != AQSHELL_GETVERSION) {
					sendNodeCommand(AQSHELL_GETVERSION);
					wait(5000);
				}
			}
			return version;
		}

        /* Send a command to the node to change its forwarding cache size. 
         * app is the index in src/mos/deluge_proto.c's list of applications. */
		public void setCacheSize(int size, int app)
		{
            try
			{
            	out.write(syncbytes);
            	AqShellPacket spkt = new AqShellPacket(AQSHELL_SETCACHESIZE, 
													   AQSHELL_F_PLEASEACK|AQSHELL_F_NODECTL|AQSHELL_F_FORWARDREPLY,
													   -1, 0, 2);
            	spkt.data[0] = (byte)size;
            	spkt.data[1] = (byte)app;
            	spkt.send(out);
			}
			catch (IOException e)
			{
				e.printStackTrace();
			}
		}

        public void stopUpdating(int app) throws InterruptedException
        {
            try
            {
                synchronized (this) {
                    lastAck = -1;
                    while (lastAck != AQSHELL_STOPUPDATE) {
                        out.write(syncbytes);
                        AqShellPacket spkt = new AqShellPacket(AQSHELL_STOPUPDATE, 
                                                               AQSHELL_F_PLEASEACK|AQSHELL_F_NODECTL|AQSHELL_F_FORWARDREPLY,
                                                               -1, 0, 1);
                        spkt.data[0] = (byte)app;
                        spkt.send(out);
                        wait(5000);
                    }
                }
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }
    
    /* This is a GUI for interacting with the testbed. */
    class ControlPanel extends JPanel implements ActionListener
    {
		private static final long serialVersionUID = 3616444613897107767L;    // Eclipse told me to put this here
        JButton bExperAll;
        JTextField tStartSize;
        JTextField tStartIter;
        
        JButton bExperV;
        JButton bExperS;
        JButton bStop;
        JButton bScanUsb;
        JButton bPoll;
        JButton bPollAll;
        JToggleButton bLogFile;
        JTextField tLogFile;
        ButtonGroup seedSelect;
        
        Thread experThread;
        
        ControlPanel()
        {
            setLayout(new BorderLayout());
            
            JPanel p = new JPanel();
            bExperAll = new JButton("Run All");
            bExperAll.addActionListener(this);
            p.add(bExperAll);
            p.add(new JLabel("Start size:"));
            tStartSize = new JTextField(5);
            tStartSize.setText("2");
            p.add(tStartSize);
            p.add(new JLabel("Start iteration:"));
            tStartIter = new JTextField(5);
            tStartIter.setText("0");
            p.add(tStartIter);

            bExperV = new JButton("Experiment (Bump Version)");
            bExperV.addActionListener(this);
            p.add(bExperV);
            bExperS = new JButton("Experiment (Bump Version and Size)");
            bExperS.addActionListener(this);
            p.add(bExperS);
            bStop = new JButton("Stop");
            bStop.addActionListener(this);
            p.add(bStop);
            bLogFile = new JToggleButton("Logging");
            bLogFile.addActionListener(this);
            p.add(bLogFile);
            tLogFile = new JTextField(15);
            p.add(tLogFile);
            bScanUsb = new JButton("Scan USB");
            bScanUsb.addActionListener(this);
            p.add(bScanUsb);
            bPoll = new JButton("Poll Unconnected");
            bPoll.addActionListener(this);
            p.add(bPoll);
            bPollAll = new JButton("Poll All");
            bPollAll.addActionListener(this);
            p.add(bPollAll);
            add(new JScrollPane(p), BorderLayout.NORTH);
            
            p = new JPanel(new GridLayout(5,5));
			JScrollPane jsp = new JScrollPane(p);
            add(jsp, BorderLayout.CENTER);
            
            seedSelect = new ButtonGroup();
            
            // Create a GUI for each node
            nodes = new NodeSquare[TESTBED_SIZE];
            for (int i=0; i<nodes.length; i++)
            {
                nodes[i] = new NodeSquare();
                nodes[i].setBorder(BorderFactory.createTitledBorder("Node "+(i)));
                nodes[i].node = i;
                seedSelect.add(nodes[i].rSeed);
                p.add(nodes[i]);
            }
            
        }
        
        /* Handle events from the control panel buttons. */
		public void actionPerformed(ActionEvent ae)
		{
            // All of these actions are long-running so we launch them in separate threads to avoid tying up the GUI thread
            // TODO some of these buttons should be disabled while an experiment is running
			Object src = ae.getSource();
			if (src == bScanUsb) {
                // Re-run the initial scan for USB-serial convertors
				new Thread(new Runnable() {
					public void run() {
						findShells();
					}
				}).start();
			} else if (src == bLogFile) {
                // Toggle whether we are logging
				if (bLogFile.isSelected() && (logStream == null || !tLogFile.getText().equals(logName))) {
					try {
                        // If the file name in the text field changed, open a new log file,
                        // otherwise we continue appending to the original log file
						PrintWriter pw = new PrintWriter(new FileWriter(tLogFile.getText()));
						setLogStream(pw);
					} catch (IOException e) {
						e.printStackTrace();
					}
				}
				setLogging(bLogFile.isSelected());
            } else if (src == bExperAll) {
                if (seedNode == null) {
                    // Can't run reprogramming without a seed node
                    // TODO We need at least one target, too
                    JOptionPane.showMessageDialog(this, "Please select a seed node", "Run All", JOptionPane.ERROR_MESSAGE);
                    return;
                }
                // Launch a thread to run several experiments in a row
                experThread = new Thread(new Runnable() {
                    public void run() {
                        try
                        {
                            // Run five iterations at image sizes ranging from 2 pages to 10 pages
                            // The text fields give a starting image size and iteration,
                            // this is useful for continuing a series of experiments that was interrupted
                            expers.runMany(seedNode, connections, 2, 10, 2, 5,
                                           Integer.parseInt(tStartSize.getText()),
                                           Integer.parseInt(tStartIter.getText()));
                        }
                        catch (InterruptedException e)
                        {
                            System.out.println("Experiment interrupted.");
                        }
                        catch (IOException e)
                        {
                            JOptionPane.showMessageDialog(ControlPanel.this, "IOException: "+e, "Run All", JOptionPane.ERROR_MESSAGE);
                            e.printStackTrace();
                        }
                    }
                });
                experThread.start();
            } else if (src == bExperV) {
                if (seedNode == null) {
                    JOptionPane.showMessageDialog(this, "Please select a seed node", "Experiment 1", JOptionPane.ERROR_MESSAGE);
                    return;
                }
                // Launch a new experiment
                // The update is triggered by incrementing the version number at the seed node
                experThread = new Thread(new Runnable() {
                    public void run() {
                        try
                        {
                            expers.experiment1(seedNode, connections, false, -1);
                        }
                        catch (InterruptedException e)
                        {
                            System.out.println("Experiment interrupted.");
                        }
                    }
                });
                experThread.start();
            } else if (src == bExperS) {
                if (seedNode == null) {
                    JOptionPane.showMessageDialog(this, "Please select a seed node", "Experiment 1", JOptionPane.ERROR_MESSAGE);
                    return;
                }
                // Launch a new experiment and increment the image size
                // The update is triggered by incrementing the version number at the seed node
                // TODO experiment1() allows setting an arbitrary page size, add a text field for this
                experThread = new Thread(new Runnable() {
                    public void run() {
                        try
                        {
                            expers.experiment1(seedNode, connections, true, -1);
                        }
                        catch (InterruptedException e)
                        {
                            System.out.println("Experiment interrupted.");
                        }
                    }
                });
                experThread.start();
			} else if (src == bStop) {
                // The experiment thread will throw InterruptedException from any wait() calls
                if (experThread != null)
                    experThread.interrupt();
                new Thread(new Runnable() {
                    public void run() {
                        System.out.println("Stopping updates on all nodes:");
                        try
                        {
                            for (Iterator<ShellConnection> i = connections.iterator(); i.hasNext(); ) {
                                ShellConnection sc = i.next();
                                if (seedNode == sc) continue;
                                int app = 1;
                                if (sc.isTarget) app = 0;
                                System.out.print(" "+(sc.ui==null? "<null>" : Integer.toString(sc.ui.node)));
                                sc.stopUpdating(app);
                            }
                            System.out.println();
                        }
                        catch (InterruptedException e)
                        {
                            e.printStackTrace();
                        }
                        System.out.println();
                    }
                }).start();
            } else if (src == bPoll) {
                // If we missed the initial ping reply when we started any new connections, ping them again
                // TODO Why does this happen
				new Thread(new Runnable() {
					public void run() {
                        for (int i = connections.size()-1; i>=0; --i) {
                            ShellConnection sc = connections.get(i);
							if (sc.ui == null) {
								sc.sendNodeCommand(AQSHELL_GETVERSION);
							}
						}
					}
				}).start();
			} else if (src == bPollAll) {
                // Ping all connections
                // TODO Do something to all the node GUI's to help us see the ping, for example disconnect all of them
				new Thread(new Runnable() {
					public void run() {
                        for (int i = connections.size()-1; i>=0; --i) {
                            ShellConnection sc = connections.get(i);
							sc.sendNodeCommand(AQSHELL_GETVERSION);
						}
					}
				}).start();
			}
		}
    }
    
    /* This class is one element of the grid of nodes shown by the control panel. */
    class NodeSquare extends JPanel implements ActionListener
    {
		private static final long serialVersionUID = 3834587720948199730L;    // Eclipse told me to put this in
		JLabel lConn;
        JLabel lFinish;
        JLabel lMsg;
        JButton bStats;
        
        JButton bStart;
        JButton bStop;
        JButton bClear;
        JButton bGetStats;
        JButton bVersion;
        JTextField tVersion;
        JButton bSize;
        JTextField tSize;
        JButton bCache;
        JTextField tCache;
        JTextField tCacheApp;
        JCheckBox cTarget;
        JRadioButton rSeed;
		JLabel lVerPage;
        JButton bStopUpdate;
        JTextField tStopApp;

        // This node's index that points back into the array
        // ShellConnection checks this whenever it gets a packet from a node
        int node;
        // Points back to the connection for this node, may be null
        ShellConnection connection;
        
        NodeSquare()
        {
            setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
            lConn = new JLabel("Not Connected");
            add(lConn);
            lFinish = new JLabel("Not Finished");
            add(lFinish);
			lVerPage = new JLabel("( , )");
            add(lVerPage);
            lMsg = new JLabel("No Messages");
            add(lMsg);
            /*bStats = new JButton("Stats");
            bStats.addActionListener(this);
            add(bStats);*/
            bStart = new JButton("Start Record");
            bStart.addActionListener(this);
            add(bStart);
            bStop = new JButton("End Record");
            bStop.addActionListener(this);
            add(bStop);
            bClear = new JButton("Clear Stats");
            bClear.addActionListener(this);
            add(bClear);
            bGetStats = new JButton("Get Stats");
            bGetStats.addActionListener(this);
            add(bGetStats);
            
            JPanel p = new JPanel();
            bStopUpdate = new JButton("Stop Update");
            bStopUpdate.addActionListener(this);
            p.add(bStopUpdate);
            tStopApp = new JTextField(3);
            p.add(tStopApp);
            add(p);
            
            p = new JPanel();
            bVersion = new JButton("Set Version");
            bVersion.addActionListener(this);
            p.add(bVersion);
            tVersion = new JTextField(3);
            p.add(tVersion);
            add(p);
            
            p = new JPanel();
            bSize = new JButton("Set Version and Size");
            bSize.addActionListener(this);
            p.add(bSize);
            tSize = new JTextField(3);
            p.add(tSize);
            add(p);
            
            p = new JPanel();
            bCache = new JButton("Set Cache Size");
            bCache.addActionListener(this);
            p.add(bCache);
            tCache = new JTextField(3);
            p.add(tCache);
            tCacheApp = new JTextField(3);
            p.add(tCacheApp);
            add(p);
            
            p = new JPanel();
            cTarget = new JCheckBox("Target");
            cTarget.addActionListener(this);
            p.add(cTarget);
            rSeed = new JRadioButton("Seed");
            rSeed.addActionListener(this);
            p.add(rSeed);
            add(p);
            
            connection = null;
            checkEnable();
        }
        
        // Disable all the controls if there is no node connected
        void checkEnable()
        {
            bVersion.setEnabled(connection != null);
            bSize.setEnabled(connection != null);
            bCache.setEnabled(connection != null);
            cTarget.setEnabled(connection != null);
            rSeed.setEnabled(connection != null);
            bStopUpdate.setEnabled(connection != null);
        }
        
        // Update the label that shows the version a number of available pages
		public void showVersionPage(int b, int c)
		{
			lVerPage.setText("("+(b&0xFF)+", "+(c&0xFF)+")");
		}

        // Connect a node to this GUI (or disconnect it)
		public void setConnection(ShellConnection connection)
		{
			this.connection = connection;
			lConn.setText(connection != null? "CONNECTED" : "Not Connected");
            checkEnable();
		}
		
        // Set the label that says this node has finished updating
		public void setFinished(boolean state)
		{
			lFinish.setText(state? "FINISHED" : "Not Finished");
		}

        // Display a message for this node
        // TODO More commands should make use of this
		public void setMessage(String msg)
		{
			lMsg.setText(msg);
		}

        public void actionPerformed(ActionEvent ae)
        {
            Object src = ae.getSource();
            /*if (src == bStats) {
            	connection.printStats();
            } else*/ if (src == bVersion) {
                // Change the version number of the home application on a node
            	connection.setVersion(/*Integer.parseInt(tVerApp.getText()),*/ Integer.parseInt(tVersion.getText()));
            } else if (src == bSize) {
                // Change the size of the home application on a node
                // This also uses the version field, since it is a bad thing to set the size without changing the version, too
                // If you change the size without changing the version, updates may flow in unwanted directions and the image
                // will fail its CRC, since no profile packet will be flooded with the new image CRC
            	connection.setImageSize(Integer.parseInt(tSize.getText()), Integer.parseInt(tVersion.getText()));
            } else if (src == bCache) {
                // Change the cache size on a forwarding node
                // You have to specify the application index
            	connection.setCacheSize(Integer.parseInt(tCache.getText()), Integer.parseInt(tCacheApp.getText()));
            } else if (src == cTarget) {
                // Set this node as a target, experiments will wait for it to finish
                connection.isTarget = cTarget.isSelected();
                // TODO these checkboxes don't enable and disable quite correctly
                rSeed.setEnabled(cTarget.isSelected());
            } else if (src == rSeed) {
                // Set this node as the seed node
                // We use a checkbox group so only one may be selected
                if (rSeed.isSelected())
                    seedNode = connection;
                // TODO these checkboxes don't enable and disable quite correctly
                cTarget.setEnabled(rSeed.isSelected());
            } else if (src == bStart) {
            	connection.sendShellCommand(AQSHELL_STARTSTATS);
            } else if (src == bStop) {
            	connection.sendShellCommand(AQSHELL_STOPSTATS);
            } else if (src == bClear) {
            	connection.sendShellCommand(AQSHELL_CLEARSTATS);
            } else if (src == bGetStats) {
            	connection.sendShellCommand(AQSHELL_GETSTATS);
            } else if (src == bStopUpdate) {
                new Thread(new Runnable() {
                    public void run() {
                        try
                        {
                            connection.stopUpdating(Integer.parseInt(tStopApp.getText()));
                        }
                        catch (InterruptedException e)
                        {
                            e.printStackTrace();
                        }
                    }
                }).start();
            }
        }
    }
    
    /* This class collects methods for running experiments on the testbed. */
    class Experiment
	{
        /* Send the command to all the aqshells in collection 'all', then wait for each to acknowledge.
         * Resend the command every 5 seconds.
         * If 'print' is true, print the node IDs as the commands are ACKed.
         * This method may be interrupted.
         */
        void sendToAllShells(Collection<ShellConnection> all, byte command, boolean print) throws InterruptedException
        {
            for (Iterator<ShellConnection> i = all.iterator(); i.hasNext(); ) {
                ShellConnection sc = i.next();
                if (print)
                    System.out.print(" "+(sc.ui==null? "<null>" : Integer.toString(sc.ui.node)));
                synchronized (sc) {
                    sc.setAck(-1);
                    while (sc.getAck() != command) {
                        sc.sendShellCommand(command);
                        sc.wait(5000);
                    }
                }
            }
            if (print)
                System.out.println();
        }
        
        /* Same as above, but for nodes instead of shells. */
        void sendToAllNodes(Collection<ShellConnection> all, byte command, boolean print) throws InterruptedException
        {
            for (Iterator<ShellConnection> i = all.iterator(); i.hasNext(); ) {
                ShellConnection sc = i.next();
                if (print)
                    System.out.print(" "+(sc.ui==null? "<null>" : Integer.toString(sc.ui.node)));
                synchronized (sc) {
                    sc.setAck(-1);
                    while (sc.getAck() != command) {
                        sc.sendNodeCommand(command);
                        sc.wait(5000);
                    }
                }
            }
            if (print)
                System.out.println();
        }
        
        /* Run an experiment many times.
         * Vary size from 'start' to 'stop' (inclusive) in 'step' increments.
         * Repeat this 'reps' times.
         * We can start anywhere in the sequence by using the startSize and startIter parameters.
         * This function may be interrupted.
         */
        void runMany(ShellConnection seed, ArrayList<ShellConnection> all, int start, int stop, int step, int reps,
                     int startSize, int startIter) throws IOException, InterruptedException
        {
            StringWriter sw = new StringWriter();
            PrintWriter fw = new PrintWriter(sw);
            StringBuffer fname = sw.getBuffer();
            
            startSize = Math.max(startSize, start);
            startIter = Math.max(startIter, 0);
            
            for (int r=startIter; r<reps; r++)
            {
                for (int s = (r==startIter? startSize : start); s<=stop; s += step)
                {
                    fname.setLength(0);
                    fw.printf("e1_1_%02d_%02d.log", s, r);
                    //fw.printf("e4_%02d_%02d.log", s, r);
                    
                    PrintWriter pw = new PrintWriter(new FileWriter(fname.toString()));
                    setLogStream(pw);
                    setLogging(true);              

                    experiment1(seed, all, true, s);
                    //experiment4(seed, all, s);

                    setLogging(false);              
                }
            }
        }
        
        /* Run a single experiment with the given node as a seed.
         * An update is started by changing the version number on the seed.
         * If bumpSize is true change the image size as well.
         * If newSize is positive use that as the new image size, otherwise increase the size by one page.
         * TODO give this method knowledge of which protocol it is testing
         */
    	void experiment1(ShellConnection seed, ArrayList<ShellConnection> all, boolean bumpSize, int newSize) throws InterruptedException
    	{
            System.out.println("Running experiment:");
            // Ping all the nodes to make sure they're all still connected
            // I've had some USB-serial convertors cause aqshell to drop the serial connection, even though the node keeps running
            System.out.println("\tTest connections: ");
            System.out.print("\t");
            sendToAllNodes(all, AQSHELL_GETVERSION, true);
            // Make sure the shells are not recording stats
            System.out.println("\tStop recording stats.");
            sendToAllShells(all, AQSHELL_STOPSTATS, false);
            System.out.println("\tClear stats.");
            // Clear any old stats
            sendToAllShells(all, AQSHELL_CLEARSTATS, false);
            // Now we can start recording stats with a blank slate
            System.out.println("\tStart recording stats.");
            sendToAllShells(all, AQSHELL_STARTSTATS, false);
            // Clear the finish time on all target nodes
            for (int i = all.size()-1; i>=0; --i) {
            	ShellConnection sc = all.get(i);
            	synchronized (sc) {
                    if (!sc.isTarget) continue;
            		sc.finishTime = 0;
            	}
            }
            long startTime = 0;
            long finishTime = 0;
            // Find the version number on the seed node
            int version = seed.getVersion();
            int size = seed.size;
            System.out.println("\tOld version is "+version);
            // Increment the version, this will cause a new update to start
            version++;
            // Are we changing the size, too?
            if (bumpSize) {
                System.out.println("\tOld size is "+size);
                if (newSize < 0)
                    // Default is one page more than current size
                    size++;
                else 
                    size = newSize;
                // Wait for the node to ACK the new version and size, resend every 5 seconds
                synchronized (seed) {
                    seed.setAck(-1);
                    while (seed.getAck() != AQSHELL_SETIMAGESIZE) {
                        seed.setImageSize(size, version);
                        seed.wait(5000);
                    }
                    startTime = seed.versionTime;
                }
                System.out.println("\tNew size is "+size);
            } else {
                // Not changing size
                // Wait for the node to ACK the new version, resend every 5 seconds
                synchronized (seed) {
                    seed.setAck(-1);
                    while (seed.getAck() != AQSHELL_SETVERSION) {
                        seed.setVersion(version);
                        seed.wait(5000);
                    }
                    startTime = seed.versionTime;
                }
            }
            System.out.println("\tNew version is "+version);
            
            // Wait until all targets have finish downloading the image
            System.out.println("\tWaiting for target nodes to finish.");
            System.out.print("\t");
            for (int i = all.size()-1; i>=0; --i) {
                ShellConnection sc = all.get(i);
            	synchronized (sc) {
                    if (!sc.isTarget) continue;
            		while (sc.finishTime == 0) {
            			sc.wait();
            		}
            		finishTime = Math.max(finishTime, sc.finishTime);
            	}
            }
            System.out.println();
            System.out.println("\tTotal programming time: "+((finishTime-startTime)/1000) + " seconds");
            System.out.println("\tWaiting for network to be quiet.");
            // Wait until no node has sent a profile, request, or data packet for the last 30 seconds
            // We do this because Deluge will keep updating forwarding nodes,
            // even if all the target nodes have completed
            for (int i = all.size()-1; i>=0; --i) {
                ShellConnection sc = all.get(i);
            	synchronized (sc) {
            		if (sc.allquiet) continue;
            		sc.wait();
            	}
            	// Start over from the beginning, since previously quiet nodes could have broken their silences
            	i = all.size()-1;
            }
            System.out.println("\tStop recording stats.");
            sendToAllShells(all, AQSHELL_STOPSTATS, false);
            // Ping all the nodes to make sure we didn't lose one during the experiment
            System.out.println("\tTest connections again.");
            System.out.print("\t");
            sendToAllNodes(all, AQSHELL_GETVERSION, true);
            // Get the stats and print them to the log
            System.out.println("\tRetrieve stats.");
            sendToAllShells(all, AQSHELL_GETSTATS, false);
            System.out.println("\tDone.");
    	}
        
        /* Run a single experiment with the given node as a seed.
         * An update is started by changing the version number on the seed.
         * If bumpSize is true change the image size as well.
         * If newSize is positive use that as the new image size, otherwise increase the size by one page.
         * TODO give this method knowledge of which protocol it is testing
         */
    	void experiment4(ShellConnection seed, ArrayList<ShellConnection> all, int newSize) throws InterruptedException
    	{
            System.out.println("Running experiment:");
            // Ping all the nodes to make sure they're all still connected
            // I've had some USB-serial convertors cause aqshell to drop the serial connection, even though the node keeps running
            System.out.println("\tTest connections: ");
            System.out.print("\t");
            sendToAllNodes(all, AQSHELL_GETVERSION, true);
            // Make sure the shells are not recording stats
            System.out.println("\tStop recording stats.");
            sendToAllShells(all, AQSHELL_STOPSTATS, false);
            System.out.println("\tClear stats.");
            // Clear any old stats
            sendToAllShells(all, AQSHELL_CLEARSTATS, false);
            // Now we can start recording stats with a blank slate
            System.out.println("\tStart recording stats.");
            sendToAllShells(all, AQSHELL_STARTSTATS, false);
            // Clear the finish time on all target nodes
            for (int i = all.size()-1; i>=0; --i) {
            	ShellConnection sc = all.get(i);
            	synchronized (sc) {
                    if (!sc.isTarget) continue;
            		sc.finishTime = 0;
            	}
            }
            long startTime = 0;
            long finishTime = 0;
            
            // Set the cache size on all forwarding nodes
            System.out.println("Setting the cache size on forwarding nodes.");
            for (int i = all.size()-1; i>=0; --i) {
            	ShellConnection sc = all.get(i);
            	synchronized (sc) {
                    if (sc.isTarget || seed == sc) continue;
                    sc.setAck(-1);
                    do {
                    	sc.setCacheSize(newSize, 1);
                    	sc.wait(5000);
                    } while (sc.getAck() != AQSHELL_SETCACHESIZE);
            	}
            }
            
            // Find the version number on the seed node
            int version = seed.getVersion();
            int size = seed.size;
            System.out.println("\tOld version is "+version);
            // Increment the version, this will cause a new update to start
            version++;
            // Wait for the node to ACK the new version, resend every 5 seconds
            synchronized (seed) {
                seed.setAck(-1);
                while (seed.getAck() != AQSHELL_SETVERSION) {
                    seed.setVersion(version);
                    seed.wait(5000);
                }
                startTime = seed.versionTime;
            }
            System.out.println("\tNew version is "+version);
            
            // Wait until all targets have finish downloading the image
            System.out.println("\tWaiting for target nodes to finish.");
            System.out.print("\t");
            for (int i = all.size()-1; i>=0; --i) {
                ShellConnection sc = all.get(i);
            	synchronized (sc) {
                    if (!sc.isTarget) continue;
            		while (sc.finishTime == 0) {
            			sc.wait();
            		}
            		finishTime = Math.max(finishTime, sc.finishTime);
            	}
            }
            System.out.println();
            System.out.println("\tTotal programming time: "+((finishTime-startTime)/1000) + " seconds");
            System.out.println("\tStop recording stats.");
            sendToAllShells(all, AQSHELL_STOPSTATS, false);
            // Ping all the nodes to make sure we didn't lose one during the experiment
            System.out.println("\tTest connections again.");
            System.out.print("\t");
            sendToAllNodes(all, AQSHELL_GETVERSION, true);
            // Get the stats and print them to the log
            System.out.println("\tRetrieve stats.");
            sendToAllShells(all, AQSHELL_GETSTATS, false);
            System.out.println("\tDone.");
    	}
	}
}

/* Data structure for communicating with aqshell and the node. */
class AqShellPacket
{
    // aqshell adds a timestamp to all packets sent to ExperServer
	int ts_sec;    // seconds
	int ts_usec;   // microseconds
	
	int command;
	int flags;     // Tells the aqshell and the node how it should handle this packet
	int id;        // The ID of the sending node, if the packet came from a node
	int seq;       // Unused
	int length;    // Count of additional data bytes, not including the header
	byte[] data = new byte[1024];  // buffer for additional data
    // TODO I don't know why I made it so big
	
    // For treating the additional data bytes like a stream
 	ByteArrayOutputStream baos;

    /* Construct a blank packet, most likely for receiving. */
    AqShellPacket()
	{
		this.command =  ExperServer.AQSHELL_NOOP;
		this.flags = 0;
		this.id = 0xFFFF;
		this.seq = 0;
		this.length = 0;
	}
	
	public void dump()
	{
		System.out.println("Packet:");
		System.out.println("\tts_sec:  "+ts_sec);
		System.out.println("\tts_usec: "+ts_usec);
		System.out.println();
		System.out.println("\tcommand: "+command);
		System.out.println("\tflags:   "+flags);
		System.out.println("\tid:      "+id);
		System.out.println("\tseq:     "+seq);
		if (baos != null) {
			data = baos.toByteArray();
			length = data.length;
		}		
		System.out.println("\tlength:  "+length);
		System.out.println();
		System.out.print  ("\tdata:    ");
		for (int i=0; i<length; i++)
			System.out.printf(" %02x", data[i]);
		System.out.println();
		System.out.println();
	}

    /* Construct a new packet initialized to the given values. */
    AqShellPacket(int command, int flags, int id, int seq, int length)
	{
		this.command = command;
		this.flags = flags;
		this.id = id;
		this.seq = seq;
		this.length = length;
	}
	
	void recv(DataInputStream in) throws IOException
	{
        // aqshell adds a timestamp to all packets sent to ExperServer
		ts_sec = in.readInt();
		ts_usec = in.readInt();
		
		command = in.readUnsignedByte();
		flags = in.readUnsignedByte();
		id = in.readUnsignedShort();
		seq = in.readUnsignedByte();
		length = in.readUnsignedByte();
		
		if (length > 0)
			in.read(data, 0, length);
	}
	
	void send(DataOutputStream out) throws IOException
	{
        // We don't send a timestamp
		out.writeByte(command);
		out.writeByte(flags);
		out.writeShort(id);
		out.writeByte(seq);
		
		if (baos != null) {
            // If the user has created an output stream for writing the additional data,
            // we need to use it instead
			data = baos.toByteArray();
			length = data.length;
			baos.reset();
		}
		out.writeByte(length);
		if (length > 0)
			out.write(data, 0, length);
		out.flush();
	}
	
	// Get the additional data as an input stream
    DataInputStream getDataInput()
	{
		return new DataInputStream(new ByteArrayInputStream(data, 0, length));
	}
	
    // Get the additional data as an output stream
	DataOutputStream getDataOutput()
	{
		if (baos == null)
			baos = new ByteArrayOutputStream(256);
		return new DataOutputStream(baos);
	}
}
