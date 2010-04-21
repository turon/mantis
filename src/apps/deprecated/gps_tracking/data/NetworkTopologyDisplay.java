
import java.awt.Canvas;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.Iterator;

/**
 * 
 * 
 * @author Hui Dai
 * 
 */
public class NetworkTopologyDisplay extends Canvas implements NetworkTopologyObserver {

	private static final int GRIDLENGTH = 50;
        private int count;

	public NetworkTopologyDisplay(NetworkTopology topology){
		this.topology = topology;
		count = 0;
	}
	
	private NetworkTopology topology;

	/**
	 * @see edu.colorado.cs.networktop.data.NetworkTopologyObserver#removeRelationshipEvent(Node, Node)
	 */
	public void removeRelationshipEvent(Node child, Node parent) {}

	/**
	 * @see edu.colorado.cs.networktop.data.NetworkTopologyObserver#addRelationshipEvent(Node, Node)
	 */
	public void addRelationshipEvent(Node child, Node parent) {}

	/**
	 * @see edu.colorado.cs.networktop.data.NetworkTopologyObserver#changeEvent()
	 */
	public void changeEvent() { repaint(); }

	
	/**
	 * @see java.awt.Component#paint(Graphics)
	 */
	public void paint(Graphics g) {
		g.clearRect(0,0,getMinimumSize().width, getMinimumSize().height);
		int num  = topology.getNodeNum();
		Node root;
		int x_root = 0, y_root = 0, x_offset, y_offset;
		
		if(num != 0){
		    root = topology.getNode(0);
		    x_root = (int)(root.getLat()*40000);
		    y_root = (int)(root.getLon()*40000);
		}

		for(int i=0; i<num-1; i++){
		    Node n = topology.getNode(i);
		    int lat = (int)(n.getLat()*40000); 
		    x_offset = lat - x_root;
		    int lon = (int)(n.getLon()*40000); 
		    y_offset = lon - y_root;
		    g.drawRect(320+x_offset,240+y_offset,2,2);
		    //	    System.out.println("get count " + count++);
		}
	}
	
	/**
	 * @see java.awt.Component#getMinimumSize()
	 */
	public Dimension getMinimumSize() {
		return new Dimension(640,480);
	}
	
	

	/**
	 * @see java.awt.Component#getMaximumSize()
	 */
	public Dimension getMaximumSize() {
		return getMinimumSize();
	}

	/**
	 * @see java.awt.Component#getPreferredSize()
	 */
	public Dimension getPreferredSize() {
		return getMinimumSize();
	}

	/**
	 * @see java.awt.Component#update(Graphics)
	 */
	public void update(Graphics arg0) {
		paint(arg0);
	}

}
