
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
public class SensorTopoDisplay extends Canvas implements NetworkTopologyObserver {

	private static final int GRIDLENGTH = 50;
        private int count;

	public SensorTopoDisplay(SensorTopo topology){
		this.topology = topology;
		count = 0;
	}
	
	private SensorTopo topology;

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
		g.drawLine(90, 300, 600, 300);
		g.drawLine(90, 300, 90, 100);

		int num  = topology.getNodeNum();
		
		for(int i=0; i<num; i++){
		    Node n = topology.getNode(i);
		    int lat = (int)(n.getLat()); 
		    int lon = (int)(n.getLon()); 
		    g.drawRect(100+lon,300-lat,2,2);
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
