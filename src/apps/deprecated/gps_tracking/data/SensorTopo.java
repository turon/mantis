
import java.util.HashMap;
import java.util.Iterator;

/**
 * 
 * 
 * @author Hui Dai 
 * 
 */
public class SensorTopo {

	//===   CONSTANTS   =========================================================
	private static final int NODE_NUM = 2048; // 4 byte longitude, 4-byte latitude and 4 byte latitude, 1 byte start and 1 byte end

       	private Node[] nodes;
        private int nodeNum;

	public SensorTopo(){
	    nodes = new Node[NODE_NUM]; // key: Integer, value Node
	    nodeNum = 0;
	}
	
	public void setObserver(NetworkTopologyObserver observer) {
		this.observer = observer;
	}

	private NetworkTopologyObserver observer;


       public void putDot(float data, float time){
	    nodes[nodeNum] = new Node(data,time);
	    nodeNum++;
	    if(observer!=null)observer.changeEvent();
	}


	public void printTopology()
        {
	    for(int i=0; i<nodeNum; i++)
		nodes[i].printTopology();
	}
        
        public int getNodeNum() { return nodeNum;}

        public Node getNode(int index)
        {
	    if(index >= nodeNum)
		return null;
	    return nodes[index];
        }
	
}
