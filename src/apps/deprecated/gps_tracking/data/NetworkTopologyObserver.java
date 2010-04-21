/**
 * 
 * 
 * @author Hui  Dai
 * 
 */
public interface NetworkTopologyObserver {

	public void removeRelationshipEvent(Node child, Node parent);
	public void addRelationshipEvent(Node child, Node parent);
	public void changeEvent();



}
