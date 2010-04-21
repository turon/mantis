import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

/**
 * 
 * 
 * @author Hui Dai
 * 
 */
public class Node {
	
        private float lon, lat;

        public Node(){}

	public Node(float lat, float lon)
        {
		this.lat = lat;
		this.lon = lon;
	}

        public float getLon() {  return lon; }

        public float getLat() {  return lat; }

        public void set(float lat, float lon)
        {
		this.lat = lat;
		this.lon = lon;
	}

        public void setLon(float lon) {  this.lon = lon; }

        public void setLat(float lat) {  this.lat = lat; }

	public void printTopology() {
		System.out.println("lon : "+ lon + "  lat : "+ lat);
	}

}
