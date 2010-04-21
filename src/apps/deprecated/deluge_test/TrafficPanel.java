import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.util.*;

/*
 * Created on Apr 4, 2005
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */

/**
 * @author Lane Phillips
 *
 * TODO To change the template for this generated type comment go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
public class TrafficPanel extends JPanel
{
	static final float NODE_WIDTH = 30;
	HashMap nodes = new HashMap();
	Traffic controls;
    float[] localmaxes = new float[5];
	Stats stats;
	
	public TrafficPanel(Traffic controls, Stats stats)
	{
		setBackground(Color.WHITE);
		this.controls = controls;
		this.stats = stats;
		
		for (Iterator i = stats.stats.keySet().iterator(); i.hasNext(); )
		{
			Integer id = (Integer)i.next();
			HashMap map = (HashMap)stats.stats.get(id);
			
			float x = ((id.intValue())%5) * 120 + 120 - 20*(((id.intValue())/5)%2); 
			float y = ((id.intValue())/5) * 120 + 120 - 20*((id.intValue())%2); 
			
			nodes.put(id, new Node(id.intValue(), x, y, map));
		}
		calcLocalMaxes(stats);
	}
	
	public void paintComponent(Graphics g1)
	{
		draw(g1);
	}
	
	public void draw(Graphics g1)
	{
		Graphics2D g = (Graphics2D)g1;
		
		g.setColor(getBackground());
		g.fillRect(0, 0, getWidth(), getHeight());
		
		Stroke nodeborder = new BasicStroke(5);
		Color[] colors = { new Color(1, 1, 0, 0.5f), new Color(0, 1, 1, 0.5f), new Color(0, 1, 0, 0.5f),
		        new Color(1, 0, 0, 0.5f), new Color(1, 0, 1, 0.5f) };
		Color black = new Color(0, 0, 0, 0.5f);
		
		float[] norm;
		switch(controls.normalization)
		{
		    case 0: norm = localmaxes; break;
		    case 1: norm = new float[] { localmaxes[4], localmaxes[4], localmaxes[4],
		            localmaxes[4], localmaxes[4] }; break;
		    case 2: norm = controls.globalmaxes; break;
		    case 3: norm = new float[] { controls.globalmaxes[4], controls.globalmaxes[4], controls.globalmaxes[4],
		            controls.globalmaxes[4], controls.globalmaxes[4] }; break;
		    default: norm = null;
		}
		
		for (Iterator i = nodes.keySet().iterator(); i.hasNext(); )
		{
			Object key = i.next();
			Node node = (Node)nodes.get(key);
			
			for (Iterator j = node.stats.keySet().iterator(); j.hasNext(); )
			{
				Object key2 = j.next();
				if (key.equals(key2)) continue;
				
				StatEntry stats = (StatEntry)node.stats.get(key2);
				
				float width = 0;
				Color color = null;
				switch(controls.packetType)
				{
					case 0:
						width = stats.summary / norm[0];
						break;
					case 1:
						width = stats.profile / norm[1];
						break;
					case 2:
						width = stats.request / norm[2];
						break;
					case 3:
						width = stats.data / norm[3];
						break;
					case 4:
						width = (stats.summary+stats.profile+stats.request+stats.data) / norm[4];
						break;
				}
				if (width == 0) continue;
				
				Node neighbor = (Node)nodes.get(key2);
				if (neighbor == null || neighbor == node) continue;
				
				g.setStroke(new BasicStroke(NODE_WIDTH * width, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
				g.setPaint(new GradientPaint(node.x, node.y, black, neighbor.x, neighbor.y, colors[controls.packetType]));
				g.draw(new Line2D.Float(node.x, node.y, neighbor.x, neighbor.y));
			}
		}
		
		for (Iterator i = nodes.keySet().iterator(); i.hasNext(); )
		{
			Object key = i.next();
			Node node = (Node)nodes.get(key);
			StatEntry sent = (StatEntry)node.stats.get(key);
			
			Color cfill = getBackground();
			float fac;
			switch(controls.packetType)
				{
					case 0:
					    fac = sent.summary / norm[0];
					    cfill = new Color(1, 1, 1-fac);
						break;
					case 1:
					    fac = sent.profile / norm[1];
					    cfill = new Color(1-fac, 1, 1);
						break;
					case 2:
					    fac = sent.request / norm[2];
					    cfill = new Color(1-fac, 1, 1-fac);
						break;
					case 3:
					    fac = sent.data / norm[3];
					    cfill = new Color(1, 1-fac, 1-fac);
						break;
					case 4:
					    fac = (sent.summary+sent.profile+sent.request+sent.data) / norm[4];
					    cfill = new Color(1, 1-fac, 1);
						break;
				}
			
			Shape circle = new Arc2D.Double(node.x-NODE_WIDTH/2, node.y-NODE_WIDTH/2, NODE_WIDTH, NODE_WIDTH, 0, 360, Arc2D.OPEN);
			g.setPaint(cfill);
			g.fill(circle);
			g.setStroke(nodeborder);
			g.setPaint(Color.BLACK);
			g.draw(circle);
		}
	}
	
	public BufferedImage getImage()
	{
		BufferedImage bi = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_ARGB);
		draw(bi.getGraphics());
		return bi;
	}
	
    private void calcLocalMaxes(Stats stats)
    {
        localmaxes = new float[5];
        Traffic.getMaxesFromTables(localmaxes, stats.stats);
    }
    
	class Node
	{
		int id;
		float x;
		float y;
		HashMap stats;
		
		Node(int id, float x, float y, HashMap stats)
		{
			this.id = id;
			this.x = x;
			this.y = y;
			this.stats = stats;
		}
	}
}
