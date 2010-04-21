import java.util.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.awt.*;
import java.io.*;
import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.filechooser.FileFilter;

public class Traffic extends JFrame implements ActionListener
{
    JTabbedPane tabs;
    JButton btnFile;
    File lastDir = new File("/home/lane/mos/mantis-unstable/build/mica2/src/apps/deluge_test");
    JComboBox cmbType;
    JComboBox cmbNorm;
    JButton btnClose;
    JButton btnSave;
    ArrayList allstats = new ArrayList();
    
    int packetType;
    int normalization;
    float[] globalmaxes = new float[5];
    
    public Traffic()
    {
		tabs = new JTabbedPane();
		getContentPane().add(tabs);
		
		JPanel jp = new JPanel();
		btnFile = new JButton("Load File");
		btnFile.addActionListener(this);
		jp.add(btnFile);
		
		btnSave = new JButton("Save Image");
		btnSave.addActionListener(this);
		jp.add(btnSave);
		
		btnClose = new JButton("Close Tab");
		btnClose.addActionListener(this);
		jp.add(btnClose);
		
		cmbType = new JComboBox(new String[] {
		        "Advertisements", "Profiles", "Requests", "Data", "All Packets"
		});
		cmbType.addActionListener(this);
		jp.add(cmbType);
		
		jp.add(new JLabel("Normalize:"));
		cmbNorm = new JComboBox(new String[] {
		        "by Packet Type", "for All Packets", "by Packet Type Across All Experiments", "for All Packets Across All Experiments"
		});
		cmbNorm.addActionListener(this);
		jp.add(cmbNorm);
		
		getContentPane().add(jp, BorderLayout.NORTH);
    }
    
    public void load(File file)
    {
		try {
			DataInputStream dis = new DataInputStream(new BufferedInputStream(new FileInputStream(file)));
			Stats stats = new Stats();
			stats.load(dis);
			dis.close();

			allstats.add(stats);
			calcGlobalMaxes();

			TrafficPanel tp = new TrafficPanel(this, stats);

			JPanel jp = new JPanel(new BorderLayout()); 
			jp.add(tp);
			
			JPanel jp2 = new JPanel();
			jp2.add(new JLabel("Completion time: "+(stats.finishTime - stats.startTime)+"s"));
			jp2.add(new JLabel("  Total packets: "+stats.totals[4]+"  Summary: "+stats.totals[0]+"  Profile: "+stats.totals[1]+
			                   "  Request: "+stats.totals[2]+"  Data: "+stats.totals[3]));
			jp.add(jp2, BorderLayout.NORTH);
			
			tabs.addTab(file.getName(), jp);
			tabs.setSelectedComponent(jp);
		} catch (Exception e) {
			e.printStackTrace();
		}
    }
    
	public static void main(String[] args)
	{
	    Traffic f = new Traffic();
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setSize(1024, 768);
		f.setVisible(true);
	}

    /* (non-Javadoc)
     * @see java.awt.event.ActionListener#actionPerformed(java.awt.event.ActionEvent)
     */
    public void actionPerformed(ActionEvent e)
    {
        if (e.getSource() == btnFile)
        {
            JFileChooser jfc = new JFileChooser(lastDir);
            jfc.setMultiSelectionEnabled(true);
            jfc.setFileFilter(new FileFilter() {
                public boolean accept(File f) {
                    return f.isDirectory() || f.getName().endsWith(".jdat");
                }
                public String getDescription() {
                    return "Aqueduct Binary Log Data";
                }
            });
            if (jfc.showOpenDialog(btnFile.getParent()) == JFileChooser.APPROVE_OPTION)
            {
                File[] f = jfc.getSelectedFiles();
                for (int i=0; i<f.length; i++)
                {
	                lastDir = f[i].getParentFile();
	                load(f[i]);
                }
            }
        }
        else if (e.getSource() == btnSave)
        {
            JFileChooser jfc = new JFileChooser(lastDir);
            if (jfc.showSaveDialog(btnFile.getParent()) == JFileChooser.APPROVE_OPTION)
            {
                File f = jfc.getSelectedFile();
                if (!f.getName().endsWith(".png"))
                    f = new File(f.getAbsolutePath() + ".png");
                lastDir = f.getParentFile();

                Container c = (Container)tabs.getSelectedComponent();
            	TrafficPanel tp = (TrafficPanel)(c.getComponent(0) instanceof TrafficPanel?
            			c.getComponent(0) : c.getComponent(1));	// ugly, I know
            	BufferedImage i = tp.getImage();
            	
        		try
				{
					ImageIO.write(i, "png", f);
				}
				catch (IOException e1)
				{
					e1.printStackTrace();
				} 
            }
        }
        else if (e.getSource() == btnClose)
        {
            tabs.remove(tabs.getSelectedIndex());
            
        }
        else if (e.getSource() == cmbType)
        {
            packetType = cmbType.getSelectedIndex();
            Component comp = tabs.getSelectedComponent();
            comp.repaint();
        }
        else if (e.getSource() == cmbNorm)
        {
            normalization = cmbNorm.getSelectedIndex();
            Component comp = tabs.getSelectedComponent();
            comp.repaint();
        }
    }
    
    static void getMaxesFromTables(float[] maxes, HashMap tables)
    {
		for (Iterator i = tables.keySet().iterator(); i.hasNext();)
		{
			Object key = i.next();
			HashMap map = (HashMap)tables.get(key);
			for (Iterator j = map.keySet().iterator(); j.hasNext();)
			{
				Object key2 = j.next();
				StatEntry se = (StatEntry)map.get(key2);
				maxes[0] = Math.max(se.summary, maxes[0]);
				maxes[1] = Math.max(se.profile, maxes[1]);
				maxes[2] = Math.max(se.request, maxes[2]);
				maxes[3] = Math.max(se.data, maxes[3]);
				maxes[4] = Math.max(se.summary+se.profile+se.request+se.data, maxes[4]);
			}
		}
    }
    
    private void calcGlobalMaxes()
    {
        globalmaxes = new float[5];
        for (Iterator i = allstats.iterator(); i.hasNext(); )
        {
            Stats stats = (Stats)i.next();
            getMaxesFromTables(globalmaxes, stats.stats);
        }
    }
}

class Stats
{
    int[] totals = new int[5];
    //String file;
	long startTime;
	long finishTime;
	HashMap stats = new HashMap();
	HashMap neighbors = new HashMap();

	void dump()
	{
		System.out.println("startTime "+startTime);
		System.out.println("finishTime "+finishTime);
		for (Iterator i = stats.keySet().iterator(); i.hasNext();)
		{
			Object key = i.next();
			System.out.println("Stats "+key);
			HashMap map = (HashMap)stats.get(key);
			for (Iterator j = map.keySet().iterator(); j.hasNext();)
			{
				Object key2 = j.next();
				StatEntry e = (StatEntry)map.get(key2);
				System.out.println(key2+" "+e.summary+" "+
					e.profile+" "+e.request+" "+e.data);
			}
		}
		for (Iterator i = neighbors.keySet().iterator(); i.hasNext();)
		{
			Object key = i.next();
			System.out.println("Neighbors "+key);
			NeighborList nl = (NeighborList)neighbors.get(key);
			for (int j=0; j<nl.ids.length; j++)
			{
				System.out.println(nl.ids[j]+" "+nl.dtbs[j]);
			}
		}
	}

	void load(DataInputStream in) throws IOException
	{
	    totals = new int[5];
		stats.clear();
		neighbors.clear();
		
		startTime = in.readLong();
		finishTime = in.readLong();
		int nnodes = in.readInt();
		for (int i=0; i<nnodes; i++)
		{
			int id = in.readInt();
			int nstats = in.readInt();
			HashMap map = new HashMap();
			stats.put(new Integer(id), map);
			for (int j=0; j<nstats; j++) {
				int id2 = in.readInt();
				StatEntry se = new StatEntry(in);
				map.put(new Integer(id2), se);
				if (id == id2) {
				    totals[0] += se.summary;
				    totals[1] += se.profile;
				    totals[2] += se.request;
				    totals[3] += se.data;
				}
			}
		}
		nnodes = in.readInt();
		for (int i=0; i<nnodes; i++)
		{
			int id = in.readInt();
			neighbors.put(new Integer(id), new NeighborList(in));
		}
		totals[4] = totals[0] + totals[1] + totals[2] + totals[3];
	}
}

class StatEntry
{
	int summary;
	int profile;
	int request;
	int data;

	StatEntry(DataInputStream in) throws IOException
	{
		summary = in.readInt();
		profile = in.readInt();
		request = in.readInt();
		data = in.readInt();
	}
}

class NeighborList
{
	int[] ids;
	int[] dtbs;

	NeighborList(DataInputStream in) throws IOException
	{
		int n = in.readInt();
		ids = new int[n];
		dtbs = new int[n];
		for (int i=0; i<n; i++)
		{
			ids[i] = in.readInt();
			dtbs[i] = in.readInt();
		}
	}
}

