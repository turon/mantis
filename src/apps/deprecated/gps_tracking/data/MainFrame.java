
import java.awt.BorderLayout;
import java.awt.Button;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * 
 * 
 * @author Hui   Dai
 * 
 */
public class MainFrame extends Frame {

	Button nextEventButton = new Button("Exit");

	public MainFrame(NetworkTopologyDisplay networkTopologyDisplay, String title){
		super(title);
		setLayout(new BorderLayout());
		add(networkTopologyDisplay,BorderLayout.CENTER);
		add(nextEventButton, BorderLayout.SOUTH);
		nextEventButton.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent e){
				System.exit(0);
			};
		});
		pack();
		setVisible(true);
	}

	public MainFrame(SensorTopoDisplay sensorTopoDisplay, String title){
		super(title);
		setLayout(new BorderLayout());
		add(sensorTopoDisplay,BorderLayout.CENTER);
		add(nextEventButton, BorderLayout.SOUTH);
		nextEventButton.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent e){
			    System.exit(0);
			};
		});
		pack();
		setVisible(true);
	}
	

}
