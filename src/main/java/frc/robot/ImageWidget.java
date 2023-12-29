package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

public class ImageWidget extends JPanel {

  public ImageWidget() {
    SwingUtilities.invokeLater(() -> {
      JFrame frame = new JFrame("TEST");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      //frame.getContentPane().add(this);
      //frame.pack();
      frame.setSize(200, 200);
      frame.setVisible(true);
    });
  }
}
