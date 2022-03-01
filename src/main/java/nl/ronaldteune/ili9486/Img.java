package nl.ronaldteune.ili9486;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.DataBufferShort;
import java.awt.image.DataBufferUShort;
import java.io.IOException;
import java.net.URL;
import java.util.Arrays;

public class Img {
    public static final int WIDTH = 480;
    public static final int HEIGHT = 320;
    private final BufferedImage buffer = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_USHORT_565_RGB);
    private final Graphics2D g2d = buffer.createGraphics();

    static {
        System.setProperty("java.awt.headless", "true");
    }

    public Img() {
        g2d.clearRect(0, 0, WIDTH, HEIGHT);
        g2d.setColor(Color.RED);
        g2d.setFont(new Font("DejaVu Sans", Font.PLAIN, 20));
        g2d.drawString("Hoi hoi test test", 50, 50);

        final URL resource = Img.class.getClassLoader().getResource("radio.png");
        try {
            final BufferedImage image = ImageIO.read(resource);
            g2d.drawImage(image, null, 0,0);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public short[] getRow(int x) {
        final short[] data = ((DataBufferUShort) buffer.getRaster().getDataBuffer()).getData();
        return Arrays.copyOfRange(data, x * WIDTH, x * WIDTH + WIDTH);
    }
}
