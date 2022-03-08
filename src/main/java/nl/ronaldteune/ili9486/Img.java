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


        final URL resource = Img.class.getClassLoader().getResource("colour.png");
        try {
            final BufferedImage image = ImageIO.read(resource);
            g2d.drawImage(image, null, 0,0);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        g2d.setColor(Color.RED);
        g2d.setFont(new Font("DejaVu Sans", Font.PLAIN, 20));
        g2d.drawString("Rood", 100, 100);
        g2d.setColor(Color.GREEN);
        g2d.drawString("Groen", 100, 200);
        g2d.setColor(Color.BLUE);
        g2d.drawString("Blauw", 100, 300);
    }

    public short[] getRow(int x) {
        final short[] data = ((DataBufferUShort) buffer.getRaster().getDataBuffer()).getData();
        final short[] range = Arrays.copyOfRange(data, x * WIDTH, x * WIDTH + WIDTH);
        return range;
//        short[] transformed = new short[range.length];
//        for (int i = 0; i < range.length; i++) {
//            final short b1 = (short) ((range[i] & 0xff00) >> 8);
//            final short b2 = (short) ((range[i] & 0x00ff) << 8);
//            range[i] = (short) (b1 | b2);
//
//            final short b = (short) ((range[i] & ((short) 0b111110000000000)) >> 5); // blauw
//            final short r = (short) ((range[i] & ((short) 0b0000011111100000)) << 5); // rood
//            final short g = (short) ((range[i] & ((short) 0b0000000000011111))); // groen?
//            transformed[i] = (short) (r | g | b);
//        }
//        return transformed;
    }
}
