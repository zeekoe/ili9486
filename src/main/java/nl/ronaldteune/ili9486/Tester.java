package nl.ronaldteune.ili9486;

import com.sun.jna.Native;

import java.sql.Time;

public class Tester {
    public static void main(String[] args) throws InterruptedException {
        System.setProperty("jna.debug_load", "false");
        System.setProperty("jna.debug_load.jna", "false");

        final Ili9486 ili9486 = Native.loadLibrary("/usr/local/lib/ili9486.so", Ili9486.class);
        ili9486.initDisplay();

        final Img img = new Img();
        for (int i = 0; i < 320; i++) {
            ili9486.drawRow(i, img.getRow(i));
        }

        Thread.sleep(3000);
        ili9486.deInitDisplay();
    }
}
