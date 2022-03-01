package nl.ronaldteune.ili9486;

import com.sun.jna.Native;

import java.sql.Time;

public class Tester {
    public static void main(String[] args) throws InterruptedException {
        System.setProperty("jna.debug_load", "false");
        System.setProperty("jna.debug_load.jna", "false");

        final Ili9486 ili9486 = Native.loadLibrary("/usr/local/lib/ili9486.so", Ili9486.class);
        ili9486.initDisplay();
        Thread.sleep(2000);
        final short[] data = new short[480];
        for (short i = 0; i < data.length; i++) {
            data[i] = (short) (12345 - i * 10);
        }
        for(int i = 0; i < 320; i++) {
            ili9486.drawRow(i, data);
        }

        Thread.sleep(3000);
        ili9486.deInitDisplay();
    }
}