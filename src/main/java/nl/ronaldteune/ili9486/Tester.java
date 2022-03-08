package nl.ronaldteune.ili9486;

import com.sun.jna.Native;

import java.sql.Time;

public class Tester {
    public static void main(String[] args) throws InterruptedException {
        System.setProperty("jna.debug_load", "false");
        System.setProperty("jna.debug_load.jna", "false");

        final Ili9486 ili9486 = Native.load("/usr/local/lib/ili9486.so", Ili9486.class);
        ili9486.initDisplay();

        short[] row = new short[480];

        try {
            while (true) {
                for(int i = 0; i < 480; i++) {
                    row[i] = (short) 0b1000000000000000;
                }
                for (int i = 0; i < 320; i++) {
                    ili9486.drawRowRaw(i, row);
                }

                Thread.sleep(100);
            }
        } catch (Throwable e) {
            e.printStackTrace();
            ili9486.deInitDisplay();
        }
    }
}
