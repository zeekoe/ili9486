package nl.ronaldteune.ili9486;

import com.sun.jna.Library;

public interface Ssd1322 extends Library {

	void initDisplay();
	void deInitDisplay();

    void drawRow(int y, short[] data);

}
