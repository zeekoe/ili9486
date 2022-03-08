package nl.ronaldteune.ili9486;

import com.sun.jna.Callback;
import com.sun.jna.Library;
import com.sun.jna.Pointer;

public interface Ili9486 extends Library {

	void initDisplay();
	void deInitDisplay();

    void drawRow(int y, short[] data);
    void drawRowRaw(int y, short[] data);

}
