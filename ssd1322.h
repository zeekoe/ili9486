//
// Created by zeekoe on 19-02-22.
//

#ifndef SSD1322_SSD1322_H
#define SSD1322_SSD1322_H

void initDisplay();

void deInitDisplay();

void drawRow(int y, const unsigned short *dataBufPtr);

#endif //SSD1322_SSD1322_H
