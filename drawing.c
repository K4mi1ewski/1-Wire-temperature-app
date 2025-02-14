#include "drawings.h"
#include "Open1768_LCD.h"
#include "LCD_ILI9325.h"
#include "asciiLib.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

// ------------------------------------------
// Global variable for scaling values on the axis
// ------------------------------------------
volatile int MAX_Y = 100; 

/**
 * @brief DrawString
 * Draws a string character by character on the display.
 * Each character is loaded from the ASCII table (asciiLib) and drawn pixel by pixel.
 */
void DrawString ( const char * string, int x, int y ) {
    unsigned char buffer[16];
    for (int k = 0; k < (int)strlen(string); k++) {
        GetASCIICode(1, buffer, string[k]);
        
        int xOffset, yOffset, _x, _y;
        xOffset = x;
        yOffset = y + k * 12;
        
        for (int i = 0; i < 16; i++) {
            _x = xOffset - i;
            for (int j = 0; j < 8; j++) {
                if (buffer[i] & (unsigned char)pow(2, j)) {
                    // Character pixel
                    _y = yOffset - j;
                    lcdWriteReg(ADRX_RAM, _x);
                    lcdWriteReg(ADRY_RAM, _y);
                    lcdWriteReg(DATA_RAM, LCDMagenta);
                } else {
                    // Character background
                    _y = yOffset - j;
                    lcdWriteReg(ADRX_RAM, _x);
                    lcdWriteReg(ADRY_RAM, _y);
                    lcdWriteReg(DATA_RAM, LCDBlueSea);
                }
            }
        }
    }
}

/**
 * @brief DrawRect
 * Draws a rectangle filled with a color.
 * It uses the provided coordinates and fills the area pixel by pixel.
 */
void DrawRect(const int _x1, const int _y1, const int _x2, const int _y2, const int colour) {
    int x1, x2, y1, y2;
    x1 = (_x2 > _x1) ? _x1 : _x2;
    x2 = (_x2 > _x1) ? _x2 : _x1;
    y1 = (_y2 > _y1) ? _y1 : _y2;
    y2 = (_y2 > _y1) ? _y2 : _y1;

    for (int i = x1; i < x2; i++) {
        for (int j = y1; j < y2; j++) {
            lcdWriteReg(ADRX_RAM, i);
            lcdWriteReg(ADRY_RAM, j);
            lcdWriteReg(DATA_RAM, colour);
        }
    }
}

/**
 * @brief DrawLine
 * Draws a line between two points (y1, x1) and (y2, x2).
 * The implementation is based on Bresenham's line algorithm.
 */
void DrawLine(const int y1, const int x1, const int y2, const int x2, const int color) {
    int d, dx, dy, ai, bi, xi, yi;
    int x = x1, y = y1;

    if (x1 < x2) { xi = 1; dx = x2 - x1; } else { xi = -1; dx = x1 - x2; }
    if (y1 < y2) { yi = 1; dy = y2 - y1; } else { yi = -1; dy = y1 - y2; }

    // Draw the first pixel
    lcdWriteReg(ADRX_RAM, x);
    lcdWriteReg(ADRY_RAM, y);
    lcdWriteReg(DATA_RAM, color);

    if (dx > dy) {
        ai = (dy - dx) * 2;
        bi = dy * 2;
        d = bi - dx;
        while (x != x2) {
            if (d >= 0) { x += xi; y += yi; d += ai; } else { d += bi; x += xi; }
            lcdWriteReg(ADRX_RAM, x);
            lcdWriteReg(ADRY_RAM, y);
            lcdWriteReg(DATA_RAM, color);
        }
    } else {
        ai = (dx - dy) * 2;
        bi = dx * 2;
        d = bi - dy;
        while (y != y2) {
            if (d >= 0) { x += xi; y += yi; d += ai; } else { d += bi; y += yi; }
            lcdWriteReg(ADRX_RAM, x);
            lcdWriteReg(ADRY_RAM, y);
            lcdWriteReg(DATA_RAM, color);
        }
    }
}

/**
 * @brief ColorBackground
 * Fills the entire background with a given color by iterating through all
 * pixels on the display.
 */
void ColorBackground(int color) {
    lcdWriteIndex(DATA_RAM);
    for (int i = 0; i < LCD_MAX_X; i++) {
        for (int j = 0; j < LCD_MAX_Y; j++) {
            lcdWriteData(color);
        }
    }
}

/**
 * @brief DrawScale
 * Draws the axes (X and Y) and the auxiliary ticks on the graph.
 */
void DrawScale() {
    // Osie
    DrawLine(5, 100, 310, 100, LCDWhite); // X axis
    DrawLine(35, 5, 35, 235, LCDWhite);   // Y axis

    // Y-axis ticks and temperature values
    int step = 10;
	int y_pos1 = 0;
	int y_pos2 = 0;

    for (int i = 0; i <= 5; i++) {
		y_pos1 = 100 + 10 * i * 300 / MAX_Y;
		y_pos2 = 100 - 10 * i * 300 / MAX_Y;

        //Wartosci na osi OY
        char temp_label[4];
		char temp_label2[4];
        sprintf(temp_label, "%d", step * i);
		sprintf(temp_label2, "%d", step * i * (-1));
        if (i < 5) DrawString(temp_label, y_pos1 + 7, 14);
		if (i < 4 & i > 0) DrawString(temp_label2, y_pos2 + 7, 5);
		DrawLine(33, y_pos1, 37, y_pos1, LCDWhite);
		DrawLine(33, y_pos2, 37, y_pos2, LCDWhite);
    }

     // Arrows at the ends of the axes
    DrawLine(302, 7 + 90, 310, 10 + 90, LCDWhite);
    DrawLine(302, 13 + 90, 310, 10 + 90, LCDWhite);
    DrawLine(32, 227, 35, 235, LCDWhite);
    DrawLine(38, 227, 35, 235, LCDWhite);
}

/**
 * @brief display_graph
 * Draws a point (e.g., a temperature measurement) on the graph.
 * val - measurement value (e.g., temperature)
 * iter - index of the next measurement (along the Y-axis)
 * colour - color of the point
 * Scaling: currently MAX_Y defines the range on the X-axis.
 * Mapping: X ~ val, Y ~ iter.
 */
void display_graph(int val, int iter, int colour) {
    // X axis: 100 + (val * 300 / MAX_Y)
    // Y axis: 35 + iter
    int x = 100 + val * 300 / MAX_Y;
    int y = 35 + iter;
    if (y > 0 && y < 320 && x > 0 && x < 240) {
        DrawRect(x - 2, y, x, y + 3, colour);
    }
}