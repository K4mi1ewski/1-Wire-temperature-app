#ifndef DRAWING_H
#define DRAWING_H

#include <stdint.h>

// Global variable for scaling the axis
extern volatile int MAX_Y;

// ------------------------------------------
// Declarations of drawing functions
// ------------------------------------------

/**
 * @brief DrawString
 * Draws the provided string on the LCD display.
 * @param string - text to be displayed
 * @param x, y - coordinates of the starting point (starting point)
 */
void DrawString(const char *string, int x, int y);

/**
 * @brief DrawRect
 * Draws a filled rectangle.
 * @param _x1, _y1, _x2, _y2 - coordinates of the rectangle's corners
 * @param colour - fill color
 */
void DrawRect(int _x1, int _y1, int _x2, int _y2, int colour);

/**
 * @brief DrawLine
 * Draws a line between two points.
 * @param y1, x1, y2, x2 - coordinates of the starting and ending points
 * @param color - line color
 */
void DrawLine(int y1, int x1, int y2, int x2, int color);

/**
 * @brief ColorBackground
 * Fills the entire background with the selected color.
 * @param color - background color
 */
void ColorBackground(int color);

/**
 * @brief DrawScale
 * Draws the axes and the basic grid of the graph.
 */
void DrawScale();

/**
 * @brief display_graph
 * Displays a point (e.g., a temperature measurement) on the graph.
 * @param val - measured value (e.g., temperature)
 * @param iter - current position on the Y-axis (e.g., successive measurements)
 * @param colour - color of the point/display
 */
void display_graph(int val, int iter, int colour);

#endif