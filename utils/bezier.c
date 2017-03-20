#include "bezier.h"

// lirp = linear interpolation
// pOut = (1 - t)*p1 + t*p2
void lirp(float t, Point *p1, Point *p2, Point *pOut)
{
    pOut->x = (1-t)*p1->x + t*p2->x;
    pOut->y = (1-t)*p1->y + t*p2->y;
}

// p0 is implicitly 0,0. p1 is the near line camera point, p2 is the far line
// camera point. pOut is the output
void Bezier(float t, Point *p1, Point *p2, Point *pOut)
{
    // p0 is the 0 of the coordinate system
    // located at the center of the front of the car
    Point p0;
    p0.x = 0; p0.y = 0;
    Point inter1, inter2;


    lirp(t, &p0, p1, &inter1);
    lirp(t, p1, p2, &inter2);

    lirp(t, &inter1, &inter2, pOut);
}


