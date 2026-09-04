#include <cmath>

extern "C" {
#include "inc_irit/irit_sm.h"
#include "inc_irit/iritprsr.h"
#include "inc_irit/cagd_lib.h"
//#include "inc_irit/bsp_lib.h"   
#include "inc_irit/misc_lib.h"  
#include "inc_irit/geom_lib.h"  
}

// Generates the spiral fidget spinner surfaces mathematically without relying on SweepSrf
CagdSrfStruct* GenerateFidgetSpinnerArms() {
    int vSteps = 40;       // Steps along the spiral path (Length V)
    int uSteps = 30;       // Steps around the circular profile (Length U)
    int numV = vSteps + 1;
    int numU = uSteps + 1;

    double a = 15.0;
    double b = 3.0;
    double maxTheta = 1.5 * M_PI;
    double radius = 5.0;

    // 1. Allocate a blank B-Spline Surface (U = Profile, V = Path, Order = 4 for Cubic)
    // Change this line:
    CagdSrfStruct* arm1 = IritCagdBspSrfNew(numU, numV, 4, 4, CAGD_PT_E3_TYPE);
    IritCagdBspKnotUniformOpen(numU, 4, arm1->UKnotVector);
    IritCagdBspKnotUniformOpen(numV, 4, arm1->VKnotVector);

    // 2. Mathematically evaluate and set every control point on the surface grid
    for (int v = 0; v <= vSteps; ++v) {
        double theta = (static_cast<double>(v) / vSteps) * maxTheta;
        double r = a + b * theta;

        // Path center point
        double cx = r * cos(theta);
        double cy = r * sin(theta);

        // Path tangent (derivative of the Archimedean spiral)
        double dx = b * cos(theta) - r * sin(theta);
        double dy = b * sin(theta) + r * cos(theta);
        double len = sqrt(dx * dx + dy * dy);

        // Normal vector (perpendicular to the tangent in the XY plane)
        double nx = -dy / len;
        double ny = dx / len;

        for (int u = 0; u <= uSteps; ++u) {
            double phi = (static_cast<double>(u) / uSteps) * 2.0 * M_PI;

            // Calculate the 3D offset from the path center using the Normal and Z axes
            double offsetX = nx * radius * cos(phi);
            double offsetY = ny * radius * cos(phi);
            double offsetZ = radius * sin(phi);

            // IRIT stores surface points in a 1D array where U changes fastest
            int idx = u + (v * numU);

            arm1->Points[1][idx] = cx + offsetX;
            arm1->Points[2][idx] = cy + offsetY;
            arm1->Points[3][idx] = offsetZ;
        }
    }

    // 3. Create the other two arms via 120 and 240 degree rotations on the Z-axis
    IrtHmgnMatType rotZ120, rotZ240;
    IritMiscMatGenMatRotZ1(120.0 * M_PI / 180.0, rotZ120);
    IritMiscMatGenMatRotZ1(240.0 * M_PI / 180.0, rotZ240);

    CagdSrfStruct* arm2 = IritCagdSrfMatTransform(arm1, rotZ120);
    CagdSrfStruct* arm3 = IritCagdSrfMatTransform(arm1, rotZ240);

    // Link the surfaces together into a list
    arm1->Pnext = arm2;
    arm2->Pnext = arm3;

    return arm1;
}