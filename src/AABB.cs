using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    //TODO: check whether we can use inbuild AABB
    public class Aabb
    {
        public EndPoint[] Min;//an array of size 3 with the x,y,z value for the AABB min
        public EndPoint[] Max;//an array of size 3 with the x,y,z value for the AABB max
        public int BranchRef;

        //constructor
        public Aabb(Point3d tMin, Point3d tMax, int tBranchRef)
        {
            Min = new EndPoint[] { new EndPoint(tMin.X, true, this), new EndPoint(tMin.Y, true, this), new EndPoint(tMin.Z, true, this) };
            Max = new EndPoint[] { new EndPoint(tMax.X, false, this), new EndPoint(tMax.Y, false, this), new EndPoint(tMax.Z, false, this) };
            BranchRef = tBranchRef;
        }
    }
}
