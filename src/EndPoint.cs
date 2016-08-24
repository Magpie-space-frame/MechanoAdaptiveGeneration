using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{

    public class EndPoint
    {
        public AABB owner;
        public double num;//its actual value - corresponds to the x,y or z value
        public bool isMin;//is this a min point or a max point?

        //constructor
        public EndPoint(double tNum, bool tIsMin, AABB tOwner)
        {
            num = tNum;
            isMin = tIsMin;
            owner = tOwner;
        }

        //used as a comparison to sort the endpoints
        public static int compareEndPoints(EndPoint x, EndPoint y)
        {
            return x.num.CompareTo(y.num);
        }
    }
}
