using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{

    /// <summary>
    /// Data structure to handle end points of axis-aligned bounding boxes
    /// </summary>
    public class EndPoint
    {
        public Aabb Owner;
        public double Num;//its actual value - corresponds to the x,y or z value
        public bool IsMin;//is this a min point or a max point?

        //constructor
        public EndPoint(double tNum, bool tIsMin, Aabb tOwner)
        {
            Num = tNum;
            IsMin = tIsMin;
            Owner = tOwner;
        }

        //used as a comparison to sort the endpoints
        public static int CompareEndPoints(EndPoint x, EndPoint y)
        {
            return x.Num.CompareTo(y.Num);
        }
    }
}
