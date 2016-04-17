using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration.customK2goals
{
    public class EllipsoidCollide // No longer a Goal object itself. This can detect collision and output info then used for the actual goal
    {
        Ellipsoid e1;
        Ellipsoid e2;

        public double overlap;
        public double extension;

        public EllipsoidCollide(Ellipsoid _e1, Ellipsoid _e2)
        {
            e1 = _e1;
            e2 = _e2;
        }

        public double CalculateCollision()
        {
            e1.calculateAxesAndScales();
            e2.calculateAxesAndScales();

            //calculate distance vectors back and forth, and the length of the distance
            //this will become part of the narrow phase check
            Vector3d pjk = new Vector3d(e2.position - e1.position);
            double pjkLength = pjk.Length;
            Vector3d pkj = new Vector3d(-pjk);

            //store direction before transforming distance vectors
            Vector3d pushVector = new Vector3d(pjk);

            //Transform connection vector onto "sphere" coordinates of either ellipsoid
            e1.calculateInverse();
            e2.calculateInverse();
            //find length of vector inside each ellipsoid
            double jDistance = e1.findDistanceInside(pjk);
            double kDistance = e2.findDistanceInside(pkj);

            double touchingDistance = jDistance + kDistance;
            if (pjkLength - (touchingDistance) < 0)
            {
                return touchingDistance;
            }
            else
            {
                return -1;
            }

            //      //longest overlap is:
            //      extension = pjkLength - (jDistance + kDistance);
            //
            //      //scalar w quantifying overlap in ]0,1]. w = 1 means no overlap, w close to 0 means a lot of overlap
            //      double overlap = pjkLength / (jDistance + kDistance);
            //
            //      if(overlap <= 1.0 && overlap > 0)
            //      {
            //        pushVector.Unitize();
            //        return extension * pushVector;
            //      } else
            //      {
            //        return Vector3d.Zero;
            //      }

        }
    }
}
