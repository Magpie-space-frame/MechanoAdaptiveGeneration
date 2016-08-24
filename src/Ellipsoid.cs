using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    public class Ellipsoid
    {

        //centre point of the ellipsoid
        public Point3d position;
        //transform matrix that defines the scale and rotation make 3x3 (but really a 1x9)
        double[] transform;
        //the inverse transform this is calculated once and then referenced..
        double[] inverse;

        //unitized vectors for the axis
        //perhaps store the squared values as they are
        public Vector3d unitXAxis;
        double scaleX;
        public Vector3d unitYAxis;
        double scaleY;
        public Vector3d unitZAxis;
        double scaleZ;

        //points defining the bounding box
        public Point3d boundingBoxMin;
        public Point3d boundingBoxMax;

        //constructor
        public Ellipsoid(Point3d tPos)
        {
            //set the values use Vector3d.Axis and caluate bounding box function
            position = tPos;

            //set the transfrom as an identity matrix
            transform = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            inverse = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

            //could use calculateAxes but a waste of three unitize calls!
            unitXAxis = Vector3d.XAxis;
            scaleX = 1.0;
            unitYAxis = Vector3d.YAxis;
            scaleY = 1.0;
            unitZAxis = Vector3d.ZAxis;
            scaleZ = 1.0;

            calculateBoundingBox();
        }

        //takes an set of vectors and updates the transform - taken from the eigenvectors of the interpolated field
        public void updateTransform(Vector3d xAxis, Vector3d yAxis, Vector3d zAxis)
        {
            transform[0] = xAxis[0];
            transform[3] = xAxis[1];
            transform[6] = xAxis[2];

            transform[1] = yAxis[0];
            transform[4] = yAxis[1];
            transform[7] = yAxis[2];

            transform[2] = zAxis[0];
            transform[5] = zAxis[1];
            transform[8] = zAxis[2];
        }

        public void scale(double s)
        {
            for (int i = 0; i < 9; i++)
            {
                transform[i] = s * transform[i];
            }
        }

        public void calculateInverse()
        {
            inverse[0] = transform[0] / (scaleX * scaleX);
            inverse[1] = transform[3] / (scaleX * scaleX);
            inverse[2] = transform[6] / (scaleX * scaleX);

            inverse[3] = transform[1] / (scaleY * scaleY);
            inverse[4] = transform[4] / (scaleY * scaleY);
            inverse[5] = transform[7] / (scaleY * scaleY);

            inverse[6] = transform[2] / (scaleZ * scaleZ);
            inverse[7] = transform[5] / (scaleZ * scaleZ);
            inverse[8] = transform[8] / (scaleZ * scaleZ);
        }

        //calculate and return boundingBox
        public void calculateBoundingBox()
        {
            //takes a transform matrix representation of an ellipsoid and returns two points defining the min x,y,z and max x,y,x
            double[] minAndMaxX = getMinAndMax(position.X, 0);
            double[] minAndMaxY = getMinAndMax(position.Y, 3);
            double[] minAndMaxZ = getMinAndMax(position.Z, 6);

            boundingBoxMin = new Point3d(minAndMaxX[0], minAndMaxY[0], minAndMaxZ[0]);
            boundingBoxMax = new Point3d(minAndMaxX[1], minAndMaxY[1], minAndMaxZ[1]);


        }

        //method used by the calculate bounding box method
        private double[] getMinAndMax(double position, int adder)
        {
            double m11 = transform[adder];
            double m12 = transform[adder + 1];
            double m13 = transform[adder + 2];
            double d = Math.Sqrt(m11 * m11 + m12 * m12 + m13 * m13);
            double[] minMax = { position - d, position + d };
            return minMax;
        }

        //set updated unit vectors for
        public void calculateAxesAndScales()
        {
            unitXAxis = new Vector3d(transform[0], transform[3], transform[6]);
            scaleX = unitXAxis.Length;
            unitXAxis.Unitize();
            unitYAxis = new Vector3d(transform[1], transform[4], transform[7]);
            scaleY = unitYAxis.Length;
            unitYAxis.Unitize();
            unitZAxis = new Vector3d(transform[2], transform[5], transform[8]);
            scaleZ = unitZAxis.Length;
            unitZAxis.Unitize();
        }

        //combines the postion and the transform to create an Affine transform
        public double[] createAffineTransform()
        {
            return new double[] { transform[0], transform[1], transform[2], position[0], transform[3], transform[4], transform[5], position[1], transform[6], transform[7], transform[8], position[2], 0, 0, 0, 1 };
        }

        //using similarity ratios: 1/transformedLength = distanceInside/inputlegth
        public double findDistanceInside(Vector3d inputVector)
        {
            Vector3d transformed = transformVector(inputVector);
            return inputVector.Length / transformed.Length;
        }

        //method used by findDistanceInside method
        private Vector3d transformVector(Vector3d toTransform)
        {
            Vector3d transformedVector = new Vector3d();
            transformedVector[0] = inverse[0] * toTransform[0] + inverse[1] * toTransform[1] + inverse[2] * toTransform[2];
            transformedVector[1] = inverse[3] * toTransform[0] + inverse[4] * toTransform[1] + inverse[5] * toTransform[2];
            transformedVector[2] = inverse[6] * toTransform[0] + inverse[7] * toTransform[1] + inverse[8] * toTransform[2];
            return transformedVector;
        }
    }
}
