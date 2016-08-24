using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    class HelperFunctions
    {
        public static double StdDev(ref List<double> values)
        {
            double ret = 0;
            int count = values.Count();
            if (count > 1)
            {
                //Compute the Average
                double avg = values.Average();

                //Perform the Sum of (value-avg)^2
                double sum = values.Sum(d => (d - avg) * (d - avg));

                //Put it all together
                ret = Math.Sqrt(sum / count);
            }
            return ret;
        }

        //evaluate % total volume taken up by ellipsoids, and correct the scaling factor if needed
        public static void updateScaleByVolume(ref double scale, ref double totalEllipsoidVolume, ref List<Ellipsoid> ellis, ref double meshVol)
        {
            double targetEllipsoidVolume = meshVol;
            scale = scale * Math.Pow(targetEllipsoidVolume / totalEllipsoidVolume, 1.0 / 3.0);
            for (int i = 0; i < ellis.Count(); i++)
            {
                ellis[i].scale(scale);
            }
        }

        public static void processData(List<double> dataToProcess, ref BackGroundData backGroundData, ref StressTensor[,,] grid)
        {
            backGroundData.G = new List<Point3d>();
            backGroundData.T0 = new List<double>();
            backGroundData.T1 = new List<double>();
            backGroundData.T2 = new List<double>();
            backGroundData.T3 = new List<double>();
            backGroundData.T4 = new List<double>();
            backGroundData.T5= new List<double>();
                                     
            while (dataToProcess.Any())
            {
                List<double> localDataList = dataToProcess.Take(9).ToList();

                backGroundData.G.Add(new Point3d(localDataList[0], localDataList[1], localDataList[2]));

                backGroundData.T0.Add(localDataList[3]);
                backGroundData.T1.Add(localDataList[4]);
                backGroundData.T2.Add(localDataList[5]);
                backGroundData.T3.Add(localDataList[6]);
                backGroundData.T4.Add(localDataList[7]);
                backGroundData.T5.Add(localDataList[8]);

                dataToProcess = dataToProcess.Skip(9).ToList();
            }

            //find grid count and size
            backGroundData.xCount = backGroundData.yCount = backGroundData.zCount = 0;
            backGroundData.xSize = backGroundData.ySize = backGroundData.zSize = 0;

            StressTensor[] Tensors = new StressTensor[backGroundData.G.Count];
            for (int i = 0; i < backGroundData.G.Count; i++)
            {
                Tensors[i] = new StressTensor(new List<double> { backGroundData.T0[i], backGroundData.T1[i], backGroundData.T2[i], backGroundData.T1[i], backGroundData.T3[i], backGroundData.T4[i], backGroundData.T2[i], backGroundData.T4[i], backGroundData.T5[i] });
            }

            Array.Sort(backGroundData.G.ToArray(), Tensors);
            backGroundData.G.Sort();

            backGroundData.zSize = backGroundData.G[1].Z - backGroundData.G[0].Z;

            for (int i = 1; i < backGroundData.G.Count; i++)
            {
                if (backGroundData.G[i].Y > backGroundData.G[0].Y)
                {
                    backGroundData.ySize = backGroundData.G[i].Y - backGroundData.G[0].Y;
                    backGroundData.zCount = i;
                    break;
                }
            }

            for (int i = 1; i < backGroundData.G.Count; i++)
            {
                if (backGroundData.G[i].X > backGroundData.G[0].X)
                {
                    backGroundData.xSize = backGroundData.G[i].X - backGroundData.G[0].X;
                    backGroundData.yCount = i / backGroundData.zCount;
                    break;
                }
            }

            backGroundData.xCount = backGroundData.G.Count / (backGroundData.zCount * backGroundData.yCount);

            grid = new StressTensor[backGroundData.xCount, backGroundData.yCount, backGroundData.zCount];

            for (int i = 0; i < backGroundData.xCount; i++)
            {
                for (int j = 0; j < backGroundData.yCount; j++)
                {
                    for (int k = 0; k < backGroundData.zCount; k++)
                    {
                        grid[i, j, k] = Tensors[k + j * backGroundData.zCount + i * backGroundData.zCount * backGroundData.yCount];
                    }
                }
            }
        }

        public static void SweepAndPrune(List<Ellipsoid> Ellipsoids, Point3d[] Positions, ref List<int> CollideRef0, ref List<int> CollideRef1)
        {
            List<AABB> boxes = new List<AABB>();

            //use the points to create AABB objects
            for (int i = 0; i < Ellipsoids.Count; i++)
            {
                Ellipsoids[i].position = Positions[i];
                Ellipsoids[i].calculateBoundingBox();

                boxes.Add(new AABB(Ellipsoids[i].boundingBoxMin, Ellipsoids[i].boundingBoxMax, i));
            }

            List<EndPoint> sortedInX = new List<EndPoint>();

            //add the endpoints in x
            foreach (AABB boxForPoint in boxes)
            {
                sortedInX.Add(boxForPoint.min[0]);//change this 0 to 1 or 2 to sort in Y or Z - will also need to change the second half of the test function
                sortedInX.Add(boxForPoint.max[0]);//change this 0 to 1 or 2 to sort in Y or Z
            }

            //sort by the num value of each EndPoint
            sortedInX.Sort(EndPoint.compareEndPoints);

            List<int> openBoxes = new List<int>();

            foreach (EndPoint endPoint in sortedInX)
            {
                if (endPoint.isMin)
                {
                    AABB thisPointOwner = endPoint.owner;
                    //check against all in openBoxes
                    foreach (int openBoxRef in openBoxes)
                    {
                        //if it collides output the integers of the branches than collide
                        //do they collide in y?
                        if (thisPointOwner.max[1].num > boxes[openBoxRef].min[1].num && thisPointOwner.min[1].num < boxes[openBoxRef].max[1].num)
                        {
                            //they collide in y, do they collide in z?
                            if (thisPointOwner.max[2].num > boxes[openBoxRef].min[2].num && thisPointOwner.min[2].num < boxes[openBoxRef].max[2].num)
                            {
                                //they collide in z
                                //therefore they collide! Add to the list of collide refs
                                CollideRef0.Add(endPoint.owner.branchRef);
                                CollideRef1.Add(boxes[openBoxRef].branchRef);
                            }
                        }
                    }
                    //add corresponding box to openBoxes
                    openBoxes.Add(thisPointOwner.branchRef);

                }
                else
                {
                    //it must be an max point
                    //remove corresponding box from openBoxes
                    openBoxes.Remove(endPoint.owner.branchRef);
                }
            }
        }

        public static void InterpolateTensor(Point3d[] Pts, ref List<Vector3d> Evec1, ref List<Vector3d> Evec2, ref List<Vector3d> Evec3, ref List<double> Eval1, ref List<double> Eval2, ref List<double> Eval3, StressTensor[,,] Grid, BackGroundData backGroundData)
        {
            int nOfPoints = Pts.Count();

            var EigenVectors = new Vector3d[nOfPoints][];
            var EigenValues = new Double[nOfPoints][];

            var EVA = new Vector3d[nOfPoints];
            var EVB = new Vector3d[nOfPoints];
            var EVC = new Vector3d[nOfPoints];
            var EigenValuesA = new double[nOfPoints];
            var EigenValuesB = new double[nOfPoints];
            var EigenValuesC = new double[nOfPoints];

            // for each point in P, find its coordinates in the grid
            // output the 8 points/indices of the corners
            // and the point coordinates of the position within that cell

            var arrayOfCellCoords = new Point3d[nOfPoints]; // always in [0,1]^3
            var arrayOfTensors0 = new double[nOfPoints][];
            var arrayOfTensors1 = new double[nOfPoints][];
            var arrayOfTensors2 = new double[nOfPoints][];
            var arrayOfTensors3 = new double[nOfPoints][];
            var arrayOfTensors4 = new double[nOfPoints][];
            var arrayOfTensors5 = new double[nOfPoints][];
            var arrayOfTensors6 = new double[nOfPoints][];
            var arrayOfTensors7 = new double[nOfPoints][];

            System.Threading.Tasks.Parallel.For(0, nOfPoints,
              j =>
              {
                  var PVec = Pts[j] - backGroundData.G[0];  //position vector of the current point relative to grid origin

                  var PX = PVec.X / backGroundData.xSize;
                  var PY = PVec.Y / backGroundData.ySize;
                  var PZ = PVec.Z / backGroundData.zSize;

                  int PXF, PXC, PYF, PYC, PZF, PZC;

                  var CoordsInCell = new Point3d();

                  if (PX < 0)
                  {
                      PXF = 0; PXC = 1; CoordsInCell.X = 0;
                  }
                  else if (PX > backGroundData.xCount - 1)
                  {
                      PXF = backGroundData.xCount - 2; PXC = backGroundData.xCount - 1; CoordsInCell.X = 1;
                  }
                  else
                  {
                      PXF = (int)Math.Floor(PX); PXC = (int)Math.Ceiling(PX); CoordsInCell.X = PX - PXF;
                  }


                  if (PY < 0)
                  {
                      PYF = 0; PYC = 1; CoordsInCell.Y = 0;
                  }
                  else if (PY > backGroundData.yCount - 1)
                  {
                      PYF = backGroundData.yCount - 2; PYC = backGroundData.yCount - 1; CoordsInCell.Y = 1;
                  }
                  else
                  {
                      PYF = (int)Math.Floor(PY); PYC = (int)Math.Ceiling(PY); CoordsInCell.Y = PY - PYF;
                  }


                  if (PZ < 0)
                  {
                      PZF = 0; PZC = 1; CoordsInCell.Z = 0;
                  }
                  else if (PZ > backGroundData.zCount - 1)
                  {
                      PZF = backGroundData.zCount - 2; PZC = backGroundData.zCount - 1; CoordsInCell.Z = 1;
                  }
                  else
                  {
                      PZF = (int)Math.Floor(PZ); PZC = (int)Math.Ceiling(PZ); CoordsInCell.Z = PZ - PZF;
                  }


                  //
                  //      int PXF = Math.Max(Math.Min((int) Math.Floor(PX), xCount - 1), 0);
                  //      int PYF = Math.Max(Math.Min((int) Math.Floor(PY), yCount - 1), 0);
                  //      int PZF = Math.Max(Math.Min((int) Math.Floor(PZ), zCount - 1), 0);
                  //
                  //      int PXC = Math.Max(Math.Min((int) Math.Ceiling(PX), xCount - 1), 0);
                  //      int PYC = Math.Max(Math.Min((int) Math.Ceiling(PY), yCount - 1), 0);
                  //      int PZC = Math.Max(Math.Min((int) Math.Ceiling(PZ), zCount - 1), 0);

                  //     arrayOfCellCoords[j] = new Point3d(Math.Min(Math.Max(PX - PXF, 0), 1), Math.Min(Math.Max(PY - PYF, 0), 1), Math.Min(Math.Max(PZ - PZF, 0), 1));

                  arrayOfCellCoords[j] = CoordsInCell;

                  arrayOfTensors0[j] = Grid[PXF, PYF, PZF].Values;
                  arrayOfTensors1[j] = Grid[PXF, PYF, PZC].Values;
                  arrayOfTensors2[j] = Grid[PXF, PYC, PZF].Values;
                  arrayOfTensors3[j] = Grid[PXF, PYC, PZC].Values;
                  arrayOfTensors4[j] = Grid[PXC, PYF, PZF].Values;
                  arrayOfTensors5[j] = Grid[PXC, PYF, PZC].Values;
                  arrayOfTensors6[j] = Grid[PXC, PYC, PZF].Values;
                  arrayOfTensors7[j] = Grid[PXC, PYC, PZC].Values;

                  var Corners = new StressTensor[8];

                  Corners[0] = new StressTensor();
                  Corners[1] = new StressTensor();
                  Corners[2] = new StressTensor();
                  Corners[3] = new StressTensor();
                  Corners[4] = new StressTensor();
                  Corners[5] = new StressTensor();
                  Corners[6] = new StressTensor();
                  Corners[7] = new StressTensor();

                  Corners[0].Values = (double[])arrayOfTensors0[j];
                  Corners[1].Values = (double[])arrayOfTensors1[j];
                  Corners[2].Values = (double[])arrayOfTensors2[j];
                  Corners[3].Values = (double[])arrayOfTensors3[j];
                  Corners[4].Values = (double[])arrayOfTensors4[j];
                  Corners[5].Values = (double[])arrayOfTensors5[j];
                  Corners[6].Values = (double[])arrayOfTensors6[j];
                  Corners[7].Values = (double[])arrayOfTensors7[j];

                  var EVec = new Vector3d[3];
                  var EVal = new Double[3];

                  var T = new Double[3] { arrayOfCellCoords[j].X, arrayOfCellCoords[j].Y, arrayOfCellCoords[j].Z };
                  //var T = new Double[3]{0.5,0.5,0.5};

                  //THIS IS WHERE THE ACTUAL EIGENVALUE PROBLEM IS SOLVED
                  StressTensor tensorAtPoint = TriLinearInterpolate(Corners, T);
                  EigenSolve(tensorAtPoint.Values, out EVal, out EVec);

                  int[] ott = new int[3]; // o ne, t wo, t hree
                  ott[0] = 0;
                  ott[1] = 1;
                  ott[2] = 2;
                  double[] ev = new double[] { Math.Abs(EVal[0]), Math.Abs(EVal[1]), Math.Abs(EVal[2]) };
                  //sort according to maximum absolute value of Eval
                  Array.Sort(ev, ott);

                  EVA[j] = EVec[ott[2]];
                  EVB[j] = EVec[ott[1]];
                  EVC[j] = EVec[ott[0]];

                  EigenValuesA[j] = Math.Abs(EVal[ott[2]]);
                  EigenValuesB[j] = Math.Abs(EVal[ott[1]]);
                  EigenValuesC[j] = Math.Abs(EVal[ott[0]]);
              });

            Evec1 = EVA.ToList();
            Evec2 = EVB.ToList();
            Evec3 = EVC.ToList();
            Eval1 = EigenValuesA.ToList();
            Eval2 = EigenValuesB.ToList();
            Eval3 = EigenValuesC.ToList();
        }

        /// <summary>
        /// given stress tensors at the 8 corners of a cube, and coordinates of a point within that cube
        /// perform a trilinear interpolation
        /// </summary>
        /// <param name="CornerTensors">Tensor values at the corners, ordered 000,001,010,011,100,101,110,111</param>
        /// <param name="T">3 numbers in the range 0 to 1</param>
        /// <returns>the interpolated stress tensor at the given coordinates</returns>
        ///
        public static StressTensor TriLinearInterpolate(StressTensor[] CornerTensors, double[] T)
        {
            //first get the 4 values along the edges in the x direction
            StressTensor SX0 = LinearInterpolate(CornerTensors[0], CornerTensors[4], T[0]);
            StressTensor SX1 = LinearInterpolate(CornerTensors[1], CornerTensors[5], T[0]);
            StressTensor SX2 = LinearInterpolate(CornerTensors[2], CornerTensors[6], T[0]);
            StressTensor SX3 = LinearInterpolate(CornerTensors[3], CornerTensors[7], T[0]);

            //interpolate along Y
            StressTensor SY0 = LinearInterpolate(SX0, SX2, T[1]);
            StressTensor SY1 = LinearInterpolate(SX1, SX3, T[1]);

            //and finally along Z
            var ST = LinearInterpolate(SY0, SY1, T[2]);
            bool HasNaN = false;
            for (int i = 0; i < 9; i++)
            {
                if (Double.IsNaN(ST.Values[i])) { HasNaN = true; }
            }
            if (!HasNaN)
            {
                return LinearInterpolate(SY0, SY1, T[2]);
            }
            else
            {
                //if NaN then send in a large number..
                return new StressTensor(new List<double> { 1000, 0, 0, 0, 1000, 0, 0, 0, 1000 });
            }

        }

        public static StressTensor LinearInterpolate(StressTensor TensorA, StressTensor TensorB, double T)
        {
            var Interp = new StressTensor();
            int NaNorZeroCount = 0;
            for (int i = 0; i < 9; i++)
            {
                Interp.Values[i] = 0;
                if (!Double.IsNaN(TensorA.Values[i]))
                {
                    Interp.Values[i] += (1 - T) * TensorA.Values[i];
                }
                else {
                    Interp.Values[i] += (1 - T) * TensorB.Values[i];
                    NaNorZeroCount++;
                }
                if (!Double.IsNaN(TensorB.Values[i]))
                {
                    Interp.Values[i] += T * TensorB.Values[i];
                }
                else {
                    Interp.Values[i] += T * TensorA.Values[i];
                    NaNorZeroCount++;
                }

                if (NaNorZeroCount == 2)
                {
                    Interp.Values[i] = double.NaN;
                }
            }

            return Interp;
        }

        public static void EigenSolve(double[] x, out double[] EigenValues, out Vector3d[] EigenVectors)
        {
            double[,] matValues = new double[3, 3] { { x[0], x[1], x[2] }, { x[3], x[4], x[5] }, { x[6], x[7], x[8] } };
            double[,] OutMatValues = new double[3, 3];
            double[] OutVecValues = new double[3];
            Eigen eigenCalculate = new Eigen();

            eigenCalculate.eigen_decomposition(matValues, OutMatValues, OutVecValues);

            var EigenVectorsUnsorted = new Vector3d[3];

            if (eigenCalculate.calcDone)
            {
                EigenVectorsUnsorted[0] = new Vector3d(OutMatValues[0, 0], OutMatValues[1, 0], OutMatValues[2, 0]);
                EigenVectorsUnsorted[1] = new Vector3d(OutMatValues[0, 1], OutMatValues[1, 1], OutMatValues[2, 1]);
                EigenVectorsUnsorted[2] = new Vector3d(OutMatValues[0, 2], OutMatValues[1, 2], OutMatValues[2, 2]);
            }

            Array.Sort(OutVecValues, EigenVectorsUnsorted, new DescendingAbsoluteVals());
            EigenValues = OutVecValues;
            EigenVectors = EigenVectorsUnsorted;
        }

        public class DescendingAbsoluteVals : Comparer<double>
        {
            public override int Compare(double x, double y)
            {
                return (y * y).CompareTo(x * x);
            }
        }
    }
}
