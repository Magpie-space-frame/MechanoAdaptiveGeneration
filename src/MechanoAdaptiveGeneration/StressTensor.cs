using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{

    /// <summary>
    /// Data structure to handle a stress tensor
    /// </summary>
    public class StressTensor
    {
        public double[] Values;

        public StressTensor()
        {
            Values = new Double[9] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        }

        public StressTensor(List<double> vals)
        {
            Values = vals.ToArray();
        }
    }
}
