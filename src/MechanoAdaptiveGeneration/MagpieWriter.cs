using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MechanoAdaptiveGeneration
{
    public class MagpieWriter
    {

        public MagpieWriter()
        {

        }

        public void write(string outputPath, MagpieResults results)
        {
            string[] resultsString = new string[1];
            resultsString[0] = results.ToString();
            System.IO.File.WriteAllLines(outputPath, resultsString);
        }
    }
}