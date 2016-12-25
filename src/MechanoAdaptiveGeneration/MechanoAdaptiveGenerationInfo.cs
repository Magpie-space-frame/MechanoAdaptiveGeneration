using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace MechanoAdaptiveGeneration
{
    /// <summary>
    /// GH_AssemblyInfo for the MAG plugin
    /// </summary>
    public class MechanoAdaptiveGenerationInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "MechanoAdaptiveGeneration";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("c6f951cc-616f-41e9-b833-444e57114d73");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
