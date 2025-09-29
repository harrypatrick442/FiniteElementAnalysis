using Core.FileSystem;
using FiniteElementAnalysis.Boundaries;
using FiniteElementAnalysis.MeshGeneration;
using FiniteElementAnalysis.Polyhedrals;
using FiniteElementAnalysis;
using FiniteElementAnalysis.Solvers;
using Core.Enums;
using FiniteElementAnalysis.SourceRegions;
using FiniteElementAnalysis.Fields;
using InfernoDispatcher;
using Core.MemoryManagement;
using Shutdown;
using Logging;
using FiniteElementAnalysis.Ply;
using FiniteElementAnalysis.CloudCompare;
using Core.Maths.Tensors;
using FiniteElementAnalysis.Boundaries.Magnetic;
using FiniteElementAnalysis.Results;
using FiniteElementAnalysis.Mesh.Tetrahedral;
using FiniteElementAnalysis.Mesh.Generation;
using FiniteElementAnalysis.Boundaries.Statics;

namespace BuckingFieldExperimentation
{
    // Example usage:
    public class Program
    {
        /*
         * https://wias-berlin.de/software/tetgen/fformats.html
         * */
        public static void Main(string[] args)
        {
            ShutdownManager.Initialize(Environment.Exit, () => Logs.Default);
            GpuMemoryInfoNVML.Initialize();
            Dispatcher.InitializeWithNative(Console.WriteLine);
            //secondaryCurrent / 
            //  (8/*bobin segments*/* 0.005d * 0.005d);
            const string OPERATION_1 = "Operation1",
                MATERIAL_BOUNDARY = "MaterialBoundary",
                FIXED_FORCE_BOUNDARY = "FixedForceBoundary",
                MATERIAL_VOLUME = "MaterialVolume",
                FIXED_DISPLACEMENT_BOUNDARY = "FixedDisplacementBoundary";
            const double FORCE = 10000,
                S275_YOUNGS_MODULUS=210*1E+9, 
                S275_POISONS_RATIO=0.3;
            string OUTPUT_DIRECTORY = Path.Combine(DirectoryHelper.GetProjectDirectory(), "output");
            Directory.CreateDirectory(OUTPUT_DIRECTORY);
            string CANTILEVER_DISPLACEMENT_PLY_FILE_PATH = Path.Combine(OUTPUT_DIRECTORY, "displacements.ply");
            BoundariesCollection boundaries = new BoundariesCollection(
                new FreeBoundary(MATERIAL_BOUNDARY),
                new FixedNormalForceNeumannBoundary(FIXED_FORCE_BOUNDARY, FORCE),
                new FixedDisplacementDirichletBoundary(FIXED_DISPLACEMENT_BOUNDARY, new double[] { 0, 0, 0 }, new double[] { 0, 0, 0 })
            );
            double maximumTetrahedronVolumeConstraint = 10e-9;
            VolumesCollection volumes = new VolumesCollection(
                new StaticLinearElasticVolume(MATERIAL_VOLUME, S275_YOUNGS_MODULUS, S275_POISONS_RATIO,
                maximumTetrahedronVolumeConstraint)
            );
            PolyhedralDomain domain = ObjFileToPoly.Read(
                //File.ReadAllBytes("C:\\repos\\snippets\\CircuitAnalysis\\VoltageMultiplier\\Meshes\\TestWindings.obj")
                File.ReadAllBytes("C:\\repos\\snippets\\CircuitAnalysis\\LinearStaticAnalysisExample2Beam\\Models\\Beam.obj"), 
                volumes,
                boundaries,
                out Dictionary<int, Boundary> mapMarkerToBoundary, 
                Units.Millimeters,
                0.001d);
            string tempDirectoryPath = "D:\\temp\\";
            foreach (var b in mapMarkerToBoundary.Values) {
                Console.WriteLine(b.Name);
            }
            using (TemporaryDirectory temporaryDirectory = TemporaryDirectory
                .InCustomTempDirectory(tempDirectoryPath))
            {
                using (TemporaryWorkingDirectoryManager workingDirectoryManager = new TemporaryWorkingDirectoryManager(
                        Path.Combine(temporaryDirectory.AbsolutePath, "workings")))
                {
                    Console.WriteLine($"Output to: \"{OUTPUT_DIRECTORY}\"");
                    DirectoryHelper.DeleteRecursively(OUTPUT_DIRECTORY, throwOnError: false);
                    string polyFilePath = Path.Combine(temporaryDirectory.AbsolutePath, "mesh.poly");
                    PolyFileGenerator.Generate(polyFilePath, domain);
                    Tetgen.CopyTetViewToDirectory(temporaryDirectory.AbsolutePath);
                    using (Tetgen tetgen = new Tetgen())
                    {
                        TetgenGenerateMeshResult generateMeshResult = tetgen.GenerateTetrahedralMesh(
                            polyFilePath,
                            new TetgenParameters
                            {
                                RefineMesh = true,
                                MaximumTetrahedralVolumeConstraint = maximumTetrahedronVolumeConstraint,
                                CheckConsistencyOfFinalMesh = true
                            });
                        Console.WriteLine(generateMeshResult.Output);
                        if (generateMeshResult.ExitCode != 0)
                        {
                            string moreExceptionInfo = generateMeshResult.GetMoreExceptionInfo();
                            Console.WriteLine(moreExceptionInfo);
                            throw new Exception(generateMeshResult.Output);
                        }
                        TetrahedralMesh mesh = generateMeshResult.ToMesh(boundaries, volumes, mapMarkerToBoundary);
                        PlyWriter.Write(Path.Combine(OUTPUT_DIRECTORY, "mesh.ply"), mesh);
                        var volumeElements = mesh.Elements.GroupBy(e => e.VolumeName).Select(g => g.ToArray()).ToArray();
                        LinearStaticAnalysisSolver solver = new LinearStaticAnalysisSolver();
                        LinearStaticAnalysisResult result
                         = solver.Solve(
                            mesh,
                            workingDirectoryManager,
                            OPERATION_1,
                            cachedSolverResult: null);

                        result.Print();

                        PlyWriter.Write(
                            CANTILEVER_DISPLACEMENT_PLY_FILE_PATH,
                            mesh,
                            new FieldResult[] {
                                new VectorFieldResult("displacement", result.Displacements, includeMagnitude:true),
                            }
                        );
                    }
                }

            }
        }

    }
}