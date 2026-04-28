package macro;

import java.io.*;
import java.util.*;

import star.base.neo.DoubleVector;
import star.base.neo.FJTask;
import star.base.neo.NamedObject;
import star.base.neo.StringVector;
import star.base.report.*;
import star.cadmodeler.*;
import star.common.*;
import star.flow.*;
import star.kwturb.KOmegaTurbulence;
import star.kwturb.KwAllYplusWallTreatment;
import star.kwturb.KwTurbSpecOption;
import star.kwturb.SstKwTurbModel;
import star.material.*;
import star.meshing.*;
import star.metrics.ThreeDimensionalModel;
import star.prismmesher.NumPrismLayers;
import star.prismmesher.PrismLayerStretching;
import star.prismmesher.PrismThickness;
import star.turbulence.*;
import star.coupledflow.CoupledFlowModel;
import star.keturb.*;
import star.vis.*;

public class Simple_Setup extends StarMacro {

    // Global Map to store your "Pre-Known" values
    private Map<String, Double> configMap = new HashMap<>();

    // Store the selected turbulence model (Default to k-epsilon)
    private String turbulenceModel = "k-epsilon";

    @Override
    public void execute() {
        Simulation sim = getActiveSimulation();
        sim.println("--- Starting Automation ---");


        // 1. PARAMETERS
        createGlobalParameters(sim);

        // 2. GEOMETRY
        GeometryPart myPart = createPartsGeneric(sim);
        if (myPart == null) return;

        // SCAN GEOMETRY & CREATE BOUNDARY PARAMETERS
        createDynamicParameters(sim, myPart);

        // 3. PHYSICS & MATERIALS
        setupPhysicsAndMaterials(sim);

        // INITIAL CONDITIONS
        setupInitialConditions(sim);

        // 4. REGION & BOUNDARIES
        setupRegionAndBoundaries(sim, myPart);

        // SOLVER SETUP
        setupSolverSettings(sim);

        // 5. MESH SETUP
        setupMesh(sim, myPart);

        // 6. CUSTOM CONTROLS
        // Retrieve the mesh op we just created to pass it to the control
        AutoMeshOperation meshOp = (AutoMeshOperation) sim.get(MeshOperationManager.class).getObject("Automated Mesh");
        addControlToAllSurfaces(sim, meshOp, myPart);

        // 7. GENERATE MESH
        sim.println("--- Generating Volume Mesh ---");
        try {
            sim.getMeshPipelineController().generateVolumeMesh();
        } catch (Exception e) {
            sim.println("Mesh generation failed. stopping.");
            return;
        }

        // RUN SOLVER
        runSolver(sim);

        // CREATE PLANES
        List<String> analysisPlanes = new ArrayList<>();

        try {
            analysisPlanes = setupSectionPlanes(sim);
        } catch (FileNotFoundException e) {
            sim.println("Warning: Could not create planes (File not found).");
        }

        // create uniformity index
        // Loop through every plane created from the input.txt file
        if (analysisPlanes.isEmpty()) {
            sim.println("--- No planes defined in input.txt. Skipping uniformity reports. ---");
        } else {
            for (String currentPlane : analysisPlanes) {
                sim.println("--- Generating Indices for Plane: " + currentPlane + " ---");

                createVelocityUniformityReport(sim, currentPlane);

                String deadCondition = "mag($$Velocity) < 0.5 ? 1.0 : 0.0";
                createCustomAreaReport(sim, currentPlane, "DeadAreaFraction", deadCondition);
                createMetricScene(sim, currentPlane, "DeadAreaFraction", "DeadAreaFraction_Func_" + currentPlane);

                String highVelCondition = "mag($$Velocity) > 1.0 ? 1.0 : 0.0";
                createCustomAreaReport(sim, currentPlane, "HighVelocityArea", highVelCondition);
                createMetricScene(sim, currentPlane, "High Velocity", "HighVelocityArea_Func_" + currentPlane);

                String safeZoneCondition = "($$Velocity[0] > 1.0) && ($$Velocity[0] < 4.0) ? 1.0 : 0.0";
                createCustomAreaReport(sim, currentPlane, "SafeVelocityZone", safeZoneCondition);
                createMetricScene(sim, currentPlane, "Safe Zone", "SafeVelocityZone_Func_" + currentPlane);

                createDeltaZIndexReport(sim, currentPlane);
                createMetricScene(sim, currentPlane, "Vertical Velocity", "AbsZVel_" + currentPlane);

                createTotalPerformanceIndex(sim, currentPlane, 0.30, 0.30, 0.15, 0.15, 0.10);
            }
        }
        
        sim.println("--- Automation Complete ---");
    }


    // ==========================================================
    // DYNAMIC PARAMETER CREATION
    // ==========================================================
    private void createDynamicParameters(Simulation sim, GeometryPart part) {
        sim.println("--- Mapping Configuration to Geometry ---");

        for (PartSurface surf : part.getPartSurfaces()) {
            String name = surf.getPresentationName().toLowerCase();

            // ----------------------------------------------------
            // CASE: INLET
            // ----------------------------------------------------
            if (name.contains("inlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);
                String key = "Inlet_Velocity_" + id;
                String dhKey = "Inlet_Hydraulic_Diameter_" + id;

                // CHECK: Do we have a pre-known value for this ID?
                if (configMap.containsKey(key)) {
                    double userValue = configMap.get(key);
                    createOrUpdateParameter(sim, key, userValue, "m/s");
                    sim.println("   -> Matched " + surf.getPresentationName() + " with configured value: " + userValue);
                } else {
                    // forgot to define this ID
                    // CRASH: Geometry exists, but Input File is missing the line!
                    sim.println("ERROR: Missing parameter for surface: " + surf.getPresentationName());
                    throw new RuntimeException("Input File Missing: " + key);
                }

                if (configMap.containsKey(dhKey)) {
                    double val = configMap.get(dhKey);
                    createOrUpdateParameter(sim, dhKey, val, "m");
                } else {
                    // CRASH: User forgot the diameter
                    sim.println("ERROR: Missing hydraulic diameter for: " + surf.getPresentationName());
                    throw new RuntimeException("Input File Missing: " + dhKey);
                }

            } else if (name.contains("outlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);
                String key = "Outlet_Pressure_" + id;
                String dhKey = "Outlet_Hydraulic_Diameter_" + id;

                if (configMap.containsKey(key)) {
                    double userValue = configMap.get(key);
                    createOrUpdateParameter(sim, key, userValue, "Pa");
                    sim.println("   -> Matched " + name + " with configured value: " + userValue);
                } else {
                    // CRASH
                    sim.println("ERROR: Missing parameter for surface: " + surf.getPresentationName());
                    throw new RuntimeException("Input File Missing: " + key);
                }

                if (configMap.containsKey(dhKey)) {
                    double val = configMap.get(dhKey);
                    createOrUpdateParameter(sim, dhKey, val, "m");
                } else {
                    // CRASH
                    sim.println("ERROR: Missing hydraulic diameter for: " + surf.getPresentationName());
                    throw new RuntimeException("Input File Missing: " + dhKey);
                }
            }
        }

    }

    // HELPER: EXTRACT ID
    private int extractIDFromName(String name) {
        String digit = name.replaceAll("\\D+", "");
        return digit.isEmpty() ? 1 : Integer.parseInt(digit);
    }

    // ----------------------------------------------------------
    // HELPER: PARAMETER
    // ----------------------------------------------------------
    private void createGlobalParameters(Simulation sim) {

        // Call the file reader FIRST
        readAndCreateParameters(sim);

        //TODO: This must be extracted  from inlet geometry
        //createOrGetParameter(sim, "Hydraulic_Diameter", 0.08, "m");
    }

    private void createOrUpdateParameter(Simulation sim, String name, double value, String unitString) {
        GlobalParameterManager gpm = sim.get(GlobalParameterManager.class);
        ScalarGlobalParameter param;

        // ----------------------------------------------------------------
        // 1. PRE-VALIDATION: Check if the unit exists BEFORE creating anything
        // ----------------------------------------------------------------
        Units foundUnit = null;

        if (unitString != null && !unitString.isEmpty()) {
            for (Units u : sim.getUnitsManager().getObjects()) {
                if (u.getPresentationName().equals(unitString)) {
                    foundUnit = u;
                    break;
                }
            }

            // If user gave a unit string (e.g. "m/ss") but we found nothing...
            if (foundUnit == null) {
                sim.println("ERROR: Input validation failed.");
                sim.println("       The unit '" + unitString + "' for parameter '" + name + "' does not exist.");
                sim.println("       Aborting creation to prevent corrupt parameters.");
                // CRASH HERE - Nothing has been created yet!
                throw new RuntimeException("Invalid Unit: " + unitString);
            }
        }

        // ----------------------------------------------------------------
        // 2. SAFE EXECUTION: Now we know the unit is valid (or null)
        // ----------------------------------------------------------------
        if (gpm.has(name)) {
            param = (ScalarGlobalParameter) gpm.getObject(name);
            sim.println("Updated: " + name);
        } else {
            param = (ScalarGlobalParameter) gpm.createGlobalParameter(ScalarGlobalParameter.class, name);
            sim.println("Created: " + name);
        }

        // 3. APPLY SETTINGS
        if (foundUnit != null) {
            // We already found it in step 1, so just apply it
            param.setDimensions(foundUnit.getDimensions());
            param.getQuantity().setUnits(foundUnit);
            param.getQuantity().setValueAndUnits(value, foundUnit);
        } else {
            // Dimensionless case
            param.getQuantity().setValue(value);
        }
    }

    private ScalarGlobalParameter getParamByName(Simulation sim, String paramName) {
        GlobalParameterManager gpm = sim.get(GlobalParameterManager.class);

        // Safety Check
        if (gpm.has(paramName)) {
            return (ScalarGlobalParameter) gpm.getObject(paramName);
        }

        // If not found, log a warning and return null
        sim.println("Warning: Global Parameter '" + paramName + "' was not found.");
        return null;
    }

    private void linkMaterialProperty(Gas gas, String propName, String paramName, Simulation sim) {
        MaterialProperty prop = (MaterialProperty) gas.getMaterialProperties().getObject(propName);
        ConstantMaterialPropertyMethod method = (ConstantMaterialPropertyMethod) prop.getMethod();
        ScalarGlobalParameter param = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject(paramName);
        method.getQuantity().setDefinition(param);
    }

    // ==========================================================
    // INITIAL CONDITION SETUP
    // ==========================================================
    private void setupInitialConditions(Simulation sim) {
        sim.println("--- Setting Initial Conditions ---");

        if (sim.getContinuumManager().getObjects().isEmpty()) return;
        PhysicsContinuum physics = (PhysicsContinuum) sim.getContinuumManager().getObjects().iterator().next();

        sim.println("   -> Applying conditions to: " + physics.getPresentationName());

        // set static pressure
        InitialPressureProfile pressProfile = physics.getInitialConditions().get(InitialPressureProfile.class);
        ScalarGlobalParameter initPressParam = getParamByName(sim, "Initial_Pressure");

        if (initPressParam != null) {
            ((ConstantScalarProfileMethod) pressProfile.getMethod()).getQuantity().setDefinition(initPressParam);
            sim.println("      -> Initial Pressure linked.");
        }

        // link turbulence
        try {
            // DYNAMIC SPECIFICATION OPTION
            if (turbulenceModel.equals("k-omega-sst")) {
                physics.getInitialConditions().get(KwTurbSpecOption.class).setSelected(KwTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);
            } else {
                physics.getInitialConditions().get(KeTurbSpecOption.class).setSelected(KeTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);
            }

            ScalarGlobalParameter turbParam = getParamByName(sim, "Turbulence_Intensity");
            if (turbParam != null) {
                TurbulenceIntensityProfile tiProfile = physics.getInitialConditions().get(TurbulenceIntensityProfile.class);
                ((ConstantScalarProfileMethod) tiProfile.getMethod()).getQuantity().setDefinition(turbParam);
            }

            // link length scale
            ScalarGlobalParameter lenParam = getParamByName(sim, "Initial_Turbulence_Length_Scale");
            if (lenParam != null) {
                TurbulentLengthScaleProfile tlsProfile = physics.getInitialConditions().get(TurbulentLengthScaleProfile.class);
                ((ConstantScalarProfileMethod) tlsProfile.getMethod()).getQuantity().setDefinition(lenParam);
            }
            sim.println("      -> Turbulence Initial Conditions set.");

        } catch (Exception e) {
            sim.println("      -> Warning: Turbulence settings skipped (Model mismatch).");
        }
    }

    // ==========================================================
    // GEOMETRY HELPER
    // ==========================================================
    private GeometryPart createPartsGeneric(Simulation sim) {
        if (!sim.getGeometryPartManager().getObjects().isEmpty()) {
            GeometryPart existingPart = sim.getGeometryPartManager().getObjects().iterator().next();
            sim.println("Info: Part already exists (" + existingPart.getPresentationName() + "). Skipping creation.");
            return existingPart;
        }

        SolidModelManager mgr = sim.get(SolidModelManager.class);
        if (mgr.getObjects().isEmpty()) {
            sim.println("Error: No 3D-CAD Models found.");
            return null;
        }
        CadModel cadModel = ((CadModel) sim.get(SolidModelManager.class).getObjects().iterator().next());
        Collection<Body> allBodies = cadModel.getBodyManager().getObjects();

        if (allBodies.isEmpty()) return null;

        sim.println("Creating parts from CAD: " + cadModel.getPresentationName());

        //star.cadmodeler.Body cadModelerBody = (star.cadmodeler.Body) cadModel.getBodyManager().getObjects().iterator().next();
        cadModel.createParts(allBodies, new ArrayList<>(Collections.emptyList()), true,
                false, 1, false, false,
                3, "SharpEdges", 30.0, 2, true,
                1.0E-5, false);

        if (!sim.getGeometryPartManager().getObjects().isEmpty()) {
            return sim.getGeometryPartManager().getObjects().iterator().next();
        }
        return null;
    }

    // ==========================================================
    // PHYSICS HELPER
    // ==========================================================
    private void setupPhysicsAndMaterials(Simulation sim) {
        sim.println("--- Setting up Physics ---");

        PhysicsContinuum physics;
        try {
            physics = (PhysicsContinuum) sim.getContinuumManager().getContinuum("Argon Physics");
        } catch (Exception e) {
            physics = sim.getContinuumManager().createContinuum(PhysicsContinuum.class);
            physics.setPresentationName("Argon Physics");

            physics.enable(ThreeDimensionalModel.class);
            physics.enable(SteadyModel.class);
            physics.enable(SingleComponentGasModel.class);
            physics.enable(CoupledFlowModel.class);
            physics.enable(ConstantDensityModel.class);
            physics.enable(TurbulentModel.class);
            physics.enable(RansTurbulenceModel.class);

            if(turbulenceModel.equals("k-omega-sst")) {
                physics.enable(KOmegaTurbulence.class);
                physics.enable(SstKwTurbModel.class);
                physics.enable(KwAllYplusWallTreatment.class);
                sim.println("   -> Applied Physics: k-omega SST with all Y+ wall treatment");
            } else {
                physics.enable(KEpsilonTurbulence.class);
                physics.enable(LienKeTurbModel.class);
                physics.enable(KeLowYplusWallTreatment.class);
                sim.println("   -> Applied Physics: k-epsilon (Lien Low-Re) with low Y+ wall treatment");
            }

            // TODO: not sure if this is necessary
            //physics.enable(SolutionInterpolationModel.class);

            // CONFIGURE ARGON
            // ------------------------------------------------
            SingleComponentGasModel gasModel = physics.getModelManager().getModel(SingleComponentGasModel.class);
            Gas argonGas = (Gas) gasModel.getMaterial();
            argonGas.setPresentationName("Argon");

            // Link Parameters
            linkMaterialProperty(argonGas, "Density", "Argon_Density", sim);
            linkMaterialProperty(argonGas, "Dynamic Viscosity", "Argon_Viscosity", sim);

            sim.println("--- Success: Physics Setup Complete ---");
        }
    }

    // ==========================================================
    // REGION & BOUNDARY HELPER
    // ==========================================================
    private void setupRegionAndBoundaries(Simulation sim, GeometryPart part) {
        sim.println("--- Setting up Regions & Boundaries ---");

        String partName = part.getPresentationName();

        if (sim.getRegionManager().has(partName)) {
            sim.println("Info: Region '" + partName + "' already exists. Skipping creation.");
            return;
        }

        // Assign Part
        sim.println("Creating new Region for part: " + partName);
        sim.getRegionManager().newRegionsFromParts(new ArrayList<>(List.<GeometryPart>of(part)), "OneRegionPerPart",
                null, "OneBoundaryPerPartSurface", null, RegionManager.CreateInterfaceMode.BOUNDARY,
                "OneEdgeBoundaryPerPart", null);

        sim.println("--- Classifying Boundaries ---");

        Region region = sim.getRegionManager().getRegion(partName);

        // Shared Turbulence Parameter
        ScalarGlobalParameter globalTurb = getParamByName(sim, "Turbulence_Intensity");

        for (Boundary b : region.getBoundaryManager().getBoundaries()) {
            String name = b.getPresentationName().toLowerCase();

            if (name.contains("inlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);

                String velName = "Inlet_Velocity_" + id;
                String dhName = "Inlet_Hydraulic_Diameter_" + id;

                ScalarGlobalParameter velParam = getParamByName(sim, velName);
                ScalarGlobalParameter dhParam = getParamByName(sim, dhName);

                //ScalarGlobalParameter velParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Inlet_Velocity");
                //ScalarGlobalParameter turbParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Turbulence_Intensity");
                //ScalarGlobalParameter dhParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Hydraulic_Diameter");

                // Safety check: verify we actually found them
                if (velParam != null) {
                    configureInlet(sim, b, velParam, globalTurb, dhParam);
                    sim.println("   -> Configured " + b.getPresentationName() + " using " + velName);
                } else {
                    sim.println("   ERROR: Could not find parameter " + velName + " for boundary " + b.getPresentationName());
                }

            } else if (name.contains("outlet") && !name.contains("wall")) {
                int id = extractIDFromName(name);

                String pressName = "Outlet_Pressure_" + id;
                String dhName = "Outlet_Hydraulic_Diameter_" + id;

                ScalarGlobalParameter pressParam = getParamByName(sim, pressName);
                ScalarGlobalParameter dhParam = getParamByName(sim, dhName);

                //ScalarGlobalParameter pressureParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Outlet_Pressure");
                //ScalarGlobalParameter turbParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Turbulence_Intensity");
                //ScalarGlobalParameter dhParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Hydraulic_Diameter");

                if (pressParam != null) {
                    configureOutlet(sim, b, pressParam, globalTurb, dhParam);
                    sim.println("   -> Configured " + b.getPresentationName() + " using " + pressName);
                } else {
                    sim.println("   ERROR: Could not find parameter " + pressName + " for boundary " + b.getPresentationName());
                }

            } else {
                b.setBoundaryType(WallBoundary.class);
            }
        }
    }

    // ==========================================================
    // CONFIGURE INLET HELPER
    // ==========================================================
    private void configureInlet(Simulation sim, Boundary boundary, ScalarGlobalParameter velocityParam,
                                ScalarGlobalParameter turbParam, ScalarGlobalParameter dhParam) {
        if (!(boundary.getBoundaryType().getClass().equals(InletBoundary.class))) {
            boundary.setBoundaryType(InletBoundary.class);
        }

        if (velocityParam != null) {
            VelocityMagnitudeProfile velProfile = boundary.getValues().get(VelocityMagnitudeProfile.class);
            ConstantScalarProfileMethod velMethod = (ConstantScalarProfileMethod) velProfile.getMethod();
            velMethod.getQuantity().setDefinition(velocityParam);
        }

        // configure turbulence
        if (turbulenceModel.equals("k-omega-sst")) {
            boundary.getConditions().get(KwTurbSpecOption.class).setSelected(KwTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);
        } else {
            boundary.getConditions().get(KeTurbSpecOption.class).setSelected(KeTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);
        }

        if (turbParam != null) {
            TurbulenceIntensityProfile tiProfile = boundary.getValues().get(TurbulenceIntensityProfile.class);
            ConstantScalarProfileMethod tiMethod = (ConstantScalarProfileMethod) tiProfile.getMethod();
            tiMethod.getQuantity().setDefinition(turbParam);
        }

        if (dhParam != null) {
            double dhValue = dhParam.getQuantity().getSIValue();
            double lengthScale = 0.07 * dhValue;
            TurbulentLengthScaleProfile tlProfile = boundary.getValues().get(TurbulentLengthScaleProfile.class);
            ConstantScalarProfileMethod tlMethod = (ConstantScalarProfileMethod) tlProfile.getMethod();
            tlMethod.getQuantity().setValue(lengthScale);
        }
    }

    // ==========================================================
    // CONFIGURE OUTLET HELPER
    // ==========================================================
    private void configureOutlet(Simulation sim, Boundary boundary, ScalarGlobalParameter pressureParam,
                                 ScalarGlobalParameter turbParam, ScalarGlobalParameter dhParam) {
        if (!boundary.getBoundaryType().getClass().equals(PressureBoundary.class)) {
            boundary.setBoundaryType(PressureBoundary.class);
        }

        if (pressureParam != null) {
            StaticPressureProfile pProfile = boundary.getValues().get(StaticPressureProfile.class);
            ConstantScalarProfileMethod pMethod = (ConstantScalarProfileMethod) pProfile.getMethod();
            pMethod.getQuantity().setDefinition(pressureParam);
        }

        // configure turbulence
        if (turbulenceModel.equals("k-omega-sst")) {
            boundary.getConditions().get(KwTurbSpecOption.class).setSelected(KwTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);
        } else {
            boundary.getConditions().get(KeTurbSpecOption.class).setSelected(KeTurbSpecOption.Type.INTENSITY_LENGTH_SCALE);
        }

        if (turbParam != null) {
            TurbulenceIntensityProfile tiProfile = boundary.getValues().get(TurbulenceIntensityProfile.class);
            ConstantScalarProfileMethod tiMethod = (ConstantScalarProfileMethod) tiProfile.getMethod();
            tiMethod.getQuantity().setDefinition(turbParam);
        }

        if (dhParam != null) {
            double dhValue = dhParam.getQuantity().getSIValue();
            double lengthScale = 0.07 * dhValue;
            TurbulentLengthScaleProfile tlProfile = boundary.getValues().get(TurbulentLengthScaleProfile.class);
            ConstantScalarProfileMethod tlMethod = (ConstantScalarProfileMethod) tlProfile.getMethod();
            tlMethod.getQuantity().setValue(lengthScale);
        }
    }

    // ==========================================================
    // MESH HELPER
    // ==========================================================
    private void setupMesh(Simulation sim, GeometryPart part) {
        sim.println("--- Setting up Mesh Operation ---");

        MeshOperationManager meshManager = sim.get(MeshOperationManager.class);
        AutoMeshOperation meshOp = null;

        // "star.bodyfittedmesher.AdvancingLayerAutoMesher"
        String[] mesherNames = new String[]{
                "star.resurfacer.ResurfacerAutoMesher",
                "star.dualmesher.DualAutoMesher",
                "star.prismmesher.PrismAutoMesher"
        };

        if (meshManager.has("Automated Mesh")) {
            meshOp = (AutoMeshOperation) meshManager.getObject("Automated Mesh");
            sim.println("Info: 'Automated Mesh' already exists.");
        } else {
            meshOp = meshManager.createAutoMeshOperation(new StringVector(mesherNames),
                    new ArrayList<>(Arrays.<GeometryPart>asList(part)));

            meshOp.setPresentationName("Automated Mesh");
            sim.println("Created: Automated Mesh (Meshers pre-selected)");
        }

        if (meshOp == null) return;

        ScalarGlobalParameter baseSizeParam = (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject("Base_Size");
        if (baseSizeParam != null) {
            meshOp.getDefaultValues().get(BaseSize.class).setDefinition(baseSizeParam);

            // CONFIGURE PRISM LAYERS
            meshOp.getDefaultValues().get(NumPrismLayers.class).getNumLayersValue().getQuantity().
                    setDefinition(getParamByName(sim, "Num_Prism_Layers"));

            meshOp.getDefaultValues().get(PrismLayerStretching.class).getStretchingQuantity().
                    setDefinition(getParamByName(sim, "Prism_Layer_Stretching"));

            PrismThickness plt = meshOp.getDefaultValues().get(PrismThickness.class);
            plt.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
            plt.getAbsoluteSizeValue().setDefinition(getParamByName(sim, "Prism_Layer_Thickness"));

            // SET VOLUME GROWTH RATE
            PartsTetPolyGrowthRate ptpgr = meshOp.getDefaultValues().get(PartsTetPolyGrowthRate.class);
            ScalarGlobalParameter growthParam = getParamByName(sim, "Mesh_Volume_Growth_Rate");
            if (growthParam != null) {
                double growthRateValue = growthParam.getQuantity().getSIValue();
                ptpgr.setGrowthRate(growthRateValue);
            }


        }

        sim.println("--- Mesh Setup Complete ---");
    }

    // ==========================================================
    // MESH SURFACE CONTROL HELPER
    // ==========================================================
    private void addControlToAllSurfaces(Simulation sim, AutoMeshOperation meshOp, GeometryPart part) {
        sim.println("--- Adding Surface Custom Control ---");

        Collection<PartSurface> allSurface = part.getPartSurfaces();

        // Get custom control manager
        CustomMeshControlManager controlManager = meshOp.getCustomMeshControls();

        SurfaceCustomMeshControl surfControl = null;

        // Create or retrieve custom control
        String controlName = "Control_All_Surfaces";

        if (controlManager.has(controlName)) {
            surfControl = (SurfaceCustomMeshControl) controlManager.getObject(controlName);
            sim.println("Info: Custom control '" + controlName + "' already exists.");
        } else {
            surfControl = controlManager.createSurfaceControl();
            surfControl.setPresentationName(controlName);

            surfControl.getGeometryObjects().setQuery(null);
            surfControl.getGeometryObjects().setObjects(allSurface);


            sim.println("Created: " + controlName + " (Applied to part: " + part.getPresentationName() + ")");
        }

        // --- CONFIGURE TARGET SIZE (50% Relative) ---
        surfControl.getCustomConditions().get(PartsTargetSurfaceSizeOption.class).setSelected(PartsTargetSurfaceSizeOption.Type.CUSTOM);
        PartsTargetSurfaceSize surfSize = surfControl.getCustomValues().get(PartsTargetSurfaceSize.class);
        surfSize.getRelativeSizeScalar().setValue(50.0);

        sim.println("   -> Configured Custom Control: " + controlName + " (Target: 50%)");
    }

    // ==========================================================
    // SOLVER SETTINGS
    // ==========================================================
    private void setupSolverSettings(Simulation sim) {
        sim.println("--- Setting Solver Parameters ---");

        SolverStoppingCriterionManager stopManager = sim.getSolverStoppingCriterionManager();
        StepStoppingCriterion stepStop;

        if (stopManager.has("Maximum Steps")) {
            stepStop = (StepStoppingCriterion) stopManager.getSolverStoppingCriterion("Maximum Steps");
            sim.println("   -> Found existing 'Maximum Steps' criterion.");
        } else {
            stepStop = (StepStoppingCriterion) stopManager.create("star.common.StepStoppingCriterion");
            sim.println("   -> Created new 'Maximum Steps' criterion.");
        }

        ScalarGlobalParameter maxStepsParameter = getParamByName(sim, "Max_Steps");

        if (maxStepsParameter != null) {
            double steps = maxStepsParameter.getQuantity().getSIValue();
            stepStop.getMaximumNumberStepsObject().getQuantity().setValue(steps);
            sim.println("   -> Created 'Maximum Steps' criterion set to: " + steps);
        } else {
            stepStop.getMaximumNumberStepsObject().getQuantity().setValue(1000.0);
            sim.println("   -> Created 'Maximum Steps' criterion set to default: 1000");
        }

        if (stopManager.has("Fixed Steps")) {
            FixedStepsStoppingCriterion fixedSteps = (FixedStepsStoppingCriterion) stopManager.getSolverStoppingCriterion("Fixed Steps");
            stopManager.removeObjects(fixedSteps);
            sim.println("   -> Removed default 'Fixed Steps' criterion.");
        }
    }

    // ==========================================================
    // SOLVER RUN
    // ==========================================================
    private void runSolver(Simulation sim) {
        sim.println("--- Initializing Solution ---");

        sim.getSolution().initializeSolution();

        sim.println("--- Starting Solver ---");
        try {
            sim.getSimulationIterator().run();
            sim.println("--- Solver Finished ---");
        } catch (Exception e) {
            sim.println("Error during calculation: " + e.getMessage());
        }
    }

    // ==========================================================
    // 1. READ PARAMETERS (STRICT NAMESPACE: "Param.")
    // ==========================================================
    private void readAndCreateParameters(Simulation sim) {
        sim.println("--- Reading Input File (Namespace: Param.) ---");

        File file = new File(sim.getSessionDir(), "input.txt");
        if (!file.exists()) {
            sim.println("Error: input.txt not found in session directory.");
            return;
        }

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = br.readLine()) != null) {
                String cleanLine = line.trim();

                // ---------------------------------------------------------
                // FILTER 1: Skip Empty Lines & Comments
                // ---------------------------------------------------------
                if (cleanLine.isEmpty() || cleanLine.startsWith("#")) continue;

                // ---------------------------------------------------------
                // FILTER 2: ONLY Process "Param." Lines
                // ---------------------------------------------------------
                if (!cleanLine.startsWith("Param.")) {
                    continue;
                }

                // ---------------------------------------------------------
                // PARSING LOGIC
                // ---------------------------------------------------------
                String[] parts = cleanLine.split("=");
                if (parts.length == 2) {

                    // 1. EXTRACT KEY & STRIP PREFIX
                    // "Param.Inlet_Velocity_1" -> "Inlet_Velocity_1"
                    String rawKey = parts[0].trim();
                    String finalKey = rawKey.substring(6); // Remove first 6 chars ("Param.")

                    // 2. EXTRACT VALUE & REMOVE INLINE COMMENTS
                    // "25.5 m/s # Comment" -> "25.5 m/s"
                    String valuePart = parts[1].split("#")[0].trim();


                    // STRICT STRING PARAMETER VALIDATION
                    if (finalKey.equalsIgnoreCase("Turbulence_Model")) {
                        String parsedModel =  valuePart.toLowerCase().trim();

                        if (parsedModel.equals("k-epsilon") || parsedModel.equals("k-omega-sst")) {
                            turbulenceModel = parsedModel;
                            sim.println("   -> Configured Turbulence Model: " + turbulenceModel);
                        } else {
                            sim.println("   -> WARNING: Unknown Turbulence_Model '" + parsedModel + "'.");
                            sim.println("      Allowed options are 'k-epsilon' or 'k-omega-sst'.");
                            sim.println("      Defaulting to 'k-epsilon' to prevent solver crash.");
                            turbulenceModel = "k-epsilon";
                        }
                        continue; // Skip the Double parsing logic
                    }

                    try {
                        // 3. SEPARATE VALUE AND UNIT
                        // Logic: The last chunk after the last space is likely the unit.
                        // Example: "25.5 m/s" -> Val="25.5", Unit="m/s"
                        // Example: "0.05"     -> Val="0.05", Unit=""

                        String[] valTokens = valuePart.split("\\s+"); // Split by whitespace
                        double val;
                        String unitStr = "";

                        if (valTokens.length >= 2) {
                            // Assumes format "Value Unit"
                            unitStr = valTokens[valTokens.length - 1]; // Last part is unit
                            // Join the rest back together (in case value has spaces, rare)
                            String valStr = valuePart.substring(0, valuePart.lastIndexOf(unitStr)).trim();
                            val = Double.parseDouble(valStr);
                        } else {
                            // Dimensionless scalar
                            val = Double.parseDouble(valuePart);
                        }

                        // 4. CREATE IN STAR-CCM+
                        createOrUpdateParameter(sim, finalKey, val, unitStr);

                        configMap.put(finalKey, val);

                    } catch (Exception e) {
                        sim.println("   Warning: Could not parse line: " + cleanLine);
                        sim.println("   Reason: " + e.getMessage());
                    }
                }
            }
        } catch (IOException e) {
            sim.println("Error reading file: " + e.getMessage());
        }
    }

    // ==========================================================
    // DYNAMIC PLANE CREATION
    // ==========================================================
    private List<String> setupSectionPlanes(Simulation sim) throws FileNotFoundException {
        sim.println("--- Creating Planes ---");

        // Map: PlaneName -> Vector
        Map<String, double[]> origins = new HashMap<>();
        Map<String, double[]> normals = new HashMap<>();

        List<String> createdPlanes = new ArrayList<>();

        File file = new File(sim.getSessionDir(), "input.txt");
        if (!file.exists()) return createdPlanes;

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;

            while ((line = br.readLine()) != null) {
                String cleanLine = line.trim();

                //STRICT FILTER: Only read lines starting with "Plane."
                if (!cleanLine.startsWith("Plane.")) continue;

                String[] parts = cleanLine.split("=");
                if (parts.length < 2) continue;

                String rawKey = parts[0].trim(); // "Plane.Mid_Section.Origin"
                String[] keyParts = rawKey.split("\\.");

                if (keyParts.length == 3) {
                    String name = keyParts[1]; //Mid_Section
                    String property = keyParts[2]; // origin or normal

                    double[] vec = parseVector(parts[1].split("#")[0].trim());

                    if (vec !=null) {
                        if (property.equalsIgnoreCase("Origin")) {
                            origins.put(name, vec);
                        } else if (property.equalsIgnoreCase("Normal")) {
                            normals.put(name, vec);
                        }
                    }
                }
            }

        } catch (IOException e) {
            sim.println("Error reading plane data: " + e.getMessage());
            throw new RuntimeException(e);
        }

        //create Planes
        for (String name : origins.keySet()) {
            if (normals.containsKey(name)) {
                createSinglePlane(sim, name, origins.get(name), normals.get(name));

                createdPlanes.add(name);
            }
        }

        return createdPlanes;
    }

    // ==========================================================
    // CREATE SINGLE PLANE
    // ==========================================================
    private void createSinglePlane(Simulation sim, String planeName, double[] origin, double[] normal) {
        sim.println("   -> Configuring Plane: " + planeName);

        // 1. GET UNITS (Needed for the first 3 parameters)
        // We retrieve the "meter" unit object
        Units unitM = sim.getUnitsManager().getObject("m");

        // 2. GET PART MANAGER
        PartManager partManager = sim.getPartManager();
        PlaneSection plane;

        // 3. CREATE OR UPDATE
        if (partManager.has(planeName)) {
            plane = (PlaneSection) partManager.getObject(planeName);
        } else {
            // Create Implicit Part (Using your recording's signature)
            plane = (PlaneSection) partManager.createImplicitPart(
                    new ArrayList<>(Collections.<NamedObject>emptyList()),
                    new DoubleVector(new double[] {0.0, 0.0, 1.0}),
                    new DoubleVector(new double[] {0.0, 0.0, 0.0}),
                    0, 1, new DoubleVector(new double[] {0.0}),
                    null
            );
            plane.setPresentationName(planeName);
        }

        // 4. ASSIGN REGION
        if (sim.getRegionManager().getRegions().isEmpty()) {
            sim.println("      Error: No Regions found! Plane is empty.");
            return;
        }
        Region region = sim.getRegionManager().getRegions().iterator().next();
        plane.getInputParts().setObjects(region);

        // 5. SET GEOMETRY
        // Signature: setCoordinate(UnitX, UnitY, UnitZ, VectorValue)

        // -- Origin --
        plane.getOriginCoordinate().setCoordinate(
                unitM, unitM, unitM, // Units for X, Y, Z
                new DoubleVector(new double[] {origin[0], origin[1], origin[2]}) // The Vector
        );

        // -- Normal --
        // Note: Normal vectors are technically dimensionless, but the API often accepts 'm'
        // or checks the magnitude. Using 'm' is usually safe for direction vectors here.
        plane.getOrientationCoordinate().setCoordinate(
                unitM, unitM, unitM,
                new DoubleVector(new double[] {normal[0], normal[1], normal[2]})
        );

        sim.println("      -> Applied Geometry: Origin=" + Arrays.toString(origin));
    }

    private double[] parseVector(String vectorstring) {
        if (vectorstring == null || vectorstring.isEmpty()) return null;

        try {
            // Split by comma
            String[] parts = vectorstring.split(",");

            if (parts.length == 3) {
                return new double[]{
                        Double.parseDouble(parts[0]),
                        Double.parseDouble(parts[1]),
                        Double.parseDouble(parts[2])
                };
            }
        } catch (RuntimeException e) {
            throw new RuntimeException(e);
        }
        return null;
    }

    // ==========================================================
    // CREATE FLOW HOMOGENEITY (UNIFORMITY INDEX) REPORT
    // ==========================================================
    private void createVelocityUniformityReport(Simulation sim, String surfaceName) {
        sim.println("--- Setting up Uniformity Report for: " + surfaceName + " ---");



        PartManager partManager = sim.getPartManager();
        if (!partManager.has(surfaceName)) {
            sim.println("   ERROR: Could not find a part named '" + surfaceName + "'. Report skipped.");
            return;
        }

        Part surfacePart = (Part) partManager.getObject(surfaceName);

        ReportManager reportManager = sim.getReportManager();
        String reportName = "Uniformity_" + surfaceName;
        SurfaceUniformityReport gammaReport;

        if (reportManager.has(reportName)) {
            gammaReport = (SurfaceUniformityReport) reportManager.getObject(reportName);
            sim.println("   -> Updating existing report.");
        } else {
            gammaReport = reportManager.createReport(SurfaceUniformityReport.class);
            gammaReport.setPresentationName(reportName);
            sim.println("   -> Created new Uniformity Index report.");
        }

        gammaReport.getParts().setObjects(surfacePart);

        FieldFunction velocityFunc = sim.getFieldFunctionManager().getFunction("Velocity");
        FieldFunction velMagnitude = velocityFunc.getMagnitudeFunction();

        gammaReport.setFieldFunction(velMagnitude);

        sim.println("   -> Success! Report created for " + surfaceName);

        // create a monitor
        // TODO: creat a plot
        if (!sim.getMonitorManager().has(reportName + " Monitor")) {
            gammaReport.createMonitor();
        }

        sim.println("   -> Success! Monitor created for " + surfaceName);
    }

    // ==========================================================
    // CREATE GENERIC AREA FRACTION REPORT
    // ==========================================================
    private void createCustomAreaReport(Simulation sim, String surfaceName, String metricName, String definition) {
        sim.println("--- Setting up " + metricName + " Report for: " + surfaceName + " ---");

        PartManager partManager = sim.getPartManager();
        if (!partManager.has(surfaceName)) {
            sim.println("      ERROR: Could not find part '" + surfaceName + "'.");
            return;
        }

        Part surfacePart = (Part) partManager.getObject(surfaceName);

        String ffName = metricName + "_Func_" + surfaceName;
        FieldFunctionManager ffManager = sim.getFieldFunctionManager();
        UserFieldFunction customAreaFunc;

        if (ffManager.has(ffName)) {
            customAreaFunc = (UserFieldFunction) ffManager.getObject(ffName);
        } else {
            customAreaFunc = ffManager.createFieldFunction();
            customAreaFunc.setPresentationName(ffName);
            customAreaFunc.setFunctionName(ffName);
        }

        customAreaFunc.setDefinition(definition);

        ReportManager reportManager = sim.getReportManager();
        String reportName = metricName + "_" + surfaceName;
        AreaAverageReport customAreaReport;

        if (reportManager.has(reportName)) {
            customAreaReport = (AreaAverageReport) reportManager.getReport(reportName);
            sim.println("      -> Updating existing report.");
        } else {
            customAreaReport = reportManager.createReport(AreaAverageReport.class);
            customAreaReport.setPresentationName(reportName);
            sim.println("      -> Created new Area Fraction report.");
        }

        customAreaReport.getParts().setObjects(surfacePart);
        customAreaReport.setFieldFunction(customAreaFunc);

        sim.println("      -> Math evaluated: " + definition);
    }

    // ==========================================================
    // CREATE DELTA Z INDEX REPORT (|v_z| / |v|)
    // ==========================================================
    private void createDeltaZIndexReport(Simulation sim, String surfaceName) {
        sim.println("--- Setting up Delta Z Index Report for: " + surfaceName + " ---");

        PartManager partManager = sim.getPartManager();
        if (!partManager.has(surfaceName)) {
            sim.println("      ERROR: Could not find part '" + surfaceName + "'.");
            return;
        }

        Part surfacePart = (Part) partManager.getObject(surfaceName);

        FieldFunctionManager ffManager = sim.getFieldFunctionManager();
        ReportManager reportManager = sim.getReportManager();

        // Z-Velocity
        String zVelFuncName = "AbsZVel_" + surfaceName;
        UserFieldFunction zVelFunc;
        if (ffManager.has(zVelFuncName)) {
            zVelFunc = (UserFieldFunction) ffManager.getObject(zVelFuncName);
        } else {
            zVelFunc = ffManager.createFieldFunction();
            zVelFunc.setPresentationName(zVelFuncName);
            zVelFunc.setFunctionName(zVelFuncName);
            zVelFunc.setDefinition("abs($$Velocity[1])");
        }

        // Velocity Magnitude
        String magVelFuncName = "MagVel_" + surfaceName;
        UserFieldFunction magVelFunc;
        if (ffManager.has(magVelFuncName)) {
            magVelFunc = (UserFieldFunction) ffManager.getObject(magVelFuncName);
        } else {
            magVelFunc = ffManager.createFieldFunction();
            magVelFunc.setPresentationName(magVelFuncName);
            magVelFunc.setFunctionName(magVelFuncName);
            magVelFunc.setDefinition("mag($$Velocity)");
        }

        // Z-Velocity Area Average Report
        String zVelReportName = "AvgAbsZVel_" + surfaceName;
        AreaAverageReport zVelReport;
        if (reportManager.has(zVelReportName)) {
            zVelReport = (AreaAverageReport) reportManager.getReport(zVelReportName);
        } else {
            zVelReport = reportManager.createReport(AreaAverageReport.class);
            zVelReport.setPresentationName(zVelReportName);
        }
        zVelReport.getParts().setObjects(surfacePart);
        zVelReport.setFieldFunction(zVelFunc);

        // Velocity Magnitude Area Average Report
        String magVelReportName = "AvgMagVel_" +  surfaceName;
        AreaAverageReport magVelReport;
        if (reportManager.has(magVelReportName)) {
            magVelReport = (AreaAverageReport) reportManager.getReport(magVelReportName);
        } else {
            magVelReport = reportManager.createReport(AreaAverageReport.class);
            magVelReport.setPresentationName(magVelReportName);
        }
        magVelReport.getParts().setObjects(surfacePart);
        magVelReport.setFieldFunction(magVelFunc);

        // Final Delta Z Expression Report
        String finalReportName = "DeltaZ_Index_" + surfaceName;
        ExpressionReport deltaZReport;
        if (reportManager.has(finalReportName)) {
            deltaZReport = (ExpressionReport) reportManager.getReport(finalReportName);
            sim.println("   -> Updating existing Delta Z report.");
        } else {
            deltaZReport = reportManager.createReport(ExpressionReport.class);
            deltaZReport.setPresentationName(finalReportName);
            sim.println("   -> Created new Delta Z Expression report.");
        }

        String equation = "${" + zVelReportName + "} / ${" + magVelReportName + "}";
        deltaZReport.setDefinition(equation);
    }

    private void createTotalPerformanceIndex(Simulation sim, String surfaceName, double wUnif,
                                            double wSafe, double wDead, double wHigh, double wZVel) {
        sim.println("--- Setting up Total Performance Report for: " + surfaceName + " ---");

        ReportManager reportManager = sim.getReportManager();

        String unifName = "Uniformity_" + surfaceName;
        String safeName = "SafeVelocityZone_" + surfaceName;
        String deadName = "DeadAreaFraction_" + surfaceName;
        String highName = "HighVelocityArea_" + surfaceName;
        String deltaZName = "DeltaZ_Index_" + surfaceName;

        String totalReportName = "Total_Performance_" + surfaceName;

        double sum = wUnif + wSafe + wDead + wHigh + wZVel;
        if (Math.abs(sum - 1.0) > 1.0e-5) {
            sim.println("      ERROR: The weighting factors sum to " + sum + ", not 1.0!");

            if (reportManager.has(totalReportName)) {
                ExpressionReport totalReport = (ExpressionReport) reportManager.getReport(totalReportName);
                totalReport.setPresentationName(totalReportName);
                totalReport.setDefinition("0.0/0.0");
            }

            sim.println("      -> Aborting update.");
            return;
        }

        ExpressionReport totalReport;
        if (reportManager.has(totalReportName)) {
            totalReport = (ExpressionReport) reportManager.getReport(totalReportName);
            sim.println("   -> Updating existing Total Performance report.");
        } else {
            totalReport = reportManager.createReport(ExpressionReport.class);
            totalReport.setPresentationName(totalReportName);
            sim.println("   -> Created new Total Performance Expression report.");
        }

        String equation =
                "(" + wUnif + " * ${" + unifName + "}) + " +
                        "(" + wSafe + " * ${" + safeName + "}) + " +
                        "(" + wDead + " * (1.0 - ${" + deadName + "})) + " +
                        "(" + wHigh + " * (1.0 - ${" + highName + "})) + " +
                        "(" + wZVel + " * (1.0 - ${" + deltaZName + "}))";

        totalReport.setDefinition(equation);

        String monitorName = totalReportName + " Monitor";
        if (!sim.getMonitorManager().has(monitorName)) {
            totalReport.createMonitor();
            sim.println("   -> Success! Monitor created for " + totalReportName);
        }
    }

    // ==========================================================
    // CREATE VISUALIZATION SCENES FOR METRICS
    // ==========================================================
    private void createMetricScene(Simulation sim, String surfaceName, String metricName, String ffName) {
        sim.println("--- Building Scene for: " + metricName + " ---");

        if (!sim.getPartManager().has(surfaceName)) {
            sim.println("   ERROR: Plane '" + surfaceName + "' not found.");
            return;
        }
        PlaneSection planeSection = (PlaneSection) sim.getPartManager().getObject(surfaceName);

        if (!sim.getFieldFunctionManager().has(ffName)) {
            sim.println("   ERROR: Field function '" + ffName + "' not found.");
            return;
        }
        UserFieldFunction fieldFunction = (UserFieldFunction) sim.getFieldFunctionManager().getFunction(ffName);

        String sceneName = metricName + " on " + surfaceName;
        SceneManager sceneManager = sim.getSceneManager();
        Scene scene;

        if (sceneManager.has(sceneName)) {
            scene = sceneManager.getScene(sceneName);
        } else {
            scene = (Scene) sceneManager.createScene("Scalar");
            scene.setPresentationName(sceneName);
        }

        scene.resetCamera();

        // Discover the displayer at runtime — don't assume its name
        ScalarDisplayer scalarDisplayer = null;
        for (Displayer d : scene.getDisplayerManager().getObjects()) {
            if (d instanceof ScalarDisplayer) {
                scalarDisplayer = (ScalarDisplayer) d;
                sim.println("   -> Found displayer: " + d.getPresentationName());
                break;
            }
        }
        if (scalarDisplayer == null) {
            scalarDisplayer = (ScalarDisplayer) scene.getDisplayerManager().createScalarDisplayer("Scalar");
            sim.println("   -> Created new ScalarDisplayer.");
        }

        scalarDisplayer.getInputParts().setQuery(null);
        scalarDisplayer.getInputParts().setObjects(planeSection);

        ScalarDisplayQuantity qty = scalarDisplayer.getScalarDisplayQuantity();
        qty.setFieldFunction(fieldFunction);
        qty.setRange(new double[]{0.0, 1.0});

        scene.getSceneUpdate().getHardcopyProperties().setCurrentResolutionWidth(1523);
        scene.getSceneUpdate().getHardcopyProperties().setCurrentResolutionHeight(528);

        sim.println("   -> Success! Scene configured: " + sceneName);
    }
}